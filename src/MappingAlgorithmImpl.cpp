#include "drone_mapper/MappingAlgorithmImpl.h"
#include "drone_mapper/IMap3D.h"
#include "drone_mapper/Units.h"
#include <cmath>
#include <numbers>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

namespace {

// Keeps an angle difference within (-180, 180] degrees so callers can pick
// the shorter rotation direction (Left for positive, Right for negative).
double normalizeAngleDeg(double degrees) {
    while (degrees > 180.0) {
        degrees -= 360.0;
    }
    while (degrees <= -180.0) {
        degrees += 360.0;
    }
    return degrees;
}

// Rounds a position to the map's grid so repeated visits collapse onto the
// same key despite tiny floating-point drift from trigonometric movement.
std::array<long long, 3> gridKey(const Position3D& p, double resolution_cm) {
    const double x = p.x.force_numerical_value_in(cm);
    const double y = p.y.force_numerical_value_in(cm);
    const double z = p.z.force_numerical_value_in(cm);
    auto toIdx = [resolution_cm](double v) {
        return static_cast<long long>(std::llround(v / resolution_cm));
    };
    return {toIdx(x), toIdx(y), toIdx(z)};
}

// MockLidar's concentric-ring beams can offset at most just under 90 degrees
// from the scan direction they're given (the offset angle is an atan2 that
// saturates below 90 as the ring radius grows), so a single scan call only
// ever covers a forward hemisphere around whatever direction it's aimed at.
// Requesting all 6 axis directions once per newly-reached cell gives full
// spherical coverage from that vantage point instead of leaving whatever is
// behind the drone's arrival heading permanently unresolved.
const std::array<Orientation, 6> kPanoramaDirections = {
    Orientation{0.0 * deg, 0.0 * deg},
    Orientation{90.0 * deg, 0.0 * deg},
    Orientation{180.0 * deg, 0.0 * deg},
    Orientation{270.0 * deg, 0.0 * deg},
    Orientation{0.0 * deg, 90.0 * deg},
    Orientation{0.0 * deg, -90.0 * deg},
};

} // namespace

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan; 

    types::MappingStepCommand cmd;
    types::MovementCommand move_cmd;
    
    // אם סיימנו - נמסור פקודת עמידה במקום
    if (is_finished_) {
    // נחזיר פקודת רחף (Hover) כדי להישאר יציבים במקום
    move_cmd.type = types::MovementCommandType::Hover;
    move_cmd.distance = 0.0 * cm;
    cmd.movement = move_cmd;
    // Without this, DroneControlImpl never sees Finished/Completed and the
    // mission just hovers in place until it burns through every allotted
    // step (status ends up MaxSteps instead of Completed).
    cmd.status = types::AlgorithmStatus::Finished;
    return cmd;
}

// 2. לוגיקת יצירת יעדים בטוחה:
// Expand from wherever the drone actually is *every time it reaches a new
// cell*, not only once the whole queue has drained. With 6 directions
// expanding at once, the queue rarely empties, so gating expansion on
// "queue is empty" meant only whichever cell happened to be current at that
// rare moment ever got to spawn its own neighbors -- most reached cells
// never expanded at all, leaving large stretches of reachable space
// unexplored. expanded_cells_ makes this a proper multi-source flood fill:
// each cell expands exactly once, the first time it's the current position.
double r = output_map_.getMapConfig().resolution.force_numerical_value_in(cm);
auto current_key = gridKey(state.position, r);

// Every subsequent return in this function requests a scan too, so mapping
// keeps progressing opportunistically while traveling between cells, not
// just during a cell's dedicated panorama below.
cmd.scan_orientation = Orientation{0.0 * deg, 0.0 * deg};

if (panorama_started_cells_.insert(current_key).second) {
    for (const auto& direction : kPanoramaDirections) {
        pending_scan_orientations_.push(direction);
    }
}

if (!pending_scan_orientations_.empty()) {
    Orientation next_scan = pending_scan_orientations_.front();
    pending_scan_orientations_.pop();

    move_cmd.type = types::MovementCommandType::Hover;
    move_cmd.distance = 0.0 * cm;
    cmd.movement = move_cmd;
    cmd.scan_orientation = next_scan;
    return cmd;
}

if (expanded_cells_.insert(current_key).second) {
    visited_cells_.insert(current_key);

    double px = state.position.x.force_numerical_value_in(cm);
    double py = state.position.y.force_numerical_value_in(cm);
    double pz = state.position.z.force_numerical_value_in(cm);

    // רשימת כיוונים אפשריים (כולל למעלה/למטה כדי לסרוק גם בציר Z)
    std::vector<Position3D> candidates = {
        {(px + r) * cm, py * cm, pz * cm},
        {(px - r) * cm, py * cm, pz * cm},
        {px * cm, (py + r) * cm, pz * cm},
        {px * cm, (py - r) * cm, pz * cm},
        {px * cm, py * cm, (pz + r) * cm},
        {px * cm, py * cm, (pz - r) * cm}
    };

    for (const auto& p : candidates) {
        // בדיקת גבולות קפדנית לפני הכנסה לתור
        if (!output_map_.isInBounds(p)) {
            continue;
        }
        // A neighbor already known to be Empty is still worth visiting: its
        // own occupancy is settled, but the space beyond it might not be.
        // Only Occupied voxels are non-traversable; a visited-set (below)
        // is what keeps this from re-queuing cells forever.
        if (output_map_.atVoxel(p) == types::VoxelOccupancy::Occupied) {
            continue;
        }
        if (visited_cells_.insert(gridKey(p, r)).second) {
            target_queue_.push(p);
        }
    }
}

// אם עדיין אין יעדים, זה הזמן לסמן סיום
if (target_queue_.empty() && !pending_target_.has_value()) {
    is_finished_ = true;
    move_cmd.type = types::MovementCommandType::Hover;
    cmd.movement = move_cmd;
    cmd.status = types::AlgorithmStatus::Finished;
    return cmd;
}

    if (!pending_target_.has_value()) {
        pending_target_ = target_queue_.front();
        target_queue_.pop();
    }
    Position3D next_target = pending_target_.value();

    if (output_map_.atVoxel(next_target) == types::VoxelOccupancy::Occupied) {
        // Give up on this target and let the next call pick a fresh one.
        pending_target_.reset();
        move_cmd.type = types::MovementCommandType::Hover;
        move_cmd.distance = 0.0 * cm;

        cmd.movement = move_cmd;
        return cmd;
    }

    // התיקון: חילוץ בטוח של הערכים כדי לחשב מרחק ללא התערבות של mp_units
    double tx = next_target.x.force_numerical_value_in(cm);
    double ty = next_target.y.force_numerical_value_in(cm);
    double tz = next_target.z.force_numerical_value_in(cm);
    double cx = state.position.x.force_numerical_value_in(cm);
    double cy = state.position.y.force_numerical_value_in(cm);
    double cz = state.position.z.force_numerical_value_in(cm);

    double dx = tx - cx;
    double dy = ty - cy;
    double dz = tz - cz;

    // The queue is FIFO across position changes, so by the time a target is
    // dispatched the drone may have since moved on another axis, making a
    // once vertical-only (or horizontal-only) candidate now differ from the
    // current position on more than one axis at once. No single movement
    // command can close all axes together, so close them one at a time:
    // Elevate (Z, heading-independent) first, then rotate-to-face + Advance
    // (X/Y) once Z is settled. pending_target_ is only cleared once every
    // axis is within tolerance, so remaining axes are retried next call.
    constexpr double kAxisEpsilonCm = 1e-6;
    if (std::abs(dz) >= kAxisEpsilonCm) {
        move_cmd.type = types::MovementCommandType::Elevate;
        move_cmd.distance = dz * cm;

        cmd.movement = move_cmd;
        if (std::abs(dx) < kAxisEpsilonCm && std::abs(dy) < kAxisEpsilonCm) {
            pending_target_.reset();
        }
        return cmd;
    }

    double distance_to_target = std::sqrt(dx*dx + dy*dy);

    // Advance always moves the drone along its CURRENT heading (see
    // MockMovement::advance), not toward an arbitrary (x, y). The candidate
    // above was only verified to be in-bounds for the drone facing the
    // right direction, so we must rotate to face it before advancing --
    // otherwise the drone travels `distance_to_target` along whatever
    // heading it already had and can walk straight out of the map.
    double target_heading_deg = std::atan2(dy, dx) * 180.0 / std::numbers::pi;
    double current_heading_deg = state.heading.horizontal.force_numerical_value_in(deg);
    double heading_diff_deg = normalizeAngleDeg(target_heading_deg - current_heading_deg);

    constexpr double kHeadingEpsilonDeg = 1e-6;
    if (std::abs(heading_diff_deg) > kHeadingEpsilonDeg) {
        move_cmd.type = types::MovementCommandType::Rotate;
        move_cmd.rotation = heading_diff_deg >= 0 ? types::RotationDirection::Left
                                                   : types::RotationDirection::Right;
        move_cmd.angle = std::abs(heading_diff_deg) * deg;
        move_cmd.distance = 0.0 * cm;

        cmd.movement = move_cmd;
        return cmd;
    }

    move_cmd.type = types::MovementCommandType::Advance;
    move_cmd.distance = distance_to_target * cm;

    cmd.movement = move_cmd;
    pending_target_.reset();
    return cmd;
}

} // namespace drone_mapper