#include "drone_mapper/DroneControlImpl.h"
#include "drone_mapper/ScanResultToVoxels.h"
#include "drone_mapper/Units.h"

#include <array>
#include <utility>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

namespace {

// MockLidar's concentric-ring beams can offset at most just under 90 degrees
// from the scan direction it's given (the offset angle is an atan2 that
// saturates below 90 as the ring radius grows), so a single scan call only
// ever covers a forward hemisphere around wherever the drone is currently
// facing. Scanning along all 6 axis directions each step gives full
// spherical coverage regardless of the drone's heading, instead of leaving
// whatever is behind the drone's current facing perpetually unresolved.
const std::array<Orientation, 6> kScanDirections = {
    Orientation{0.0 * deg, 0.0 * deg},
    Orientation{90.0 * deg, 0.0 * deg},
    Orientation{180.0 * deg, 0.0 * deg},
    Orientation{270.0 * deg, 0.0 * deg},
    Orientation{0.0 * deg, 90.0 * deg},
    Orientation{0.0 * deg, -90.0 * deg},
};

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm,
                                   types::LidarConfigData lidar_config)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm),
      lidar_config_(std::move(lidar_config)) {}

types::DroneStepResult DroneControlImpl::step() {
    types::DroneState current_state = state();

    if (step_index_ >= mission_.max_steps) {
        return types::DroneStepResult{types::DroneStepStatus::Error, "Max steps reached"};
    }

    // Scan in all 6 axis directions relative to the current heading;
    // MockLidar adds the drone's actual heading internally to get the
    // world-space beam direction, so passing the heading again here would
    // double-rotate it.
    types::LidarScanResult scan_result;
    for (const Orientation& direction : kScanDirections) {
        types::LidarScanResult partial = lidar_.scan(direction);
        scan_result.insert(scan_result.end(), partial.begin(), partial.end());
    }

    // Record what the scan observed into the output map before deciding the
    // next move, so the mapping algorithm can tell already-explored voxels
    // apart from unmapped ones.
    ScanResultToVoxels::applyToMap(output_map_, current_state.position, current_state.heading,
                                   scan_result, lidar_config_);

    types::MappingStepCommand next_move = mapping_algorithm_.nextStep(current_state, &scan_result);

    if (next_move.movement.has_value()) {
        auto move_cmd = next_move.movement.value();
        if (move_cmd.type == types::MovementCommandType::Advance) {
            movement_.advance(move_cmd.distance);
        } else if (move_cmd.type == types::MovementCommandType::Rotate) {
            movement_.rotate(move_cmd.rotation, move_cmd.angle);
        } else if (move_cmd.type == types::MovementCommandType::Elevate) {
            movement_.elevate(move_cmd.distance);
        }
    }

    step_index_++;
    
    if (next_move.status == types::AlgorithmStatus::Finished || 
        next_move.status == types::AlgorithmStatus::FinishedWithUnmappableVoxels) {
        return types::DroneStepResult{types::DroneStepStatus::Completed, "Finished"};
    }

    return types::DroneStepResult{types::DroneStepStatus::Continue, "Step completed"};
}

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper