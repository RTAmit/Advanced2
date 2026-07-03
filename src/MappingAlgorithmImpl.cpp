#include "drone_mapper/MappingAlgorithmImpl.h"
#include "drone_mapper/IMap3D.h"
#include "drone_mapper/Units.h"
#include <cmath>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(types::DroneConfigData drone_config, const IMap3D& output_map)
    : IMappingAlgorithm(drone_config, output_map), is_finished_(false) {}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state, 
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan; 

    types::MappingStepCommand cmd;
    types::MovementCommand move_cmd;
    
    // אם סיימנו - נמסור פקודת עמידה במקום
    if (is_finished_) {
        move_cmd.type = types::MovementCommandType::Rotate;
        move_cmd.rotation = types::RotationDirection::Left;
        move_cmd.angle = 0.0 * deg;
        move_cmd.distance = 0.0 * cm;
        
        cmd.movement = move_cmd;
        return cmd;
    }

    // אם התור ריק, נייצר נקודות חדשות מסביב
    if (target_queue_.empty()) {
        // התיקון: חילוץ הערכים המספריים (double) כדי למנוע שגיאות חיבור יחידות
        double r = _output_map.getMapConfig().resolution.force_numerical_value_in(cm);
        double px = state.position.x.force_numerical_value_in(cm);
        double py = state.position.y.force_numerical_value_in(cm);
        double pz = state.position.z.force_numerical_value_in(cm);
        
        // יצירת הנקודות בבטחה אחרי החישוב המתמטי
        Position3D p1 = {(px + r) * cm, py * cm, pz * cm};
        Position3D p2 = {(px - r) * cm, py * cm, pz * cm};
        Position3D p3 = {px * cm, (py + r) * cm, pz * cm};
        Position3D p4 = {px * cm, (py - r) * cm, pz * cm};

        for (const auto& p : {p1, p2, p3, p4}) {
            if (_output_map.isInBounds(p) && _output_map.atVoxel(p) == types::VoxelOccupancy::Unmapped) {
                target_queue_.push(p);
            }
        }

        if (target_queue_.empty()) {
            is_finished_ = true;
            move_cmd.type = types::MovementCommandType::Rotate;
            move_cmd.rotation = types::RotationDirection::Left;
            move_cmd.angle = 0.0 * deg;
            move_cmd.distance = 0.0 * cm;
            
            cmd.movement = move_cmd;
            return cmd;
        }
    }

    Position3D next_target = target_queue_.front();
    target_queue_.pop();

    if (_output_map.atVoxel(next_target) == types::VoxelOccupancy::Occupied) {
        move_cmd.type = types::MovementCommandType::Rotate;
        move_cmd.rotation = types::RotationDirection::Left;
        move_cmd.angle = 0.0 * deg;
        move_cmd.distance = 0.0 * cm;
        
        cmd.movement = move_cmd;
        return cmd;
    }

    // התיקון: חילוץ בטוח של הערכים כדי לחשב מרחק ללא התערבות של mp_units
    double tx = next_target.x.force_numerical_value_in(cm);
    double ty = next_target.y.force_numerical_value_in(cm);
    double cx = state.position.x.force_numerical_value_in(cm);
    double cy = state.position.y.force_numerical_value_in(cm);
    
    double dx = tx - cx;
    double dy = ty - cy;
    double distance_to_target = std::sqrt(dx*dx + dy*dy);

    move_cmd.type = types::MovementCommandType::Advance;
    move_cmd.rotation = types::RotationDirection::Left;
    move_cmd.angle = 0.0 * deg;
    move_cmd.distance = distance_to_target * cm;
    
    cmd.movement = move_cmd;
    return cmd;
}

} // namespace drone_mapper