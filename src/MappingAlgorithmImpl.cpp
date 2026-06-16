#include "MappingAlgorithmImpl.h"

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map)
    : IMappingAlgorithm(drone_config, output_map) {}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state, 
                                                         const types::LidarScanResult* latest_scan) {
    (void)state;
    (void)latest_scan;
    
    types::MappingStepCommand cmd;
    
    if (consecutive_advances_ < 10) {
        consecutive_advances_++;
        // מגדירים תנועה קדימה (ללא זווית סיבוב)
        cmd.movement = types::MovementCommand{
            types::MovementCommandType::Advance, types::RotationDirection::Left, 0.0*deg, 10.0*cm
        };
    } else {
        consecutive_advances_ = 0;
        // מגדירים סיבוב ימינה
        cmd.movement = types::MovementCommand{
            types::MovementCommandType::Rotate, types::RotationDirection::Right, 90.0*deg, 0.0*cm
        };
    }
    
    return cmd;
}

} // namespace drone_mapper