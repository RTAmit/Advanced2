#include "MappingAlgorithmImpl.h"

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map)
    : IMappingAlgorithm(drone_config, output_map) {}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state, 
                                                         const types::LidarScanResult* latest_scan) {
    (void)state;
    (void)latest_scan;
    
    if (consecutive_advances_ < 10) {
        consecutive_advances_++;
        return types::MappingStepCommand::Advance; 
    } else {
        consecutive_advances_ = 0;
        return types::MappingStepCommand::RotateRight; 
    }
}

} // namespace drone_mapper