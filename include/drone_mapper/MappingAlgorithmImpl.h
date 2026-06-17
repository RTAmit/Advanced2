#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

namespace drone_mapper {

class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map);
    
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state, 
                                                     const types::LidarScanResult* latest_scan) override;

private:
    int consecutive_advances_ = 0; 
};

} // namespace drone_mapper