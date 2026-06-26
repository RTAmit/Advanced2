#pragma once
#include <drone_mapper/IMappingAlgorithm.h>
#include <queue>

namespace drone_mapper {

class MappingAlgorithmImpl : public IMappingAlgorithm {
public:
    MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map);

    types::MappingStepCommand nextStep(const types::DroneState& state, 
                                       const types::LidarScanResult* latest_scan) override;

private:
    std::queue<Position3D> target_queue_;
    bool is_finished_ = false;
    int consecutive_advances_ = 0; 
};

} // namespace drone_mapper