#pragma once

#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

class IMap3D;

class IMappingAlgorithm {
public:
    virtual ~IMappingAlgorithm() = default;
    IMappingAlgorithm(const types::DroneConfigData drone_config, const IMap3D& output_map)
        : _drone_config(drone_config),
          _output_map(output_map) {}
    [[nodiscard]] virtual types::MappingStepCommand nextStep(const types::DroneState& state,
                                                             const types::LidarScanResult* latest_scan) = 0; // latest_scan can be a null pointer!

protected:
    const types::DroneConfigData _drone_config;
    const IMap3D& _output_map;
    
};

} // namespace drone_mapper