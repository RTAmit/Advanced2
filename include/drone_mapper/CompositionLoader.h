#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

// Loads a simulation_compositions.yaml file (and every drone/lidar/mission/
// simulation config file it references) into a fully-populated
// types::SimulationCompositionData. All paths referenced anywhere in the
// composition tree are resolved relative to the composition file's own
// directory, matching how the provided sample inputs/ tree is laid out.
class CompositionLoader {
public:
    [[nodiscard]] static types::SimulationCompositionData load(const std::filesystem::path& composition_file);

    [[nodiscard]] static types::DroneConfigData loadDrone(const std::filesystem::path& path);
    [[nodiscard]] static types::LidarConfigData loadLidar(const std::filesystem::path& path);
    [[nodiscard]] static types::MissionConfigData loadMission(const std::filesystem::path& path);
    // map_filename inside the simulation config is resolved relative to
    // base_dir (the top-level composition file's directory), not to path's
    // own directory -- the sample inputs/ tree keeps "map/" as a sibling of
    // "simulation/", not nested under it.
    [[nodiscard]] static types::SimulationConfigData loadSimulation(const std::filesystem::path& path,
                                                                    const std::filesystem::path& base_dir);
};

} // namespace drone_mapper
