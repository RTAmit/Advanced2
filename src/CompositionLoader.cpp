#include "drone_mapper/CompositionLoader.h"
#include "drone_mapper/Units.h"

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <tuple>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

namespace {

double asDoubleOr(const YAML::Node& node, double fallback) {
    return node ? node.as<double>() : fallback;
}

types::MappingBounds parseBoundaries(const YAML::Node& boundaries) {
    types::MappingBounds bounds;
    if (!boundaries) {
        return bounds;
    }
    if (const auto x = boundaries["x_boundary"]) {
        bounds.min_x = asDoubleOr(x["min_cm"], 0.0) * cm;
        bounds.max_x = asDoubleOr(x["max_cm"], 0.0) * cm;
    }
    if (const auto y = boundaries["y_boundary"]) {
        bounds.min_y = asDoubleOr(y["min_cm"], 0.0) * cm;
        bounds.max_y = asDoubleOr(y["max_cm"], 0.0) * cm;
    }
    if (const auto h = boundaries["height_boundary"]) {
        bounds.min_height = asDoubleOr(h["min_cm"], 0.0) * cm;
        bounds.max_height = asDoubleOr(h["max_cm"], 0.0) * cm;
    }
    return bounds;
}

} // namespace

types::DroneConfigData CompositionLoader::loadDrone(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    YAML::Node node = root["drone_config"];
    if (!node) {
        throw std::runtime_error("Missing drone_config in " + path.string());
    }

    types::DroneConfigData drone;
    // The YAML gives the sphere's diameter; DroneConfigData stores a radius.
    drone.radius = (asDoubleOr(node["dimensions_cm"], 0.0) / 2.0) * cm;
    drone.max_rotate = asDoubleOr(node["max_rotate_deg"], 0.0) * deg;
    drone.max_advance = asDoubleOr(node["max_advance_cm"], 0.0) * cm;
    drone.max_elevate = asDoubleOr(node["max_elevate_cm"], 0.0) * cm;
    return drone;
}

types::LidarConfigData CompositionLoader::loadLidar(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    YAML::Node node = root["lidar_config"];
    if (!node) {
        throw std::runtime_error("Missing lidar_config in " + path.string());
    }

    types::LidarConfigData lidar;
    lidar.z_min = asDoubleOr(node["z_min_cm"], 0.0) * cm;
    lidar.z_max = asDoubleOr(node["z_max_cm"], 0.0) * cm;
    lidar.d = asDoubleOr(node["d_cm"], 0.0) * cm;
    lidar.fov_circles = node["fov_circles"] ? node["fov_circles"].as<std::size_t>() : 0;
    return lidar;
}

types::MissionConfigData CompositionLoader::loadMission(const std::filesystem::path& path) {
    YAML::Node root = YAML::LoadFile(path.string());
    YAML::Node node = root["mission_config"];
    if (!node) {
        throw std::runtime_error("Missing mission_config in " + path.string());
    }

    types::MissionConfigData mission;
    mission.max_steps = node["max_steps"] ? node["max_steps"].as<std::size_t>() : 0;
    mission.mission_bounds = parseBoundaries(node["boundaries"]);
    mission.gps_resolution = asDoubleOr(node["gps_resolution_cm"], 0.0) * cm;
    // Optional: defaults to 0 here, which CompositionLoader's consumers (see
    // SimulationRunFactoryImpl) treat as "not specified" -> factor 1.
    mission.output_mapping_resolution_factor = asDoubleOr(node["output_mapping_resolution_factor"], 0.0);
    return mission;
}

types::SimulationConfigData CompositionLoader::loadSimulation(const std::filesystem::path& path,
                                                               const std::filesystem::path& base_dir) {
    YAML::Node root = YAML::LoadFile(path.string());
    YAML::Node node = root["simulation_config"];
    if (!node) {
        throw std::runtime_error("Missing simulation_config in " + path.string());
    }

    types::SimulationConfigData simulation;

    const std::string map_filename = node["map_filename"] ? node["map_filename"].as<std::string>() : "";
    simulation.map_filename = base_dir / map_filename;
    simulation.map_resolution = asDoubleOr(node["map_resolution_cm"], 0.0) * cm;

    if (const auto pos = node["initial_drone_position"]) {
        simulation.initial_drone_position = Position3D{
            asDoubleOr(pos["x_cm"], 0.0) * cm,
            asDoubleOr(pos["y_cm"], 0.0) * cm,
            asDoubleOr(pos["height_cm"], 0.0) * cm,
        };
    }
    simulation.initial_angle = asDoubleOr(node["initial_angle_deg"], 0.0) * deg;

    if (const auto offset = node["map_axes_offset"]) {
        simulation.map_offset = Position3D{
            asDoubleOr(offset["x_offset"], 0.0) * cm,
            asDoubleOr(offset["y_offset"], 0.0) * cm,
            asDoubleOr(offset["height_offset"], 0.0) * cm,
        };
    }

    return simulation;
}

types::SimulationCompositionData CompositionLoader::load(const std::filesystem::path& composition_file) {
    YAML::Node root = YAML::LoadFile(composition_file.string());
    YAML::Node node = root["simulation_compositions"];
    if (!node) {
        throw std::runtime_error("Missing simulation_compositions in " + composition_file.string());
    }

    const std::filesystem::path base_dir = composition_file.parent_path();

    types::SimulationCompositionData composition;
    composition.composition_file = composition_file;

    for (const auto& sim_node : node["simulations"]) {
        const auto sim_path = base_dir / sim_node["simulation_config"].as<std::string>();
        types::SimulationConfigData simulation = loadSimulation(sim_path, base_dir);

        std::vector<types::MissionConfigData> missions;
        for (const auto& mission_path_node : sim_node["mission_configs"]) {
            const auto mission_path = base_dir / mission_path_node.as<std::string>();
            missions.push_back(loadMission(mission_path));
        }

        composition.simulation_mission_groups.push_back(std::tuple{std::move(simulation), std::move(missions)});
    }

    for (const auto& drone_path_node : node["drone_configs"]) {
        composition.drones.push_back(loadDrone(base_dir / drone_path_node.as<std::string>()));
    }

    for (const auto& lidar_path_node : node["lidar_configs"]) {
        composition.lidars.push_back(loadLidar(base_dir / lidar_path_node.as<std::string>()));
    }

    return composition;
}

} // namespace drone_mapper
