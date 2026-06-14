#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <iostream>
#include <memory>
#include <vector>
#include <exception>

int main(int argc, char** argv) {
    const std::filesystem::path composition_file =
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    const std::filesystem::path output_path =
        (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

    auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager simulation{std::move(run_factory)};

    drone_mapper::types::SimulationCompositionData composition;
    composition.composition_file = composition_file;

    try {
        YAML::Node config = YAML::LoadFile(composition_file.string());

        // קריאת רשימת הסימולציות
        if (config["simulations"]) {
            for (const auto& sim_node : config["simulations"]) {
                drone_mapper::types::SimulationConfigData sim_data;
                sim_data.hidden_map_path = sim_node["hidden_map_path"].as<std::string>();
                sim_data.resolution = sim_node["resolution_cm"].as<int>() * drone_mapper::cm;
                
                sim_data.initial_drone_position.x = sim_node["initial_position"]["x_cm"].as<int>();
                sim_data.initial_drone_position.y = sim_node["initial_position"]["y_cm"].as<int>();
                sim_data.initial_drone_position.z = sim_node["initial_position"]["z_cm"].as<int>();
                
                sim_data.initial_angle = sim_node["initial_angle_deg"].as<double>() * drone_mapper::horizontal_angle[drone_mapper::deg];
                
                composition.simulations.push_back(sim_data);
            }
        }

        if (config["missions"]) {
            for (const auto& miss_node : config["missions"]) {
                drone_mapper::types::MissionConfigData miss_data;
                miss_data.max_steps = miss_node["max_steps"].as<int>();
                
                auto b_node = miss_node["boundaries"];
                miss_data.boundaries.x_boundary.min_cm = b_node["x_boundary"]["min_cm"].as<int>();
                miss_data.boundaries.x_boundary.max_cm = b_node["x_boundary"]["max_cm"].as<int>();
                miss_data.boundaries.y_boundary.min_cm = b_node["y_boundary"]["min_cm"].as<int>();
                miss_data.boundaries.y_boundary.max_cm = b_node["y_boundary"]["max_cm"].as<int>();
                miss_data.boundaries.height_boundary.min_cm = b_node["height_boundary"]["min_cm"].as<int>();
                miss_data.boundaries.height_boundary.max_cm = b_node["height_boundary"]["max_cm"].as<int>();
                
                composition.missions.push_back(miss_data);
            }
        }

        // קריאת רשימת הרחפנים
        if (config["drones"]) {
            for (const auto& drone_node : config["drones"]) {
                drone_mapper::types::DroneConfigData drone_data;
                drone_data.radius = drone_node["radius_cm"].as<int>() * drone_mapper::cm;
                drone_data.speed = drone_node["speed_cm_per_step"].as<int>() * drone_mapper::cm;
                composition.drones.push_back(drone_data);
            }
        }

        if (config["lidars"]) {
            for (const auto& lidar_node : config["lidars"]) {
                drone_mapper::types::LidarConfigData lidar_data;
                lidar_data.z_min = lidar_node["z_min_cm"].as<int>() * drone_mapper::cm;
                lidar_data.z_max = lidar_node["z_max_cm"].as<int>() * drone_mapper::cm;
                lidar_data.d = lidar_node["d_cm"].as<int>() * drone_mapper::cm;
                lidar_data.fov_circles = lidar_node["fov_circles"].as<int>();
                
                composition.lidars.push_back(lidar_data);
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Failed to parse composition YAML: " << e.what() << "\n";
        return 1;
    }

    const drone_mapper::types::SimulationManagerReport report = simulation.run(composition, output_path);

    std::cout << "Assignment 2 simulator ran "
              << report.runs.size()
              << " run(s) successfully.\n";
              
    return 0;
}