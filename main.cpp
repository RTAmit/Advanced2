#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/Types.h>

using namespace drone_mapper;

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <config_file.yaml>" << std::endl;
        return 1;
    }

    std::string yaml_file = argv[1];
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_file);
    } catch (const std::exception& e) {
        std::cerr << "Error loading YAML file: " << e.what() << std::endl;
        return 1;
    }

    // קריאת נתוני סימולציה כלליים
    types::SimulationConfigData sim_data;
    sim_data.map_filename = config["simulation"]["map_filename"].as<std::string>();
    
    // שליחת נתוני המיקום הראשוני ברחפן
    auto pos_node = config["simulation"]["initial_drone_position"];
    sim_data.initial_drone_position = Position3D{
        pos_node["x_cm"].as<double>(),
        pos_node["y_cm"].as<double>(),
        pos_node["z_cm"].as<double>()
    };
    sim_data.initial_angle = config["simulation"]["initial_angle"].as<double>();

    // עדכון מטלה 2: פירסום הגדרות המפה (MapConfig) הכוללות את ה-boundaries
    if (config["simulation"]["map_config"]) {
        auto map_cfg_node = config["simulation"]["map_config"];
        auto b_node = map_cfg_node["boundaries"];
        
        sim_data.map_config.boundaries.x_boundary.min_cm = b_node["x_boundary"]["min_cm"].as<int>();
        sim_data.map_config.boundaries.x_boundary.max_cm = b_node["x_boundary"]["max_cm"].as<int>();
        sim_data.map_config.boundaries.y_boundary.min_cm = b_node["y_boundary"]["min_cm"].as<int>();
        sim_data.map_config.boundaries.y_boundary.max_cm = b_node["y_boundary"]["max_cm"].as<int>();
        sim_data.map_config.boundaries.height_boundary.min_cm = b_node["height_boundary"]["min_cm"].as<int>();
        sim_data.map_config.boundaries.height_boundary.max_cm = b_node["height_boundary"]["max_cm"].as<int>();
        
        // קריאת נתוני offset ו-resolution במידה וקיימים ב-YAML של הסגל
        if (map_cfg_node["offset"]) {
            sim_data.map_config.offset.x_cm = map_cfg_node["offset"]["x_cm"].as<double>();
            sim_data.map_config.offset.y_cm = map_cfg_node["offset"]["y_cm"].as<double>();
            sim_data.map_config.offset.z_cm = map_cfg_node["offset"]["z_cm"].as<double>();
        }
        if (map_cfg_node["resolution_cm"]) {
            sim_data.map_config.resolution_cm = map_cfg_node["resolution_cm"].as<double>();
        }
    }

    // קריאת נתוני משימה (MissionConfigData) - ה-boundaries הוסרו מכאן במטלה 2
    types::MissionConfigData miss_data;
    miss_data.max_time_seconds = config["mission"]["max_time_seconds"].as<int>();

    // קריאת נתוני רחפן (DroneConfigData)
    types::DroneConfigData drone_data;
    drone_data.max_speed_cm_s = config["drone"]["max_speed_cm_s"].as<double>();

    // קריאת נתוני חיישן ליידאר (LidarConfigData)
    types::LidarConfigData lidar_data;
    lidar_data.max_distance_cm = config["lidar"]["max_distance_cm"].as<double>();

    // קריאת נתיב פלט
    std::filesystem::path output_path = config["output_path"] ? config["output_path"].as<std::string>() : ".";

    // הרצת ה-SimulationManager עם הקונפיגורציות שנקראו
    try {
        SimulationManager manager;
        manager.run({sim_data}, {miss_data}, {drone_data}, {lidar_data}, output_path);
    } catch (const std::exception& e) {
        std::cerr << "Simulation panicked: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}