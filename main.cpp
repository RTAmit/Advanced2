#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <filesystem>
#include <memory>
#include <yaml-cpp/yaml.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
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
    if (config["simulation"]["map_filename"]) {
        sim_data.map_filename = config["simulation"]["map_filename"].as<std::string>();
    }
    
    // שליחת נתוני המיקום הראשוני ברחפן - שימוש במכפלת יחידות מידה cm
    if (config["simulation"]["initial_drone_position"]) {
        auto pos_node = config["simulation"]["initial_drone_position"];
        sim_data.initial_drone_position = Position3D{
            pos_node["x_cm"].as<double>() * cm,
            pos_node["y_cm"].as<double>() * cm,
            pos_node["z_cm"].as<double>() * cm
        };
    }
    
    // שליחת הזווית הראשונית - שימוש במכפלת יחידת מידה deg
    if (config["simulation"]["initial_angle"]) {
        sim_data.initial_angle = config["simulation"]["initial_angle"].as<double>() * deg;
    }

    // עדכון מטלה 2: הגדרות המפה הכוללות את ה-offset וה-resolution (ה-boundaries הוסרו מכאן)
    if (config["simulation"]["map_config"]) {
        auto map_cfg_node = config["simulation"]["map_config"];
        
        if (map_cfg_node["offset"]) {
            sim_data.map_offset.x = map_cfg_node["offset"]["x_cm"].as<double>() * cm;
            sim_data.map_offset.y = map_cfg_node["offset"]["y_cm"].as<double>() * cm;
            sim_data.map_offset.z = map_cfg_node["offset"]["z_cm"].as<double>() * cm;
        }
        if (map_cfg_node["resolution_cm"]) {
            sim_data.map_resolution = map_cfg_node["resolution_cm"].as<double>() * cm;
        }
    }

    // קריאת נתוני משימה מעודכנים לפי Struct של מטלה 2
    types::MissionConfigData miss_data;
    if (config["mission"]) {
        if (config["mission"]["max_steps"]) {
            miss_data.max_steps = config["mission"]["max_steps"].as<std::size_t>();
        }
        if (config["mission"]["gps_resolution_cm"]) {
            miss_data.gps_resolution = config["mission"]["gps_resolution_cm"].as<double>() * cm;
        }
        if (config["mission"]["output_mapping_resolution_factor"]) {
            miss_data.output_mapping_resolution_factor = config["mission"]["output_mapping_resolution_factor"].as<double>();
        }
    }

    // קריאת נתוני רחפן מעודכנים (שינוי ל-radius, max_advance וכדומה)
    types::DroneConfigData drone_data;
    if (config["drone"]) {
        if (config["drone"]["radius_cm"]) {
            drone_data.radius = config["drone"]["radius_cm"].as<double>() * cm;
        }
        if (config["drone"]["max_rotate_deg"]) {
            drone_data.max_rotate = config["drone"]["max_rotate_deg"].as<double>() * deg;
        }
        if (config["drone"]["max_advance_cm"]) {
            drone_data.max_advance = config["drone"]["max_advance_cm"].as<double>() * cm;
        }
        if (config["drone"]["max_elevate_cm"]) {
            drone_data.max_elevate = config["drone"]["max_elevate_cm"].as<double>() * cm;
        }
    }

    // קריאת נתוני חיישן ליידאר מעודכנים
    types::LidarConfigData lidar_data;
    if (config["lidar"]) {
        if (config["lidar"]["z_min_cm"]) {
            lidar_data.z_min = config["lidar"]["z_min_cm"].as<double>() * cm;
        }
        if (config["lidar"]["z_max_cm"]) {
            lidar_data.z_max = config["lidar"]["z_max_cm"].as<double>() * cm;
        }
        if (config["lidar"]["d_cm"]) {
            lidar_data.d = config["lidar"]["d_cm"].as<double>() * cm;
        }
        if (config["lidar"]["fov_circles"]) {
            lidar_data.fov_circles = config["lidar"]["fov_circles"].as<std::size_t>();
        }
    }

    // קריאת נתיב פלט
    std::filesystem::path output_path = config["output_path"] ? config["output_path"].as<std::string>() : ".";

    // הרצת ה-SimulationManager באמצעות הזרקת ה-Factory ומבנה ה-Composition החדש
    try {
        types::SimulationCompositionData comp_data;
        comp_data.composition_file = yaml_file;
        comp_data.simulations.push_back(sim_data);
        comp_data.missions.push_back(miss_data);
        comp_data.drones.push_back(drone_data);
        comp_data.lidars.push_back(lidar_data);

        SimulationManager manager(std::make_unique<SimulationRunFactoryImpl>());
        auto report = manager.run(comp_data, output_path);
    } catch (const std::exception& e) {
        std::cerr << "Simulation panicked: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}