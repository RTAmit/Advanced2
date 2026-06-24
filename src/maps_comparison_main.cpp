#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/Map3DImpl.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <string>
#include <vector>
#include <exception>

int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    try {
        std::string origin_map_path = argv[1];
        std::string target_map_path = argv[2];
        std::string config_arg = (argc == 4) ? argv[3] : "";
        std::string config_path = "";

        if (config_arg.find("comparison_config=") == 0) {
            config_path = config_arg.substr(18);
        } else if (!config_arg.empty()) {
            config_path = config_arg;
        }

        drone_mapper::types::MapConfig origin_config{};
        drone_mapper::types::MapConfig target_config{};

        if (!config_path.empty()) {
            YAML::Node yaml_config = YAML::LoadFile(config_path);
            if (yaml_config["comparison_config"]) {
                auto comp_node = yaml_config["comparison_config"];
                
                auto parse_config = [](const YAML::Node& node, drone_mapper::types::MapConfig& cfg) {
                    if (node["map_res_cm"]) {
                        cfg.resolution = node["map_res_cm"].as<int>() * drone_mapper::cm;
                    }
                    
                    if (node["map_offset"]) {
                        cfg.offset.x = node["map_offset"]["x_offset"].as<int>() * drone_mapper::cm;
                        cfg.offset.y = node["map_offset"]["y_offset"].as<int>() * drone_mapper::cm;
                        cfg.offset.z = node["map_offset"]["height_offset"].as<int>() * drone_mapper::cm;
                    }
                    
                    if (node["map_boundaries"]) {
                        auto b = node["map_boundaries"];
                        cfg.boundaries.min_x = b["x_boundary"]["min_cm"].as<int>() * drone_mapper::cm;
                        cfg.boundaries.max_x = b["x_boundary"]["max_cm"].as<int>() * drone_mapper::cm;
                        cfg.boundaries.min_y = b["y_boundary"]["min_cm"].as<int>() * drone_mapper::cm;
                        cfg.boundaries.max_y = b["y_boundary"]["max_cm"].as<int>() * drone_mapper::cm;
                        cfg.boundaries.min_height = b["height_boundary"]["min_cm"].as<int>() * drone_mapper::cm;
                        cfg.boundaries.max_height = b["height_boundary"]["max_cm"].as<int>() * drone_mapper::cm;
                    }
                };

                if (comp_node["original"]) parse_config(comp_node["original"], origin_config);
                if (comp_node["target"]) parse_config(comp_node["target"], target_config);
            }
        } else {
            origin_config.resolution = 10 * drone_mapper::cm;
            target_config.resolution = 10 * drone_mapper::cm;
            
            origin_config.boundaries.min_x = -1000 * drone_mapper::cm;
            origin_config.boundaries.max_x = 1000 * drone_mapper::cm;
            origin_config.boundaries.min_y = -1000 * drone_mapper::cm;
            origin_config.boundaries.max_y = 1000 * drone_mapper::cm;
            origin_config.boundaries.min_height = 0 * drone_mapper::cm;
            origin_config.boundaries.max_height = 1000 * drone_mapper::cm;
            
            target_config.boundaries = origin_config.boundaries;
        }

        auto origin_npy = std::make_shared<NpyArray>();
        origin_npy->LoadNPY(origin_map_path);

        auto target_npy = std::make_shared<NpyArray>();
        target_npy->LoadNPY(target_map_path);

        drone_mapper::Map3DImpl origin_map(origin_npy, origin_config);
        drone_mapper::Map3DImpl target_map(target_npy, target_config);

        std::vector<drone_mapper::IMap3D*> targets = { &target_map };
        
        std::vector<double> scores = drone_mapper::MapsComparison::compare(origin_map, targets);
        
        if (!scores.empty()) {
            std::cout << scores[0] << "\n";
        } else {
            std::cout << "-1\n";
            std::cerr << "Error: No scores returned from map comparison.\n";
            return 1;
        }

    } catch (const std::exception& e) {
        std::cout << "-1\n";
        std::cerr << "Error comparing maps: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cout << "-1\n";
        std::cerr << "Unknown error occurred.\n";
        return 1;
    }

    return 0;
}