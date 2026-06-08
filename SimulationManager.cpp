/**
 * @file SimulationManager.cpp
 * @brief Implementation of the SimulationManager class using yaml-cpp.
 */

#include "SimulationManager.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h> // External library for YAML parsing

// Constructor: Initializes paths and ensures the output directory exists
SimulationManager::SimulationManager(const fs::path& configPath, const fs::path& outputPath)
    : m_configPath(configPath), m_outputPath(outputPath) {
    
    // Ensure the base output directory exists
    if (!fs::exists(m_outputPath)) {
        fs::create_directories(m_outputPath);
    }
}

void SimulationManager::run() {
    try {
        parseCompositionFile();
        executeSimulations();
        writeOutputResults();
    } catch (const std::exception& e) {
        // All errors MUST be immediately logged to the error log file when they occur, and not deferred.
        std::cerr << "CRITICAL ERROR in SimulationManager: " << e.what() << std::endl;
        
        // In a full implementation, you would also append this to your main error log file here.
        throw; 
    }
}

void SimulationManager::parseCompositionFile() {
    try {
        // Load the YAML file
        YAML::Node config = YAML::LoadFile(m_configPath.string());
        
        // Navigate to the root "simulation_compositions" node
        YAML::Node compositions = config["simulation_compositions"];
        if (!compositions) {
            throw std::runtime_error("Missing 'simulation_compositions' node in config file.");
        }

        // 1. Parse Simulation and Mission configs
        if (compositions["simulations"]) {
            for (const auto& simNode : compositions["simulations"]) {
                std::string simConfigPath = simNode["simulation_config"].as<std::string>();
                
                std::vector<std::string> missions;
                if (simNode["mission_configs"]) {
                    for (const auto& missionNode : simNode["mission_configs"]) {
                        missions.push_back(missionNode.as<std::string>());
                    }
                }
                
                // NOTE: Make sure to add a std::map or a vector of pairs in SimulationManager.h 
                // e.g., std::vector<std::pair<std::string, std::vector<std::string>>> m_simulations;
                // m_simulations.push_back({simConfigPath, missions});
            }
        }

        // 2. Parse Drone configs
        if (compositions["drone_configs"]) {
            for (const auto& droneNode : compositions["drone_configs"]) {
                // NOTE: Add std::vector<std::string> m_droneConfigs; to SimulationManager.h
                // m_droneConfigs.push_back(droneNode.as<std::string>());
            }
        }

        // 3. Parse Lidar configs
        if (compositions["lidar_configs"]) {
            for (const auto& lidarNode : compositions["lidar_configs"]) {
                // NOTE: Add std::vector<std::string> m_lidarConfigs; to SimulationManager.h
                // m_lidarConfigs.push_back(lidarNode.as<std::string>());
            }
        }

    } catch (const YAML::Exception& e) {
        throw std::runtime_error(std::string("YAML Parsing Error: ") + e.what());
    }
}

void SimulationManager::executeSimulations() {
    // Note: This relies on the data structures populated in parseCompositionFile().
    // The Cartesian product is defined as: [mission_configs] * [drone_configs] * [lidar_configs]
    
    /* for (const auto& simPair : m_simulations) {
        const std::string& currentSimConfig = simPair.first;
        
        for (const auto& currentMissionConfig : simPair.second) {
            for (const auto& currentDroneConfig : m_droneConfigs) {
                for (const auto& currentLidarConfig : m_lidarConfigs) {
                    
                    try {
                        // 1. Use ISimulationRunFactory to create a new run
                        // auto run = m_factory->createRun(currentSimConfig, currentMissionConfig, currentDroneConfig, currentLidarConfig);
                        
                        // 2. Execute the run
                        // run->start();
                        
                        // 3. Collect score and store it for the YAML report
                        // double score = run->getScore();
                        // storeResult(currentSimConfig, currentMissionConfig, currentDroneConfig, currentLidarConfig, score, "completed");
                        
                    } catch (const std::exception& e) {
                        // Error Handling Rule: score is -1, log immediately, continue to next scenario.
                        std::cerr << "Simulation run failed: " << e.what() << std::endl;
                        
                        // TODO: Log to your dedicated error log file immediately.
                        // storeResult(currentSimConfig, currentMissionConfig, currentDroneConfig, currentLidarConfig, -1, "error", e.what());
                    }
                }
            }
        }
    }
    */
}

void SimulationManager::writeOutputResults() {
    // Prepare the sub-directory for map files and error logs
    fs::path resultsDir = m_outputPath / "output_results";
    if (!fs::exists(resultsDir)) {
        fs::create_directories(resultsDir);
    }
    
    // Create the structured YAML output file
    fs::path outputYamlPath = m_outputPath / "simulation_output.yaml";
    
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "score_report";
    out << YAML::BeginMap;
    
    // Populate header data
    out << YAML::Key << "composition_file" << YAML::Value << m_configPath.filename().string();
    // TODO: Add timestamp for 'generated_at_utc'
    out << YAML::Key << "metric" << YAML::Value << "output_map_accuracy";
    out << YAML::Key << "score_range";
    out << YAML::BeginMap;
    out << YAML::Key << "min" << YAML::Value << 0;
    out << YAML::Key << "max" << YAML::Value << 100;
    out << YAML::EndMap;
    out << YAML::Key << "error_score" << YAML::Value << -1;
    
    // TODO: Iterate over your stored results to populate the 'summary' and 'simulations' blocks.
    
    out << YAML::EndMap; // End score_report
    out << YAML::EndMap; // End document
    
    // Write out to file
    std::ofstream fout(outputYamlPath);
    if (fout.is_open()) {
        fout << out.c_str();
        fout.close();
    } else {
        throw std::runtime_error("Failed to open output file: " + outputYamlPath.string());
    }
}