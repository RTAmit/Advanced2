/**
 * @file SimulationManager.h
 * @brief Declaration of the SimulationManager class.
 *
 * The SimulationManager is responsible for parsing the main simulation composition
 * file, creating, and managing all individual simulation runs.
 */

#pragma once

#include <filesystem>
#include <vector>
#include <string>

namespace fs = std::filesystem;

/**
 * @class SimulationManager
 * @brief Manages the execution of multiple simulation compositions and orchestrates results.
 */
class SimulationManager {
public:
    /**
     * @brief Constructor for SimulationManager.
     * * @param configPath Path to the simulation composition YAML file.
     * @param outputPath Path to the directory where outputs should be saved.
     */
    SimulationManager(const fs::path& configPath, const fs::path& outputPath);

    /**
     * @brief Default destructor.
     */
    ~SimulationManager() = default;

    /**
     * @brief Executes the full simulation management workflow.
     * * This method orchestrates loading the composition configurations, generating 
     * the combinations of runs, executing them sequentially, handling errors, 
     * and aggregating reports.
     */
    void run();

private:
    fs::path m_configPath;
    fs::path m_outputPath;

    /**
     * @brief Parses the simulation composition YAML configuration file.
     * * Extracts configuration paths for simulations, missions, drones, and lidars.
     */
    void parseCompositionFile();

    /**
     * @brief Iterates over the combinations of configurations and executes individual runs.
     * * Implements the Cartesian product logic for combining mission, drone, and lidar configs.
     */
    void executeSimulations();

    /**
     * @brief Generates the final simulation_output.yaml report and manages logs.
     * * Writes out the aggregated score reports and structures the output results directory.
     */
    void writeOutputResults();
};