/**
 * @file main.cpp
 * @brief Entry point for the Drone Mapper Simulation.
 *
 * This file handles the parsing of command-line arguments and initializes the SimulationManager as per the assignment requirements.
 */

#include <iostream>
#include <string>
#include <filesystem>
#include <exception>

#include "SimulationManager.h" 

namespace fs = std::filesystem;

/**
 * @brief Resolves the path to the simulation configuration file based on CLI arguments.
 *
 * @param argc Argument count from main.
 * @param argv Argument vector from main.
 * @return fs::path The resolved absolute or relative path to the config file.
 */
fs::path resolveSimulationConfigPath(int argc, char* argv[]) {
    // If argument is missing - fetch and use "simulation.yaml" in the current working directory.
    if (argc < 2) {
        return fs::current_path() / "simulation.yaml";
    }

    std::string argPath = argv[1];
    fs::path p(argPath);

    // If it's an absolute path (starting with a /)
    if (p.is_absolute()) {
        return p;
    }

    // Relative path (starting without /) or just filename - look under the current working directory.
    return fs::current_path() / p;
}

/**
 * @brief Resolves the output directory path based on CLI arguments.
 *
 * @param argc Argument count from main.
 * @param argv Argument vector from main.
 * @return fs::path The resolved output directory path.
 */
fs::path resolveOutputPath(int argc, char* argv[]) {
    // If output path is not provided, use the current working directory.
    if (argc < 3) {
        return fs::current_path();
    }

    std::string argPath = argv[2];
    fs::path p(argPath);

    if (p.is_absolute()) {
        return p;
    }

    return fs::current_path() / p;
}

int main(int argc, char* argv[]) {
    try {
        // 1. Resolve paths according to assignment rules
        fs::path configPath = resolveSimulationConfigPath(argc, argv);
        fs::path outputPath = resolveOutputPath(argc, argv);

        // 2. Initialize the SimulationManager (Component creation)
        // SimulationManager manager(configPath, outputPath);

        // 3. Start the simulation process
        // manager.run();

    } catch (const std::exception& e) {
        // Standard error handling for fatal setup issues
        std::cerr << "Fatal Error in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}