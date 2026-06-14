/**
 * @file SimulationManager.h
 * @brief Declaration of the SimulationManager class.
 */

#pragma once

#include <filesystem>
#include <vector>
#include <string>
#include <memory>
#include "ISimulationRunFactory.h"

namespace fs = std::filesystem;

class SimulationManager {
public:
    SimulationManager(const fs::path& configPath, const fs::path& outputPath);
    ~SimulationManager() = default;

    void run();

private:
    fs::path m_configPath;
    fs::path m_outputPath;
    
    // התוספת החסרה שגרמה לקריסה:
    std::unique_ptr<ISimulationRunFactory> m_factory;

    void parseCompositionFile();
    void executeSimulations();
    void writeOutputResults();
};