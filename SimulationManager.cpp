/**
 * @file SimulationManager.cpp
 * @brief Implementation of the SimulationManager class using yaml-cpp.
 */

#include "SimulationManager.h"
#include "SimulationRunFactory.h" // תוקן: חובה להוסיף את קובץ ה-Factory
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

SimulationManager::SimulationManager(const fs::path& configPath, const fs::path& outputPath)
    : m_configPath(configPath), m_outputPath(outputPath) {
    
    if (!fs::exists(m_outputPath)) {
        fs::create_directories(m_outputPath);
    }
    
    // תוקן: אתחול ה-Factory כדי שלא תקבל שגיאת גישה לזיכרון
    m_factory = std::make_unique<SimulationRunFactory>();
}

void SimulationManager::run() {
    try {
        parseCompositionFile();
        executeSimulations();
        writeOutputResults();
    } catch (const std::exception& e) {
        std::cerr << "CRITICAL ERROR in SimulationManager: " << e.what() << std::endl;
        throw; 
    }
}

void SimulationManager::parseCompositionFile() {
    // תוכן הפונקציה שלך כפי שהיה...
}

void SimulationManager::executeSimulations() {
    // תוכן הפונקציה שלך כפי שהיה...
}

void SimulationManager::writeOutputResults() {
    // תוכן הפונקציה שלך כפי שהיה...
}