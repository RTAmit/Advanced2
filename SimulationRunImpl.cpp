/**
 * @file SimulationRunImpl.cpp
 * @brief Implementation of the SimulationRunImpl class.
 */

#include "SimulationRunImpl.h"

// כל קובצי המימוש האמיתיים
#include "Map3DImpl.h"
#include "MissionControlImpl.h"
#include "MockGPS.h"
#include "MockMovement.h"
#include "MockLidar.h"
#include "DroneControlImpl.h"
#include "MappingAlgorithmImpl.h"
#include "MapsComparison.h" 

#include <iostream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

SimulationRunImpl::SimulationRunImpl(const std::string& simConfigPath, 
                                     const std::string& missionConfigPath, 
                                     const std::string& droneConfigPath, 
                                     const std::string& lidarConfigPath)
    : m_score(-1.0), m_status("initialized"), m_steps(0), m_maxSteps(0) {
    
    initializeComponents(simConfigPath, missionConfigPath, droneConfigPath, lidarConfigPath);
}

void SimulationRunImpl::initializeComponents(const std::string& simConfigPath, 
                                             const std::string& missionConfigPath, 
                                             const std::string& droneConfigPath, 
                                             const std::string& lidarConfigPath) {
    
    // 1. Load Simulation Config to get map details and initial position
    YAML::Node simConfig = YAML::LoadFile(simConfigPath);
    std::string mapPath = simConfig["simulation_config"]["map_filename"].as<std::string>();
    int res = simConfig["simulation_config"]["map_resolution_cm"].as<int>();
    
    // 2. Initialize Maps
    m_inputMap = std::make_unique<Map3DImpl>(res);
    m_inputMap->loadFromFile(mapPath);
    m_outputMap = std::make_unique<Map3DImpl>(res);

    // 3. Initialize Mission Control
    m_missionControl = std::make_unique<MissionControlImpl>(missionConfigPath);
    m_maxSteps = m_missionControl->getMaxSteps();

    // 4. Initialize Mocks (GPS, Movement, Lidar)
    YAML::Node simData = simConfig["simulation_config"];
    Position3D startPos{
        simData["initial_drone_position"]["x_cm"].as<int>(),
        simData["initial_drone_position"]["y_cm"].as<int>(),
        simData["initial_drone_position"]["height_cm"].as<int>()
    };
    int startAngle = simData["initial_drone_position"]["initial_angle_deg"].as<int>();

    m_mockGPS = std::make_unique<MockGPS>(startPos, startAngle, 10);
    m_mockMovement = std::make_unique<MockMovement>(droneConfigPath, m_mockGPS.get());
    m_mockLidar = std::make_unique<MockLidar>(lidarConfigPath, m_inputMap.get(), m_mockGPS.get());

    // 5. Initialize Mapping Algorithm and inject into Drone Control
    // התיקון הקריטי: הגדרה מפורשת של סוג המצביע שיהיה הממשק
    std::unique_ptr<IMappingAlgorithm> mappingAlgo = std::make_unique<MappingAlgorithmImpl>(res);
    
    // Inject the algorithm into the concrete DroneControlImpl
    m_droneControl = std::make_unique<DroneControlImpl>(
        std::move(mappingAlgo), 
        m_mockLidar.get(), 
        m_mockGPS.get(), 
        m_mockMovement.get()
    );

    std::cout << "All components initialized with config paths." << std::endl;
}

void SimulationRunImpl::run() {
    try {
        m_status = "running";
        
        while (m_steps < m_maxSteps) {
            m_steps++;
        }
        
        if (m_steps >= m_maxSteps && m_status != "completed") {
            m_status = "max_steps"; 
        }
        
        if (m_status == "completed" || m_status == "max_steps") {
            evaluateScore();
        }
        
    } catch (const std::exception& e) {
        m_score = -1.0;
        m_status = "error";
        std::cerr << "Simulation Error during run: " << e.what() << std::endl;
    }
}

void SimulationRunImpl::evaluateScore() {
    // MapsComparison comparator;
    // m_score = comparator.compare(*m_inputMap, *m_outputMap);
}

double SimulationRunImpl::getScore() const {
    return m_score;
}

std::string SimulationRunImpl::getStatus() const {
    return m_status;
}

int SimulationRunImpl::getSteps() const {
    return m_steps;
}