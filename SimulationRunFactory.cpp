/**
 * @file SimulationRunFactory.cpp
 * @brief Implementation of the SimulationRunFactory class.
 */

#include "SimulationRunFactory.h"
#include "SimulationRunImpl.h" // We include the concrete implementation here

std::unique_ptr<ISimulationRun> SimulationRunFactory::createRun(
    const std::string& simConfigPath,
    const std::string& missionConfigPath,
    const std::string& droneConfigPath,
    const std::string& lidarConfigPath) {
    
    // Instantiates the concrete simulation run and returns it as an interface pointer
    return std::make_unique<SimulationRunImpl>(
        simConfigPath, missionConfigPath, droneConfigPath, lidarConfigPath);
}