/**
 * @file ISimulationRunFactory.h
 * @brief Declaration of the ISimulationRunFactory interface.
 *
 * Interface for the factory responsible for creating concrete ISimulationRun objects.
 */

#pragma once

#include <memory>
#include <string>
#include "ISimulationRun.h"

/**
 * @class ISimulationRunFactory
 * @brief Factory interface for creating simulation runs.
 */
class ISimulationRunFactory {
public:
    virtual ~ISimulationRunFactory() = default;

    /**
     * @brief Creates a new simulation run instance.
     * * @param simConfigPath Path to the simulation configuration file.
     * @param missionConfigPath Path to the mission configuration file.
     * @param droneConfigPath Path to the drone configuration file.
     * @param lidarConfigPath Path to the lidar configuration file.
     * @return std::unique_ptr<ISimulationRun> Pointer to the created simulation run.
     */
    virtual std::unique_ptr<ISimulationRun> createRun(
        const std::string& simConfigPath,
        const std::string& missionConfigPath,
        const std::string& droneConfigPath,
        const std::string& lidarConfigPath) = 0;
};