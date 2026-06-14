/**
 * @file SimulationRunFactory.h
 * @brief Declaration of the concrete SimulationRunFactory class.
 *
 * The actual concrete factory that implements ISimulationRunFactory.
 */

#pragma once

#include "ISimulationRunFactory.h"

/**
 * @class SimulationRunFactory
 * @brief Concrete implementation of the simulation run factory.
 */
class SimulationRunFactory : public ISimulationRunFactory {
public:
    SimulationRunFactory() = default;
    ~SimulationRunFactory() override = default;

    /**
     * @brief Creates a concrete SimulationRunImpl object.
     */
    std::unique_ptr<ISimulationRun> createRun(
        const std::string& simConfigPath,
        const std::string& missionConfigPath,
        const std::string& droneConfigPath,
        const std::string& lidarConfigPath) override;
};