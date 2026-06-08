/**
 * @file SimulationRunImpl.h
 * @brief Declaration of the SimulationRunImpl class.
 *
 * This class implements the ISimulationRun interface and contains the actual 
 * logic for running a single simulation scenario. It orchestrates the drone, 
 * the mission, and the mock sensors.
 */

#pragma once

#include "ISimulationRun.h"
#include <string>
#include <memory>

// Forward declarations of the components used in the simulation
// These will be fully included in the .cpp file to reduce compilation dependencies
class IMissionControl;
class IDroneControl;
class IMap3D;
class ILidar;
class IGPS;
class IDroneMovement;

/**
 * @class SimulationRunImpl
 * @brief Concrete implementation of a single simulation run.
 */
class SimulationRunImpl : public ISimulationRun {
public:
    /**
     * @brief Constructor for SimulationRunImpl.
     * * @param simConfigPath Path to the specific simulation configuration YAML.
     * @param missionConfigPath Path to the mission configuration YAML.
     * @param droneConfigPath Path to the drone configuration YAML.
     * @param lidarConfigPath Path to the lidar configuration YAML.
     */
    SimulationRunImpl(const std::string& simConfigPath, 
                      const std::string& missionConfigPath, 
                      const std::string& droneConfigPath, 
                      const std::string& lidarConfigPath);

    /**
     * @brief Destructor.
     */
    ~SimulationRunImpl() override = default;

    /**
     * @brief Executes the simulation loop.
     * Advances the state until the mission ends, hits max steps, or encounters an error.
     */
    void run() override;

    /**
     * @brief Retrieves the final score of the run.
     * @return double The accuracy score (0-100) or -1 for errors.
     */
    double getScore() const override;

    /**
     * @brief Retrieves the status of the run.
     * @return std::string "completed", "max_steps", or "error".
     */
    std::string getStatus() const override;

    /**
     * @brief Retrieves the total simulation steps taken.
     * @return int The number of steps.
     */
    int getSteps() const override;

private:
    // Simulation state variables
    double m_score;
    std::string m_status;
    int m_steps;
    int m_maxSteps; // Extracted from mission configuration

    // Pointers to the various components required for the simulation.
    // Using std::unique_ptr ensures proper memory management.
    std::unique_ptr<IMissionControl> m_missionControl;
    std::unique_ptr<IDroneControl> m_droneControl;
    std::unique_ptr<ILidar> m_mockLidar;
    std::unique_ptr<IGPS> m_mockGPS;
    std::unique_ptr<IDroneMovement> m_mockMovement;
    std::unique_ptr<IMap3D> m_inputMap;
    std::unique_ptr<IMap3D> m_outputMap;

    /**
     * @brief Initializes all the simulation components.
     * Parses the configuration files and instantiates the controls and mocks.
     */
    void initializeComponents(const std::string& simConfigPath, 
                              const std::string& missionConfigPath, 
                              const std::string& droneConfigPath, 
                              const std::string& lidarConfigPath);

    /**
     * @brief Evaluates the final generated map against the input map.
     * Utilizes the MapsComparison utility to generate the final score.
     */
    void evaluateScore();
};