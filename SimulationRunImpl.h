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

// ייבוא הממשקים ומחלקות ה-Mock האמיתיות
#include "IMissionControl.h"
#include "IDroneControl.h"
#include "IMap3D.h"
#include "MockLidar.h"
#include "MockGPS.h"
#include "MockMovement.h"

/**
 * @class SimulationRunImpl
 * @brief Concrete implementation of a single simulation run.
 */
class SimulationRunImpl : public ISimulationRun {
public:
    /**
     * @brief Constructor for SimulationRunImpl.
     * @param simConfigPath Path to the specific simulation configuration YAML.
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
    std::unique_ptr<IMissionControl> m_missionControl;
    std::unique_ptr<IDroneControl> m_droneControl;
    std::unique_ptr<MockLidar> m_mockLidar;       // תוקן ל-MockLidar
    std::unique_ptr<MockGPS> m_mockGPS;           // תוקן ל-MockGPS
    std::unique_ptr<MockMovement> m_mockMovement; // תוקן ל-MockMovement
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