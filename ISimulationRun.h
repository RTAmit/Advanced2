/**
 * @file ISimulationRun.h
 * @brief Declaration of the ISimulationRun interface.
 *
 * This component manages a single simulation run. As defined in the assignment
 * requirements, this component is specific to the simulation environment and 
 * is not relevant in the real world.
 */

#pragma once

#include <string>

/**
 * @class ISimulationRun
 * @brief Interface for managing a single simulation execution.
 */
class ISimulationRun {
public:
    /**
     * @brief Virtual destructor.
     * Essential for interfaces in C++ to ensure proper cleanup of derived objects 
     * and prevent memory leaks.
     */
    virtual ~ISimulationRun() = default;

    /**
     * @brief Executes the simulation run.
     * * This method advances the simulation step-by-step until the mission is 
     * completed, the maximum steps are reached, or an error occurs.
     */
    virtual void run() = 0;

    /**
     * @brief Retrieves the final score of the simulation run.
     * * @return double The map comparison score between 0 and 100, or -1 if an error occurred.
     */
    virtual double getScore() const = 0;

    /**
     * @brief Retrieves the final status of the run.
     *
     * @return std::string The status string (e.g., "completed", "max_steps", "error").
     */
    virtual std::string getStatus() const = 0;
    
    /**
     * @brief Retrieves the total number of steps executed during the run.
     *
     * @return int The number of simulation steps performed.
     */
    virtual int getSteps() const = 0;
};