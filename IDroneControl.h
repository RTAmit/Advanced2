/**
 * @file IDroneControl.h
 * @brief Declaration of the IDroneControl interface.
 *
 * This interface defines the contract for drone control. As per assignment 
 * requirements, the injection of the mapping algorithm occurs in this 
 * interface's constructor.
 */

#pragma once

#include <memory>

// Forward declaration of the mapping algorithm interface
#include "IMappingAlgorithm.h"
/**
 * @class IDroneControl
 * @brief Interface for drone control.
 */
class IDroneControl {
protected:
    // The mapping algorithm injected into the drone control
    std::unique_ptr<IMappingAlgorithm> m_mappingAlgorithm;

    /**
     * @brief Protected constructor to inject the mapping algorithm.
     * @param mappingAlgo Unique pointer to the mapping algorithm implementation.
     */
    explicit IDroneControl(std::unique_ptr<IMappingAlgorithm> mappingAlgo)
        : m_mappingAlgorithm(std::move(mappingAlgo)) {}

public:
    virtual ~IDroneControl() = default;

    /**
     * @brief Executes a single decision and action step for the drone.
     * This includes reading sensors, updating the map, and moving.
     */
    virtual void step() = 0;
};