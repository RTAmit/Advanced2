/**
 * @file DroneControlImpl.h
 * @brief Declaration of the DroneControlImpl class.
 *
 * Manages the drone's active logic: movement decisions, lidar scan decisions, 
 * and updating the mapping algorithm.
 */

#pragma once

#include "IDroneControl.h"

// Forward declarations for the hardware/mock interfaces
class ILidar;
class IGPS;
class IDroneMovement;

/**
 * @class DroneControlImpl
 * @brief Concrete implementation of the drone control logic.
 */
class DroneControlImpl : public IDroneControl {
public:
    /**
     * @brief Constructor for DroneControlImpl.
     * @param mappingAlgo The mapping algorithm to be injected into the base interface.
     * @param lidar Pointer to the Lidar interface.
     * @param gps Pointer to the GPS interface.
     * @param movement Pointer to the drone movement interface.
     */
    DroneControlImpl(std::unique_ptr<IMappingAlgorithm> mappingAlgo,
                     ILidar* lidar,
                     IGPS* gps,
                     IDroneMovement* movement);

    ~DroneControlImpl() override = default;

    /**
     * @brief Performs one cycle of the drone's operation.
     */
    void step() override;

private:
    ILidar* m_lidar;
    IGPS* m_gps;
    IDroneMovement* m_movement;
};