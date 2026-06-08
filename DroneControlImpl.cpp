/**
 * @file DroneControlImpl.cpp
 * @brief Implementation of the DroneControlImpl class.
 */

#include "DroneControlImpl.h"

// These will be fully included once their respective headers are created
// #include "IMappingAlgorithm.h"
// #include "ILidar.h"
// #include "IGPS.h"
// #include "IDroneMovement.h"

#include <iostream>

DroneControlImpl::DroneControlImpl(std::unique_ptr<IMappingAlgorithm> mappingAlgo,
                                   ILidar* lidar,
                                   IGPS* gps,
                                   IDroneMovement* movement)
    : IDroneControl(std::move(mappingAlgo)), // Fulfills the strict interface constructor injection requirement
      m_lidar(lidar),
      m_gps(gps),
      m_movement(movement) {
}

void DroneControlImpl::step() {
    // Note: The actual code is commented out until the interfaces are defined, 
    // but the logical flow is documented below.

    /*
     * Logical Flow for a single Drone step:
     * * 1. Sensor Reading:
     * auto currentPosition = m_gps->getPosition();
     * auto scanResult = m_lidar->performScan();
     * * 2. Map Updating:
     * // Pass the raw sensor data to the algorithm to update its internal 3D map
     * m_mappingAlgorithm->updateMap(currentPosition, scanResult);
     * * 3. Decision Making:
     * // Ask the algorithm where to go next based on the newly updated map
     * auto nextMovementCommand = m_mappingAlgorithm->calculateNextMove(currentPosition);
     * * 4. Execution:
     * // Send the command to the movement interface
     * m_movement->executeCommand(nextMovementCommand);
     */
}