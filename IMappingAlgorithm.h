/**
 * @file IMappingAlgorithm.h
 * @brief Declaration of the IMappingAlgorithm interface.
 *
 * Used solely by the drone control to update the map and decide on the next move.
 */

#pragma once

#include "DataTypes.h"

class IMappingAlgorithm {
public:
    virtual ~IMappingAlgorithm() = default;

    /**
     * @brief Updates the internal 3D map with new sensor data.
     * @param currentPos The drone's current GPS position.
     * @param scanResult The points detected by the Lidar in the current step.
     */
    virtual void updateMap(const Position3D& currentPos, const LidarScanResult& scanResult) = 0;

    /**
     * @brief Calculates the next movement command based on the current map state.
     * @param currentPos The drone's current GPS position.
     * @return DroneCommand The action the drone should take next.
     */
    virtual DroneCommand calculateNextMove(const Position3D& currentPos) = 0;
};