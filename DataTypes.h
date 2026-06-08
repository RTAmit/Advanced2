/**
 * @file DataTypes.h
 * @brief Common data structures used across the simulation components.
 */

#pragma once

#include <vector>

/**
 * @brief Represents a 3D position in centimeters.
 */
struct Position3D {
    int x_cm;
    int y_cm;
    int height_cm;
};

/**
 * @brief Represents the result of a single Lidar scan.
 * As mentioned in the requirements helper types.
 */
struct LidarScanResult {
    // A simplified representation: points detected by the lidar beams
    std::vector<Position3D> hit_points; 
};

/**
 * @brief Represents movement commands that the algorithm can issue to the drone.
 */
enum class DroneCommand {
    Advance,
    RotateLeft,
    RotateRight,
    Elevate,
    Descend,
    Idle // Do nothing / Mission Complete
};