/**
 * @file IMap3D.h
 * @brief Declaration of the IMap3D interface for voxel maps.
 */

#pragma once

#include <string>
#include <cstdint>

class IMap3D {
public:
    virtual ~IMap3D() = default;

    /**
     * @brief Loads map data from a .npy file.
     * @param filename Path to the .npy file.
     */
    virtual void loadFromFile(const std::string& filename) = 0;

    /**
     * @brief Saves the current map data to a .npy file.
     * @param filename Path where the .npy file will be saved.
     */
    virtual void saveToFile(const std::string& filename) const = 0;

    /**
     * @brief Checks if a specific voxel represents an obstacle.
     * @param x_cm X coordinate in cm.
     * @param y_cm Y coordinate in cm.
     * @param z_cm Z (height) coordinate in cm.
     * @return true if there is an obstacle, false if free space.
     */
    virtual bool isObstacle(int x_cm, int y_cm, int z_cm) const = 0;

    /**
     * @brief Sets the state of a specific voxel.
     */
    virtual void setObstacle(int x_cm, int y_cm, int z_cm, bool isObstacle) = 0;

    /**
     * @brief Gets the resolution of the map (size of a voxel in cm).
     */
    virtual int getResolution() const = 0;
};