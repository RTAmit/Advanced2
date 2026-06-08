/**
 * @file ILidar.h
 * @brief Interface for the Lidar sensor.
 */

#pragma once

#include "DataTypes.h"

class ILidar {
public:
    virtual ~ILidar() = default;

    /**
     * @brief Performs a 3D scan of the environment.
     * @return LidarScanResult The data collected from the lidar beams.
     */
    virtual LidarScanResult performScan() = 0;
};