/**
 * @file MockLidar.h
 * @brief Declaration of the MockLidar class.
 *
 * Simulates a Lidar sensor by ray-casting against the input voxel map.
 */

#pragma once

#include "ILidar.h"
#include "MockGPS.h"
#include <string>

// Forward declaration of the 3D map interface
class IMap3D;

class MockLidar : public ILidar {
public:
    MockLidar(const std::string& lidarConfigPath, IMap3D* inputMap, MockGPS* mockGPS);
    ~MockLidar() override = default;

    LidarScanResult performScan() override;

private:
    IMap3D* m_inputMap;
    MockGPS* m_mockGPS;

    // Config parameters
    int m_z_min_cm;
    int m_z_max_cm;
    double m_d_cm;
    int m_fov_circles;

    void parseLidarConfig(const std::string& configPath);
};