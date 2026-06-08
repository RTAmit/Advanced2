/**
 * @file MockLidar.cpp
 * @brief Implementation of the MockLidar class.
 */

#include "MockLidar.h"
#include <stdexcept>
#include <yaml-cpp/yaml.h>

MockLidar::MockLidar(const std::string& lidarConfigPath, IMap3D* inputMap, MockGPS* mockGPS)
    : m_inputMap(inputMap), m_mockGPS(mockGPS) {
    
    if (!m_inputMap || !m_mockGPS) {
        throw std::invalid_argument("MockLidar initialized with null dependencies.");
    }
    parseLidarConfig(lidarConfigPath);
}

void MockLidar::parseLidarConfig(const std::string& configPath) {
    try {
        YAML::Node config = YAML::LoadFile(configPath);
        YAML::Node lidar = config["lidar_config"];
        
        m_z_min_cm = lidar["z_min_cm"].as<int>();
        m_z_max_cm = lidar["z_max_cm"].as<int>();
        m_d_cm = lidar["d_cm"].as<double>();
        m_fov_circles = lidar["fov_circles"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Parsing Error in MockLidar: " + std::string(e.what()));
    }
}

LidarScanResult MockLidar::performScan() {
    LidarScanResult result;
    Position3D currentPos = m_mockGPS->getPosition();
    int currentAngle = m_mockGPS->getAngle();

    // TODO: Implement actual Lidar Ray-Casting algorithm here.
    // Use m_z_min_cm, m_z_max_cm, and m_fov_circles to cast virtual rays from 
    // currentPos in the direction of currentAngle against m_inputMap.
    // For every voxel hit, append its coordinates to result.hit_points.

    return result;
}