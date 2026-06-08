/**
 * @file MissionControlImpl.cpp
 * @brief Implementation of the MissionControlImpl class.
 */

#include "MissionControlImpl.h"
#include <stdexcept>
#include <iostream>

// Assuming yaml-cpp is approved for use
#include <yaml-cpp/yaml.h>

MissionControlImpl::MissionControlImpl(const std::string& missionConfigPath) 
    : m_maxSteps(0), m_minX(0), m_maxX(0), m_minY(0), m_maxY(0), m_minH(0), m_maxH(0) {
    
    parseMissionConfig(missionConfigPath);
}

void MissionControlImpl::parseMissionConfig(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);
        
        YAML::Node mission = config["mission_config"];
        if (!mission) {
            throw std::runtime_error("Missing 'mission_config' root node.");
        }

        m_maxSteps = mission["max_steps"].as<int>();
        
        YAML::Node boundaries = mission["boundaries"];
        
        m_minX = boundaries["x_boundary"]["min_cm"].as<int>();
        m_maxX = boundaries["x_boundary"]["max_cm"].as<int>();
        
        m_minY = boundaries["y_boundary"]["min_cm"].as<int>();
        m_maxY = boundaries["y_boundary"]["max_cm"].as<int>();
        
        m_minH = boundaries["height_boundary"]["min_cm"].as<int>();
        m_maxH = boundaries["height_boundary"]["max_cm"].as<int>();

    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Parsing Error in MissionControl: " + std::string(e.what()));
    }
}

bool MissionControlImpl::isWithinBoundaries(const Position3D& pos) const {
    if (pos.x_cm < m_minX || pos.x_cm > m_maxX) return false;
    if (pos.y_cm < m_minY || pos.y_cm > m_maxY) return false;
    if (pos.height_cm < m_minH || pos.height_cm > m_maxH) return false;
    
    return true;
}

int MissionControlImpl::getMaxSteps() const {
    return m_maxSteps;
}