/**
 * @file MockMovement.cpp
 * @brief Implementation of the MockMovement class.
 */

#include "MockMovement.h"
#include <stdexcept>
#include <yaml-cpp/yaml.h>

MockMovement::MockMovement(const std::string& droneConfigPath, MockGPS* mockGPS)
    : m_mockGPS(mockGPS) {
    
    if (!m_mockGPS) {
        throw std::invalid_argument("MockMovement requires a valid MockGPS instance.");
    }
    parseDroneConfig(droneConfigPath);
}

void MockMovement::parseDroneConfig(const std::string& configPath) {
    try {
        YAML::Node config = YAML::LoadFile(configPath);
        YAML::Node drone = config["drone_config"];
        
        m_dimensions_cm = drone["dimensions_cm"].as<int>();
        m_max_rotate_deg = drone["max_rotate_deg"].as<int>();
        m_max_advance_cm = drone["max_advance_cm"].as<int>();
        m_max_elevate_cm = drone["max_elevate_cm"].as<int>();
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML Parsing Error in MockMovement: " + std::string(e.what()));
    }
}

void MockMovement::executeCommand(DroneCommand cmd) {
    Position3D currentPos = m_mockGPS->getPosition();
    int currentAngle = m_mockGPS->getAngle();

    // Note: A real implementation would apply trigonometry based on the angle to update X and Y.
    switch (cmd) {
        case DroneCommand::Advance:
            // TODO: Calculate new X and Y using cos(angle) and sin(angle) * m_max_advance_cm
            break;
        case DroneCommand::Elevate:
            currentPos.height_cm += m_max_elevate_cm;
            break;
        case DroneCommand::Descend:
            currentPos.height_cm -= m_max_elevate_cm;
            break;
        case DroneCommand::RotateLeft:
            currentAngle -= m_max_rotate_deg;
            break;
        case DroneCommand::RotateRight:
            currentAngle += m_max_rotate_deg;
            break;
        case DroneCommand::Idle:
        default:
            break;
    }

    m_mockGPS->updatePosition(currentPos);
    m_mockGPS->updateAngle(currentAngle);
}