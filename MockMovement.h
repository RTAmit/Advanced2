/**
 * @file MockMovement.h
 * @brief Declaration of the MockMovement class.
 *
 * Simulates drone movement. Modifies the internal simulation state 
 * (held by MockGPS) when movement commands are executed.
 */

#pragma once

#include "IDroneMovement.h"
#include "MockGPS.h"
#include <string>

class MockMovement : public IDroneMovement {
public:
    MockMovement(const std::string& droneConfigPath, MockGPS* mockGPS);
    ~MockMovement() override = default;

    void executeCommand(DroneCommand cmd) override;

private:
    MockGPS* m_mockGPS;
    
    // Constraints loaded from drone configuration
    int m_dimensions_cm;
    int m_max_rotate_deg;
    int m_max_advance_cm;
    int m_max_elevate_cm;

    void parseDroneConfig(const std::string& configPath);
};