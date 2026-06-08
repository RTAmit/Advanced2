/**
 * @file MockGPS.h
 * @brief Declaration of the MockGPS class.
 *
 * Simulates a GPS sensor, used by the simulation and injected into DroneControl.
 */

#pragma once

#include "IGPS.h"

/**
 * @class MockGPS
 * @brief Mock implementation of the GPS sensor.
 */
class MockGPS : public IGPS {
public:
    /**
     * @brief Constructor for MockGPS.
     * @param initialPos The starting position of the drone.
     * @param initialAngle The starting angle (0=east, 90=south, 180=west, 270=north).
     * @param resolution_cm The precision of the GPS.
     */
    MockGPS(const Position3D& initialPos, int initialAngle, int resolution_cm);
    
    ~MockGPS() override = default;

    Position3D getPosition() const override;

    /**
     * @brief Specific to the Mock: allows movement logic to update the current position.
     */
    void updatePosition(const Position3D& newPos);
    
    /**
     * @brief Retrieves the current orientation of the drone.
     */
    int getAngle() const;
    
    /**
     * @brief Specific to the Mock: allows movement logic to update the angle.
     */
    void updateAngle(int newAngle);

private:
    Position3D m_actualPosition;
    int m_angle_deg;
    int m_resolution_cm;
};