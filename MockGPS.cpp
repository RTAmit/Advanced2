/**
 * @file MockGPS.cpp
 * @brief Implementation of the MockGPS class.
 */

#include "MockGPS.h"

MockGPS::MockGPS(const Position3D& initialPos, int initialAngle, int resolution_cm)
    : m_actualPosition(initialPos), m_angle_deg(initialAngle), m_resolution_cm(resolution_cm) {}

Position3D MockGPS::getPosition() const {
    // In a more complex simulation, we would apply noise based on m_resolution_cm here.
    // For now, it simply returns the current exact position.
    return m_actualPosition;
}

void MockGPS::updatePosition(const Position3D& newPos) {
    m_actualPosition = newPos;
}

int MockGPS::getAngle() const {
    return m_angle_deg;
}

void MockGPS::updateAngle(int newAngle) {
    // Keep angle strictly within 0-359 degrees
    m_angle_deg = (newAngle % 360 + 360) % 360; 
}