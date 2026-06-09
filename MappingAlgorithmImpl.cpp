#include "MappingAlgorithmImpl.h"
#include <iostream>

MappingAlgorithmImpl::MappingAlgorithmImpl(int resolution_cm) 
    : m_stepsInCurrentDirection(0) {
    m_internalMap = std::make_unique<Map3DImpl>(resolution_cm);
}

void MappingAlgorithmImpl::updateMap(const Position3D& currentPos, const LidarScanResult& scanResult) {
    // Mark the current position as free space
    m_internalMap->setObstacle(currentPos.x_cm, currentPos.y_cm, currentPos.height_cm, false);

    // Register Lidar hits as obstacles in our internal map
    for (const auto& hit : scanResult.hit_points) {
        m_internalMap->setObstacle(hit.x_cm, hit.y_cm, hit.height_cm, true);
    }
}

DroneCommand MappingAlgorithmImpl::calculateNextMove(const Position3D& currentPos) {
    // Very basic exploration logic: 
    // Go straight for a few steps, then rotate. 
    // If there's an obstacle immediately in front (dummy check here), rotate early.
    (void)currentPos;
    m_stepsInCurrentDirection++;

    if (m_stepsInCurrentDirection > 10) {
        m_stepsInCurrentDirection = 0;
        return DroneCommand::RotateRight; // Turn 90 degrees (or whatever drone config says)
    }

    return DroneCommand::Advance; // Move forward
}