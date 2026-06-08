/**
 * @file MissionControlImpl.h
 * @brief Declaration of the MissionControlImpl class.
 *
 * Manages the mission constraints (boundaries, steps) and is relevant 
 * both in simulation and in the real world.
 */

#pragma once

#include "IMissionControl.h"
#include <string>

/**
 * @class MissionControlImpl
 * @brief Concrete implementation of mission control.
 */
class MissionControlImpl : public IMissionControl {
public:
    /**
     * @brief Constructor that loads the mission configuration.
     * @param missionConfigPath Path to the mission configuration YAML file.
     */
    explicit MissionControlImpl(const std::string& missionConfigPath);

    ~MissionControlImpl() override = default;

    bool isWithinBoundaries(const Position3D& pos) const override;
    int getMaxSteps() const override;

private:
    int m_maxSteps;
    
    // Boundary constraints in cm
    int m_minX, m_maxX;
    int m_minY, m_maxY;
    int m_minH, m_maxH;

    /**
     * @brief Parses the mission configuration file to extract constraints.
     * @param path The path to the YAML file.
     */
    void parseMissionConfig(const std::string& path);
};