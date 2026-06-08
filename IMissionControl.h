/**
 * @file IMissionControl.h
 * @brief Declaration of the IMissionControl interface.
 *
 * This interface defines the contract for mission control, relevant both 
 * in the simulation and in the real world.
 */

#pragma once

// A helper data type to represent a position in 3D space.
// You might want to move this to a shared "DataTypes.h" file later.
struct Position3D {
    int x_cm;
    int y_cm;
    int height_cm;
};

/**
 * @class IMissionControl
 * @brief Interface for managing mission constraints and progress.
 */
class IMissionControl {
public:
    virtual ~IMissionControl() = default;

    /**
     * @brief Checks if a given position is within the mission's allowed boundaries.
     * @param pos The 3D position to check.
     * @return true if within boundaries, false otherwise.
     */
    virtual bool isWithinBoundaries(const Position3D& pos) const = 0;

    /**
     * @brief Retrieves the maximum number of steps allowed for this mission.
     * @return int The maximum steps.
     */
    virtual int getMaxSteps() const = 0;
};