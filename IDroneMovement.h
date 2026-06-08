/**
 * @file IDroneMovement.h
 * @brief Interface for controlling the drone's movement.
 */

#pragma once

#include "DataTypes.h"

class IDroneMovement {
public:
    virtual ~IDroneMovement() = default;

    /**
     * @brief Executes a physical movement command.
     * @param cmd The command to execute (e.g., Advance, Rotate).
     */
    virtual void executeCommand(DroneCommand cmd) = 0;
};