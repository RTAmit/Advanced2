/**
 * @file IGPS.h
 * @brief Interface for the drone's GPS.
 */

#pragma once

#include "DataTypes.h"

class IGPS {
public:
    virtual ~IGPS() = default;

    /**
     * @brief Retrieves the current absolute position of the drone.
     * @return Position3D The current coordinates.
     */
    virtual Position3D getPosition() const = 0;
};