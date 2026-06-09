/**
 * @file IMissionControl.h
 * @brief Declaration of the IMissionControl interface.
 */

#pragma once

#include "DataTypes.h" // תוקן: מייבא את ה-struct מכאן

class IMissionControl {
public:
    virtual ~IMissionControl() = default;

    virtual bool isWithinBoundaries(const Position3D& pos) const = 0;
    virtual int getMaxSteps() const = 0;
};