/**
 * @file MapsComparison.h
 * @brief Declaration of the MapsComparison utility class.
 */

#pragma once

#include "IMap3D.h"
#include <string>

class MapsComparison {
public:
    /**
     * @brief Compares two 3D maps and returns a similarity score.
     * @param map1 The ground truth (input) map.
     * @param map2 The generated (output) map.
     * @param resolutionRatio Optional bonus feature string (e.g. "10/20").
     * @return double The score between 0 and 100.
     */
    double compare(const IMap3D& map1, const IMap3D& map2, const std::string& resolutionRatio = "");
};