/**
 * @file MapsComparison.cpp
 * @brief Implementation of the MapsComparison utility.
 */

#include "MapsComparison.h"
#include <algorithm>

double MapsComparison::compare(const IMap3D& map1, const IMap3D& map2, const std::string& resolutionRatio) {
    // Telling the compiler we intentionally aren't using these yet
    (void)map1;
    (void)map2;
    (void)resolutionRatio;
    
    double score = 100.0;
    return std::clamp(score, 0.0, 100.0);
}