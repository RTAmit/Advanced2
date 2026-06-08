/**
 * @file maps_comparison_main.cpp
 * @brief Entry point for the standalone maps_comparison executable.
 */

#include "MapsComparison.h"
#include "Map3DImpl.h"
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    // Check arguments
    if (argc < 3) {
        std::cerr << "Usage: ./maps_comparison <map1> <map2> [resolution_ratio=<res1>/<res2>]" << std::endl;
        std::cout << -1 << std::endl; // Error rule
        return 1;
    }

    std::string map1Path = argv[1];
    std::string map2Path = argv[2];
    std::string resolutionRatio = (argc >= 4) ? argv[3] : "";

    try {
        // Instantiate maps (Resolution is dummy here, would need to be parsed or assumed)
        Map3DImpl map1(10); 
        Map3DImpl map2(10);

        map1.loadFromFile(map1Path);
        map2.loadFromFile(map2Path);

        MapsComparison comparator;
        double score = comparator.compare(map1, map2, resolutionRatio);

        // Strict requirement: print ONLY the floating point score between 0 and 100
        std::cout << score << std::endl;

    } catch (const std::exception& e) {
        // Strict requirement: -1 to stdout, description to stderr
        std::cerr << "Error comparing maps: " << e.what() << std::endl;
        std::cout << -1 << std::endl;
        return 1;
    }

    return 0;
}