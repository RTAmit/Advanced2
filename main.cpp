/**
 * @file main.cpp
 * @brief Entry point for the Drone Mapper Simulation.
 */

#include <iostream>
#include <string>
#include <filesystem>
#include <exception>

#include "SimulationManager.h" 

namespace fs = std::filesystem;

fs::path resolveSimulationPath(int argc, char* argv[]) {
    if (argc < 2) {
        return fs::current_path();
    }

    std::string argPath = argv[1];
    fs::path p(argPath);

    if (p.is_absolute()) {
        return p;
    }

    return fs::current_path() / p;
}

int main(int argc, char* argv[]) {
    try {
        // מציאת נתיב העבודה
        fs::path workingPath = resolveSimulationPath(argc, argv);

        // אתחול והרצת הסימולציה (ההערות הוסרו וכעת המשתנה בשימוש)
        SimulationManager manager(workingPath.string());
        manager.run();

    } catch (const std::exception& e) {
        std::cerr << "Fatal Error in main: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}