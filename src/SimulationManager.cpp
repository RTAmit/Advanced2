#include "drone_mapper/SimulationManager.h"
#include "drone_mapper/ErrorLog.h"
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

namespace drone_mapper {

namespace {

// Builds an error-status SimulationResult for a combination that could not
// even be constructed/run (e.g. a missing or corrupt map file), so a single
// bad scenario gets score -1 and logged instead of aborting the whole batch.
types::SimulationResult makeFailedResult(const types::SimulationConfigData& sim,
                                         const types::MissionConfigData& mission,
                                         const std::string& message) {
    ErrorLog::log("SIMULATION_RUN_FAILED", message);

    types::MissionRunResult mission_result;
    mission_result.status = types::MissionRunStatus::Error;
    mission_result.errors.push_back(types::ErrorRef{"SIMULATION_RUN_FAILED", message});

    types::SimulationResult result;
    result.simulation_config = sim;
    result.mission_config = mission;
    result.mission_results.push_back(std::move(mission_result));
    result.mission_score = -1.0;
    return result;
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) throw std::invalid_argument("Factory cannot be null.");
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    const std::filesystem::path results_root = output_path / "output_results";

    std::size_t sim_idx = 0;
    for (const auto& [sim, missions] : composition.simulation_mission_groups) {
        std::size_t mission_idx = 0;
        for (const types::MissionConfigData& mission : missions) {
            std::size_t drone_idx = 0;
            for (const types::DroneConfigData& drone : composition.drones) {
                std::size_t lidar_idx = 0;
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    // Each combination gets its own nested folder under
                    // output_results/ so map files don't overwrite each
                    // other across the batch.
                    const std::filesystem::path run_output_dir =
                        results_root / ("simulation_" + std::to_string(sim_idx)) /
                        ("mission_" + std::to_string(mission_idx)) /
                        ("drone_" + std::to_string(drone_idx)) /
                        ("lidar_" + std::to_string(lidar_idx));

                    // A single bad combination (e.g. an unreadable map file)
                    // must not take down the rest of the batch: catch it,
                    // log it immediately, and record it as an error run
                    // scored -1 so the batch continues.
                    try {
                        std::filesystem::create_directories(run_output_dir);
                        auto run_instance = run_factory_->create(sim, mission, drone, lidar, run_output_dir);
                        runs.push_back(run_instance->run());
                    } catch (const std::exception& e) {
                        runs.push_back(makeFailedResult(sim, mission, e.what()));
                    } catch (...) {
                        runs.push_back(makeFailedResult(sim, mission, "Unknown error"));
                    }
                    ++lidar_idx;
                }
                ++drone_idx;
            }
            ++mission_idx;
        }
        ++sim_idx;
    }
    return types::SimulationManagerReport{"Completed", "All simulations finished", {}, -1, std::move(runs)};
}
} // namespace drone_mapper
