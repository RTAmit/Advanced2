#include "drone_mapper/SimulationManager.h"
#include <stdexcept>
#include <utility>

namespace drone_mapper {
SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory)
    : run_factory_(std::move(run_factory)) {
    if (!run_factory_) throw std::invalid_argument("Factory cannot be null.");
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;
    for (const auto& sim : composition.simulations) {
        for (const auto& mission : composition.missions) {
            for (const auto& drone : composition.drones) {
                for (const auto& lidar : composition.lidars) {
                    auto run_instance = run_factory_->create(sim, mission, drone, lidar, output_path);
                    runs.push_back(run_instance->run());
                }
            }
        }
    }
    return types::SimulationManagerReport{"Completed", "All simulations finished", {}, -1, std::move(runs)};
}
} // namespace drone_mapper