#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

class ISimulation {
public:
    virtual ~ISimulation() = default;

    [[nodiscard]] virtual types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                                             const std::filesystem::path& output_path) = 0;
};

} // namespace drone_mapper