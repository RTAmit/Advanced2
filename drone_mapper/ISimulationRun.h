#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

class ISimulationRun {
public:
    virtual ~ISimulationRun() = default;

    [[nodiscard]] virtual types::SimulationResult run() = 0;
};

} // namespace drone_mapper