#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/Units.h>

#include <filesystem>
#include <iostream>
#include <memory>

#include <mp-units/systems/si/units.h>

int main(int argc, char** argv) {
    const std::filesystem::path composition_file = (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    const std::filesystem::path output_path = (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

    auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager simulation{std::move(run_factory)};

    drone_mapper::types::SimulationCompositionData composition{
        composition_file,
        {{
            "data_maps/single_voxel_x2_y4_z2.npy",
            10.0 * drone_mapper::cm,
            drone_mapper::Position3D{},
            drone_mapper::Position3D{},
            0.0 * mp_units::non_si::degree,
        }},
        {drone_mapper::types::MissionConfigData{1, 10.0 * drone_mapper::cm, 1}},
        {drone_mapper::types::DroneConfigData{
            30.0 * drone_mapper::cm,
            45.0 * mp_units::non_si::degree,
            50.0 * drone_mapper::cm,
            40.0 * drone_mapper::cm,
        }},
        {drone_mapper::types::LidarConfigData{
            20.0 * drone_mapper::cm,
            120.0 * drone_mapper::cm, // התיקון: חזרנו ל-cm כי ה-struct דורש PhysicalLength
            2.5 * drone_mapper::cm
        }}
    };

    static_cast<void>(simulation.run(composition, output_path));
    
    return 0;
}