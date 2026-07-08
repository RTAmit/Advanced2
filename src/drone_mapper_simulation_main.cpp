#include <drone_mapper/CompositionLoader.h>
#include <drone_mapper/ErrorLog.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationReportWriter.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <filesystem>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
    // Relative paths (whether just a filename or a longer relative path)
    // resolve against the current working directory, same as an absolute
    // path resolves to itself -- both are handled by default
    // std::filesystem::path/ifstream semantics, no extra logic needed.
    const std::filesystem::path composition_file =
        (argc >= 2) ? std::filesystem::path{argv[1]} : std::filesystem::path{"simulation.yaml"};
    const std::filesystem::path output_path =
        (argc >= 3) ? std::filesystem::path{argv[2]} : std::filesystem::current_path();

    drone_mapper::types::SimulationCompositionData composition;
    try {
        composition = drone_mapper::CompositionLoader::load(composition_file);
    } catch (const std::exception& e) {
        drone_mapper::ErrorLog::log("COMPOSITION_LOAD_FAILED", e.what());
        std::cerr << "Failed to load composition file " << composition_file << ": " << e.what() << "\n";
        return 1;
    }

    auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
    drone_mapper::SimulationManager simulation{std::move(run_factory)};

    const drone_mapper::types::SimulationManagerReport report = simulation.run(composition, output_path);
    drone_mapper::SimulationReportWriter::write(composition, report, output_path);

    std::cout << "Ran " << report.runs.size() << " simulation run(s). "
              << "Report written to " << (output_path / "simulation_output.yaml") << "\n";

    return 0;
}
