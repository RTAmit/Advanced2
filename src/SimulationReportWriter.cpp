#include "drone_mapper/SimulationReportWriter.h"
#include "drone_mapper/Units.h"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <limits>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

namespace {

std::string nowUtcIso8601() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm utc_tm{};
#if defined(_WIN32)
    gmtime_s(&utc_tm, &now_time);
#else
    gmtime_r(&now_time, &utc_tm);
#endif
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc_tm);
    return std::string(buffer);
}

std::string missionStatusToString(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed: return "completed";
    case types::MissionRunStatus::MaxSteps: return "max_steps";
    case types::MissionRunStatus::Error: return "error";
    }
    return "error";
}

std::string resolutionStatusToString(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted: return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored: return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall: return "IGNORED_TOO_SMALL";
    }
    return "IGNORED";
}

YAML::Node buildRunNode(const types::SimulationResult& result) {
    YAML::Node run;
    run["status"] = result.mission_results.empty()
                        ? std::string("error")
                        : missionStatusToString(result.mission_results.front().status);
    run["steps"] = result.mission_results.empty() ? 0u
                                                   : static_cast<unsigned int>(result.mission_results.front().steps);
    run["score"] = result.mission_score;
    run["output_map_file"] = result.output_map_file.string();

    if (result.mission_score < 0.0 && !result.mission_results.empty() &&
        !result.mission_results.front().errors.empty()) {
        const auto& err = result.mission_results.front().errors.front();
        YAML::Node error_ref;
        error_ref["code"] = err.code;
        error_ref["message"] = err.message;
        run["error_ref"] = error_ref;
    }

    return run;
}

} // namespace

void SimulationReportWriter::write(const types::SimulationCompositionData& composition,
                                   const types::SimulationManagerReport& report,
                                   const std::filesystem::path& output_dir) {
    std::size_t total_runs = report.runs.size();
    std::size_t error_runs = 0;
    double sum_score = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();
    std::size_t scored_runs = 0;

    for (const auto& run : report.runs) {
        if (run.mission_score < 0.0) {
            ++error_runs;
            continue;
        }
        ++scored_runs;
        sum_score += run.mission_score;
        min_score = std::min(min_score, run.mission_score);
        max_score = std::max(max_score, run.mission_score);
    }

    YAML::Node score_report;
    score_report["composition_file"] = composition.composition_file.string();
    score_report["generated_at_utc"] = nowUtcIso8601();
    score_report["metric"] = "output_map_accuracy";

    YAML::Node score_range;
    score_range["min"] = 0;
    score_range["max"] = 100;
    score_report["score_range"] = score_range;
    score_report["error_score"] = -1;

    YAML::Node summary;
    summary["total_runs"] = static_cast<unsigned int>(total_runs);
    summary["scored_runs"] = static_cast<unsigned int>(scored_runs);
    summary["error_runs"] = static_cast<unsigned int>(error_runs);
    summary["average_score"] = scored_runs > 0 ? sum_score / static_cast<double>(scored_runs) : 0.0;
    summary["min_score"] = scored_runs > 0 ? min_score : 0.0;
    summary["max_score"] = scored_runs > 0 ? max_score : 0.0;
    score_report["summary"] = summary;

    YAML::Node simulations;
    std::size_t run_index = 0;
    for (const auto& [sim_config, missions] : composition.simulation_mission_groups) {
        YAML::Node sim_node;
        sim_node["map_filename"] = sim_config.map_filename.string();

        YAML::Node mission_nodes;
        for (const auto& mission : missions) {
            YAML::Node mission_node;
            mission_node["max_steps"] = static_cast<unsigned int>(mission.max_steps);

            YAML::Node runs_node;
            bool resolution_recorded = false;
            for (std::size_t d = 0; d < composition.drones.size(); ++d) {
                for (std::size_t l = 0; l < composition.lidars.size(); ++l) {
                    if (run_index >= report.runs.size()) {
                        continue;
                    }
                    const types::SimulationResult& result = report.runs[run_index++];

                    if (!resolution_recorded) {
                        mission_node["resolution_cm"] =
                            result.output_map_config.resolution.force_numerical_value_in(cm);
                        mission_node["resolution_request_status"] =
                            resolutionStatusToString(result.resolution_request_status);
                        resolution_recorded = true;
                    }

                    YAML::Node run_node = buildRunNode(result);
                    run_node["drone_index"] = static_cast<unsigned int>(d);
                    run_node["lidar_index"] = static_cast<unsigned int>(l);
                    runs_node.push_back(run_node);
                }
            }
            mission_node["runs"] = runs_node;
            mission_nodes.push_back(mission_node);
        }
        sim_node["missions"] = mission_nodes;
        simulations.push_back(sim_node);
    }
    score_report["simulations"] = simulations;

    YAML::Node root;
    root["score_report"] = score_report;

    std::filesystem::path output_file = output_dir / "simulation_output.yaml";
    std::ofstream out(output_file, std::ios::trunc);
    out << root;
}

} // namespace drone_mapper
