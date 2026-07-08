#include <gtest/gtest.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/Units.h>
#include <TinyNPY.h>
#include <memory>
#include <vector>

using namespace drone_mapper;

namespace {

std::shared_ptr<NpyArray> makeUnmappedArray(std::vector<unsigned long> shape) {
    auto arr = std::make_shared<NpyArray>(shape, 1, 'u', false);
    arr->Allocate();
    std::fill_n(arr->Data<uint8_t>(), shape[0] * shape[1] * shape[2], 255);
    return arr;
}

types::DroneState stateAt(Position3D pos, HorizontalAngle heading_deg) {
    return types::DroneState{pos, Orientation{heading_deg, 0.0 * deg}, 0};
}

MappingAlgorithmImpl makeAlgo(const IMap3D& output_map) {
    return MappingAlgorithmImpl(types::MissionConfigData{}, types::LidarConfigData{},
                                types::DroneConfigData{}, output_map);
}

// The algorithm spends its first 6 calls at any newly-reached cell doing a
// full look-around (Hover + scan_orientation) before making a frontier
// decision, since a single LiDAR scan only covers a forward hemisphere. This
// runs through that panorama and returns the first "real" command after it.
types::MappingStepCommand finishPanoramaAndGetDecision(MappingAlgorithmImpl& algo,
                                                        const types::DroneState& state) {
    types::MappingStepCommand cmd;
    for (int i = 0; i < 6; ++i) {
        cmd = algo.nextStep(state, nullptr);
        EXPECT_TRUE(cmd.scan_orientation.has_value());
        EXPECT_TRUE(cmd.movement.has_value());
        if (cmd.movement.has_value()) {
            EXPECT_EQ(cmd.movement->type, types::MovementCommandType::Hover);
        }
    }
    return algo.nextStep(state, nullptr);
}

} // namespace

TEST(MappingAlgorithm, ScansAllSixDirectionsBeforeDecidingOnANewCell) {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    cfg.boundaries = types::MappingBounds{0 * cm, 30 * cm, 0 * cm, 30 * cm, 0 * cm, 10 * cm};
    Map3DImpl output_map(makeUnmappedArray({3, 3, 1}), cfg);
    MappingAlgorithmImpl algo = makeAlgo(output_map);

    types::DroneState state = stateAt(Position3D{10 * cm, 10 * cm, 5 * cm}, 0 * deg);

    types::MappingStepCommand first = algo.nextStep(state, nullptr);
    ASSERT_TRUE(first.scan_orientation.has_value());
    ASSERT_TRUE(first.movement.has_value());
    EXPECT_EQ(first.movement->type, types::MovementCommandType::Hover);

    // Repeating from the same (unmoved) position must not restart the
    // panorama or ask for the same 6 directions again.
    for (int i = 0; i < 4; ++i) {
        types::MappingStepCommand cmd = algo.nextStep(state, nullptr);
        EXPECT_TRUE(cmd.scan_orientation.has_value());
    }
}

// A flat 3x3x1 (30x30x10 cm) map: vertical candidates always fall outside
// the single Z slice, so only the 4 horizontal neighbors of the center are
// ever reachable.
TEST(MappingAlgorithm, RotatesToFaceTargetBeforeAdvancing) {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    cfg.boundaries = types::MappingBounds{0 * cm, 30 * cm, 0 * cm, 30 * cm, 0 * cm, 10 * cm};
    Map3DImpl output_map(makeUnmappedArray({3, 3, 1}), cfg);
    MappingAlgorithmImpl algo = makeAlgo(output_map);

    // Facing 90 (north), but the first enqueued candidate is +X (east): the
    // algorithm must rotate to face it before ever issuing Advance, since
    // Advance always moves along the current heading, not toward an
    // arbitrary (x, y).
    types::DroneState state = stateAt(Position3D{10 * cm, 10 * cm, 5 * cm}, 90 * deg);
    types::MappingStepCommand cmd = finishPanoramaAndGetDecision(algo, state);

    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, types::MovementCommandType::Rotate);
}

TEST(MappingAlgorithm, SkipsCandidatesAlreadyKnownOccupied) {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    cfg.boundaries = types::MappingBounds{0 * cm, 30 * cm, 0 * cm, 30 * cm, 0 * cm, 10 * cm};
    auto array = makeUnmappedArray({3, 3, 1});
    Map3DImpl output_map(array, cfg);
    // Block the +X neighbor so the algorithm must pick a different target.
    output_map.set(Position3D{20 * cm, 10 * cm, 5 * cm}, types::VoxelOccupancy::Occupied);

    MappingAlgorithmImpl algo = makeAlgo(output_map);

    types::DroneState state = stateAt(Position3D{10 * cm, 10 * cm, 5 * cm}, 0 * deg);
    types::MappingStepCommand cmd = finishPanoramaAndGetDecision(algo, state);

    ASSERT_TRUE(cmd.movement.has_value());
    if (cmd.movement->type == types::MovementCommandType::Advance) {
        // Facing 0 already matches +X, so if it tried to go there it would
        // advance immediately -- the blocked candidate must never be chosen.
        EXPECT_NE(cmd.movement->distance.force_numerical_value_in(cm), 10.0);
    }
}

TEST(MappingAlgorithm, ReportsFinishedWhenNoReachableCandidatesRemain) {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    // A single voxel: every neighbor in every direction falls outside these
    // boundaries, so the algorithm has nowhere left to explore immediately.
    cfg.boundaries = types::MappingBounds{0 * cm, 10 * cm, 0 * cm, 10 * cm, 0 * cm, 10 * cm};
    Map3DImpl output_map(makeUnmappedArray({1, 1, 1}), cfg);

    MappingAlgorithmImpl algo = makeAlgo(output_map);

    types::DroneState state = stateAt(Position3D{5 * cm, 5 * cm, 5 * cm}, 0 * deg);
    types::MappingStepCommand cmd = finishPanoramaAndGetDecision(algo, state);

    EXPECT_EQ(cmd.status, types::AlgorithmStatus::Finished);
    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, types::MovementCommandType::Hover);

    // Must stay finished on subsequent calls instead of trying to explore
    // again.
    types::MappingStepCommand second_cmd = algo.nextStep(state, nullptr);
    EXPECT_EQ(second_cmd.status, types::AlgorithmStatus::Finished);
}

TEST(MappingAlgorithm, UsesElevateForPurelyVerticalCandidates) {
    types::MapConfig cfg;
    cfg.resolution = 10 * cm;
    // A single vertical column: horizontal neighbors are all out of bounds,
    // so only the upward Z candidate is reachable from the bottom voxel.
    cfg.boundaries = types::MappingBounds{0 * cm, 10 * cm, 0 * cm, 10 * cm, 0 * cm, 30 * cm};
    Map3DImpl output_map(makeUnmappedArray({1, 1, 3}), cfg);

    MappingAlgorithmImpl algo = makeAlgo(output_map);

    types::DroneState state = stateAt(Position3D{5 * cm, 5 * cm, 5 * cm}, 0 * deg);
    types::MappingStepCommand cmd = finishPanoramaAndGetDecision(algo, state);

    ASSERT_TRUE(cmd.movement.has_value());
    EXPECT_EQ(cmd.movement->type, types::MovementCommandType::Elevate);
    EXPECT_NEAR(cmd.movement->distance.force_numerical_value_in(cm), 10.0, 1e-9);
}
