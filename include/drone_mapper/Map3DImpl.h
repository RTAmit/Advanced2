#pragma once

#include <TinyNPY.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/types/MapTypes.h>
#include <filesystem>
#include <memory>
#include <vector> 

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    Map3DImpl(std::shared_ptr<NpyArray> npy_array, const types::MapConfig& config);

    ~Map3DImpl() override = default;

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    std::shared_ptr<NpyArray> m_npy_array;
    types::MapConfig m_config;

    [[nodiscard]] std::vector<unsigned long> positionToIndices(const Position3D& pos) const;
};

} // namespace drone_mapper