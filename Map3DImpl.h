#pragma once

#include <TinyNPY.h> // ספרייה שהסגל מספק לקריאת NPY
#include <drone_mapper/IMutableMap3D.h>
#include <filesystem>
#include <memory>

namespace drone_mapper {

class Map3DImpl final : public IMutableMap3D {
public:
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    [[nodiscard]] types::MapConfig getMapConfig() const override;

    // Mutable map methods
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    void save(const std::filesystem::path& output_path) const override;

private:
    std::shared_ptr<NpyArray> map_;
    types::MapConfig config_;

    [[nodiscard]] bool getIndices(const Position3D& pos, int& x_idx, int& y_idx, int& z_idx) const;
};

} // namespace drone_mapper