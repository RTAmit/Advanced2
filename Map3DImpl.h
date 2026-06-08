/**
 * @file Map3DImpl.h
 * @brief Declaration of the Map3DImpl class.
 */

#pragma once

#include "IMap3D.h"
#include <vector>

class Map3DImpl : public IMap3D {
public:
    /**
     * @brief Constructor.
     * @param resolution_cm The side length of a single voxel.
     */
    explicit Map3DImpl(int resolution_cm);
    ~Map3DImpl() override = default;

    void loadFromFile(const std::string& filename) override;
    void saveToFile(const std::string& filename) const override;
    
    bool isObstacle(int x_cm, int y_cm, int z_cm) const override;
    void setObstacle(int x_cm, int y_cm, int z_cm, bool isObstacle) override;
    int getResolution() const override;

private:
    int m_resolution_cm;
    // Internal representation of the voxel grid.
    // In a full implementation, you'd store dimensions and a flat vector/array.
    std::vector<uint8_t> m_voxelData; 
    
    // Helper to convert real-world cm to voxel indices
    int cmToIndex(int cm) const;
};