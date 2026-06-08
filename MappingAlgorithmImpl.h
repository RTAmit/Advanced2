#pragma once
#include "IMappingAlgorithm.h"
#include "Map3DImpl.h"
#include <memory>

class MappingAlgorithmImpl : public IMappingAlgorithm {
public:
    explicit MappingAlgorithmImpl(int resolution_cm);
    ~MappingAlgorithmImpl() override = default;

    void updateMap(const Position3D& currentPos, const LidarScanResult& scanResult) override;
    DroneCommand calculateNextMove(const Position3D& currentPos) override;

private:
    std::unique_ptr<Map3DImpl> m_internalMap;
    int m_stepsInCurrentDirection;
};