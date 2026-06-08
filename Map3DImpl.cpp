/**
 * @file Map3DImpl.cpp
 * @brief Full implementation of the Map3DImpl class.
 */

#include "Map3DImpl.h"
#include <stdexcept>
#include <fstream>
#include <iostream>
#include <cmath>

Map3DImpl::Map3DImpl(int resolution_cm) 
    : m_resolution_cm(resolution_cm), m_width(100), m_height(100), m_depth(100) {
    // Initialize a default 100x100x100 grid (can be dynamically resized upon loading)
    m_voxelData.resize(m_width * m_height * m_depth, 0);
}

int Map3DImpl::cmToIndex(int cm) const {
    return cm / m_resolution_cm;
}

// 3D to 1D index flattening
size_t Map3DImpl::getFlatIndex(int x_idx, int y_idx, int z_idx) const {
    // Using a safe wrapping or offset if coordinates are negative
    // Assuming the map indices are strictly positive for this internal representation
    int safe_x = std::max(0, std::min(x_idx, m_width - 1));
    int safe_y = std::max(0, std::min(y_idx, m_height - 1));
    int safe_z = std::max(0, std::min(z_idx, m_depth - 1));
    return safe_x + m_width * (safe_y + m_height * safe_z);
}

void Map3DImpl::loadFromFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open map file for reading: " + filename);
    }

    // A minimal .npy parser
    // NPY files start with a 6-byte magic string: "\x93NUMPY"
    char magic[6];
    file.read(magic, 6);
    if (std::string(magic, 6) != "\x93NUMPY") {
        throw std::runtime_error("Invalid .npy file format");
    }

    // Skip version (2 bytes) and header length (2 bytes, little endian)
    uint8_t major, minor;
    file.read(reinterpret_cast<char*>(&major), 1);
    file.read(reinterpret_cast<char*>(&minor), 1);

    uint16_t headerLen;
    file.read(reinterpret_cast<char*>(&headerLen), 2);

    // Skip the dictionary header string
    file.seekg(headerLen, std::ios::cur);

    // Now we are at the raw binary data. Let's read it into our vector.
    // For a robust implementation, you'd parse the dictionary to get exact dimensions.
    // Here we load the remaining bytes into our voxel map.
    m_voxelData.clear();
    char byte;
    while (file.read(&byte, 1)) {
        m_voxelData.push_back(static_cast<uint8_t>(byte));
    }
    
    // Estimate dimensions based on total bytes (assuming cubic map for simplicity)
    int dim = std::round(std::cbrt(m_voxelData.size()));
    m_width = m_height = m_depth = dim;
}

void Map3DImpl::saveToFile(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    // Write a dummy NPY header (Version 1.0)
    file << "\x93NUMPY";
    file << (char)0x01 << (char)0x00; // Version 1.0
    
    // Construct dict string
    std::string dict = "{'descr': '|u1', 'fortran_order': False, 'shape': (" + 
                       std::to_string(m_width) + ", " + 
                       std::to_string(m_height) + ", " + 
                       std::to_string(m_depth) + "), }";
                       
    // Padding to make header length divisible by 64
    int padLen = 64 - ((10 + dict.length() + 1) % 64);
    dict.append(padLen, ' ');
    dict.push_back('\n');

    uint16_t headerLen = dict.length();
    file.write(reinterpret_cast<const char*>(&headerLen), 2);
    file.write(dict.c_str(), dict.length());

    // Write raw data
    file.write(reinterpret_cast<const char*>(m_voxelData.data()), m_voxelData.size());
}

bool Map3DImpl::isObstacle(int x_cm, int y_cm, int z_cm) const {
    int x_idx = cmToIndex(x_cm);
    int y_idx = cmToIndex(y_cm);
    int z_idx = cmToIndex(z_cm);
    
    if (x_idx < 0 || x_idx >= m_width || y_idx < 0 || y_idx >= m_height || z_idx < 0 || z_idx >= m_depth) {
        return true; // Treat out-of-bounds as obstacles for safety
    }
    
    return m_voxelData[getFlatIndex(x_idx, y_idx, z_idx)] > 0;
}

void Map3DImpl::setObstacle(int x_cm, int y_cm, int z_cm, bool isObstacle) {
    int x_idx = cmToIndex(x_cm);
    int y_idx = cmToIndex(y_cm);
    int z_idx = cmToIndex(z_cm);
    
    if (x_idx >= 0 && x_idx < m_width && y_idx >= 0 && y_idx < m_height && z_idx >= 0 && z_idx < m_depth) {
        m_voxelData[getFlatIndex(x_idx, y_idx, z_idx)] = isObstacle ? 1 : 0;
    }
}

int Map3DImpl::getResolution() const {
    return m_resolution_cm;
}