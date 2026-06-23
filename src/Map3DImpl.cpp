#include <drone_mapper/Map3DImpl.h>
#include <cmath>

namespace drone_mapper {

// --- בנאי ---
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> npy_array, const types::MapConfig& config)
    : m_npy_array(std::move(npy_array)), m_config(config) {
}

// --- החזרת הקונפיגורציה ---
types::MapConfig Map3DImpl::getMapConfig() const {
    return m_config;
}

// --- בדיקת גבולות (התוספת החדשה לשלד) ---
bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const auto& b = m_config.boundaries;
    // בודק האם המיקום הפיזי נמצא בתוך תיבת הגבולות המוגדרת
    return pos.x >= b.min_x && pos.x <= b.max_x &&
           pos.y >= b.min_y && pos.y <= b.max_y &&
           pos.z >= b.min_height && pos.z <= b.max_height;
}

// --- פונקציית עזר: תרגום מיקום במרחב הפיזי לאינדקס במערך ---
std::vector<unsigned long> Map3DImpl::positionToIndices(const Position3D& pos) const {
    // חישוב אינדקס על ידי חיסור המינימום וחלוקה ברזולוציה
    unsigned long ix = static_cast<unsigned long>(std::floor((pos.x - m_config.boundaries.min_x) / m_config.resolution));
    unsigned long iy = static_cast<unsigned long>(std::floor((pos.y - m_config.boundaries.min_y) / m_config.resolution));
    unsigned long iz = static_cast<unsigned long>(std::floor((pos.z - m_config.boundaries.min_height) / m_config.resolution));
    
    return {ix, iy, iz};
}

// --- קריאת מצב ווקסל ---
types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    // 1. הגנה: אם המיקום חורג מהגבולות הפיזיים
    if (!isInBounds(pos)) {
        return types::VoxelOccupancy::BeyondMissionBoundaries;
    }

    std::vector<unsigned long> indices = positionToIndices(pos);

    // 2. הגנה נוספת לזיכרון: מוודאים שהאינדקס לא חורג מה-Shape של ה-NpyArray
    if (indices[0] >= m_npy_array->shape[0] || 
        indices[1] >= m_npy_array->shape[1] || 
        indices[2] >= m_npy_array->shape[2]) {
        return types::VoxelOccupancy::BeyondMissionBoundaries;
    }

    // 3. שליפת הערך (uint8_t) מהמערך
    uint8_t val = m_npy_array->getValue<uint8_t>(indices);

    // 4. תרגום הערך המספרי ל-Enum
    switch (val) {
        case 0: return types::VoxelOccupancy::Empty;
        case 1: return types::VoxelOccupancy::Occupied;
        default: return types::VoxelOccupancy::NotMapped;
    }
}

// --- כתיבת מצב ווקסל ---
void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy state) {
    // 1. הגנה: אם המיקום חורג מהגבולות, פשוט לא כותבים כלום (מונע Segmentation Fault)
    if (!isInBounds(pos)) {
        return; 
    }

    std::vector<unsigned long> indices = positionToIndices(pos);

    // 2. הגנה לזיכרון: לא כותבים אם אנחנו מעבר לגודל המערך המוקצה
    if (indices[0] >= m_npy_array->shape[0] || 
        indices[1] >= m_npy_array->shape[1] || 
        indices[2] >= m_npy_array->shape[2]) {
        return;
    }

    // 3. תרגום ה-Enum למספר שיישמר בקובץ הבינארי
    uint8_t val = 255; // NotMapped כברירת מחדל
    switch (state) {
        case types::VoxelOccupancy::Empty: 
            val = 0; 
            break;
        case types::VoxelOccupancy::Occupied: 
            val = 1; 
            break;
        case types::VoxelOccupancy::NotMapped: 
            val = 255; // או -1 בייצוג חתום, תלוי בהגדרות שלך
            break;
        case types::VoxelOccupancy::BeyondMissionBoundaries: 
            return; // אי אפשר באמת לכתוב את זה
    }

    // 4. כתיבה למערך הזיכרון
    m_npy_array->setValue<uint8_t>(indices, val);
}

} // namespace drone_mapper