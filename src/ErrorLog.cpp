#include "drone_mapper/ErrorLog.h"

#include <fstream>

namespace drone_mapper {

void ErrorLog::log(const std::string& error_code, const std::string& message) {
    std::ofstream log_file("error_log.txt", std::ios_base::app);
    if (log_file.is_open()) {
        log_file << "[ERROR] Code: " << error_code << " | Message: " << message << "\n";
        log_file.flush();
    }
}

} // namespace drone_mapper
