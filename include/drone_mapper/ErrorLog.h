#pragma once

#include <string>

namespace drone_mapper {

// Appends one line to error_log.txt (in the current working directory) and
// flushes immediately, per the assignment's "errors MUST be immediately
// logged, not deferred" requirement.
class ErrorLog {
public:
    static void log(const std::string& error_code, const std::string& message);
};

} // namespace drone_mapper
