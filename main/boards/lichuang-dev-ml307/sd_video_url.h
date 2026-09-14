#pragma once

#include <cstddef>
#include <string>

// Shared by the LAN portal, MCP entry point, and SD download worker.
inline bool IsSdVideoUrlValid(const std::string& url) {
    if (url.size() > 1024) {
        return false;
    }
    const size_t prefix = url.compare(0, 7, "http://") == 0    ? 7
                          : url.compare(0, 8, "https://") == 0 ? 8
                                                               : 0;
    if (!prefix || url.size() <= prefix) {
        return false;
    }
    const auto host_end = url.find_first_of("/?#", prefix);
    const auto authority = url.substr(prefix, host_end - prefix);
    if (authority.empty() || authority.find('@') != std::string::npos ||
        url.find('#') != std::string::npos) {
        return false;
    }
    for (unsigned char character : url) {
        if (character <= 0x20 || character == 0x7f || character == '\\') {
            return false;
        }
    }
    return true;
}
