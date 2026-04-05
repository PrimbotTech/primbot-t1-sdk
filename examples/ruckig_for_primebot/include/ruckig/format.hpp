#pragma once

#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

namespace ruckig::detail {

inline void append_format(std::ostringstream& stream, std::string_view pattern) {
    if (pattern.find("{}") != std::string_view::npos) {
        throw std::invalid_argument("ruckig format placeholder count mismatch");
    }
    stream << pattern;
}

template<typename T, typename... Rest>
inline void append_format(std::ostringstream& stream, std::string_view pattern, T&& value, Rest&&... rest) {
    const auto pos = pattern.find("{}");
    if (pos == std::string_view::npos) {
        throw std::invalid_argument("ruckig format placeholder count mismatch");
    }

    stream << pattern.substr(0, pos);
    stream << std::forward<T>(value);
    append_format(stream, pattern.substr(pos + 2), std::forward<Rest>(rest)...);
}

template<typename... Args>
inline std::string format(std::string_view pattern, Args&&... args) {
    std::ostringstream stream;
    append_format(stream, pattern, std::forward<Args>(args)...);
    return stream.str();
}

} // namespace ruckig::detail
