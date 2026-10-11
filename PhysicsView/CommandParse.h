#pragma once

#include <charconv>
#include <cmath>
#include <cstddef>
#include <string_view>
#include <system_error>
#include <vector>

namespace Phantom::CommandParse {

// Strict number parsing shared by the PhysicsView command dispatchers (UI/Vulkan independent,
// header-only so PhysicsTest can exercise it). Every parser requires the whole text to be
// consumed -- "1.5x", "", " 1" and "+1" are rejected -- so a typo can never be read as a
// prefix, and the out-parameter is written only on success.

inline bool parseFloat(std::string_view s, float& out) {
    float v = 0.0f;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || ptr != s.data() + s.size() || !std::isfinite(v)) return false;
    out = v;
    return true;
}

inline bool parseDouble(std::string_view s, double& out) {
    double v = 0.0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || ptr != s.data() + s.size() || !std::isfinite(v)) return false;
    out = v;
    return true;
}

// Overflow ("2147483648") is rejected by from_chars (errc::result_out_of_range).
inline bool parseInt(std::string_view s, int& out) {
    int v = 0;
    const auto [ptr, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (ec != std::errc{} || ptr != s.data() + s.size()) return false;
    out = v;
    return true;
}

// Range-checked variants. The out-parameter is untouched when the value is outside [lo, hi].
inline bool parseFloatIn(std::string_view s, float lo, float hi, float& out) {
    float v = 0.0f;
    if (!parseFloat(s, v) || !(v >= lo && v <= hi)) return false;
    out = v;
    return true;
}

inline bool parseIntIn(std::string_view s, int lo, int hi, int& out) {
    int v = 0;
    if (!parseInt(s, v) || v < lo || v > hi) return false;
    out = v;
    return true;
}

// "Step:N"-style repeat counts: a whole number in [1, maxCount].
inline bool parseCount(std::string_view s, int maxCount, int& out) {
    return parseIntIn(s, 1, maxCount, out);
}

// Splits on every `delim`, keeping empty fields ("a::b" -> {"a","","b"}; "" -> {""}).
inline std::vector<std::string_view> split(std::string_view s, char delim) {
    std::vector<std::string_view> parts;
    size_t start = 0;
    while (true) {
        const size_t pos = s.find(delim, start);
        if (pos == std::string_view::npos) { parts.push_back(s.substr(start)); break; }
        parts.push_back(s.substr(start, pos - start));
        start = pos + 1;
    }
    return parts;
}

// Parses parts as exactly `count` finite floats into out[0..count). All-or-nothing: `out` is
// written only when the part count is exactly `count` and every element parses.
inline bool parseFloats(const std::vector<std::string_view>& parts, size_t count, float* out) {
    if (parts.size() != count) return false;
    constexpr size_t kMax = 16;
    if (count > kMax) return false;
    float tmp[kMax];
    for (size_t i = 0; i < count; ++i)
        if (!parseFloat(parts[i], tmp[i])) return false;
    for (size_t i = 0; i < count; ++i) out[i] = tmp[i];
    return true;
}

// "1"/"true" -> true, "0"/"false" -> false; anything else is rejected.
inline bool parseBool(std::string_view s, bool& out) {
    if (s == "1" || s == "true")  { out = true;  return true; }
    if (s == "0" || s == "false") { out = false; return true; }
    return false;
}

} // namespace Phantom::CommandParse
