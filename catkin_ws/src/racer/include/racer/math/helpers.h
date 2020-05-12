#pragma once

namespace racer::math {

template <typename T>
void hash_combine(size_t &seed, T const &v) {
  seed ^= std::hash<T>()(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

constexpr double sign(double x) { return (0 < x) - (x < 0); }

} // namespace racer::math
