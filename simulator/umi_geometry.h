// Deterministic UMi domain correction. No RNG calls and no event scheduling.
#ifndef UMI_FIXED_GEOMETRY_H
#define UMI_FIXED_GEOMETRY_H
#include <cmath>
#include <stdexcept>

namespace umi_geometry
{
constexpr double kBsHeightM = 10.0;
constexpr double kUtHeightM = 1.5;
constexpr double kHorizontalShiftM = 20.0;
constexpr double kOriginalExtentM = 3.0;
constexpr double kMaximumD2DM = 23.194827009486403; // sqrt(23^2 + 3^2)

struct Position { double x; double y; double z; };

inline Position Translate(const Position& position)
{
    return {position.x + kHorizontalShiftM, position.y, position.z};
}

inline double Distance2D(const Position& first, const Position& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

inline void Check(const Position& bs, const Position& ue)
{
    const double distance = Distance2D(bs, ue);
    const double epsilon = 1e-9;
    if (!std::isfinite(bs.x) || !std::isfinite(bs.y) || !std::isfinite(bs.z) ||
        !std::isfinite(ue.x) || !std::isfinite(ue.y) || !std::isfinite(ue.z) ||
        std::abs(bs.z - kBsHeightM) > epsilon ||
        std::abs(ue.z - kUtHeightM) > epsilon ||
        ue.x - bs.x < kHorizontalShiftM - epsilon ||
        ue.x - bs.x > kHorizontalShiftM + kOriginalExtentM + epsilon ||
        ue.y - bs.y < -epsilon || ue.y - bs.y > kOriginalExtentM + epsilon ||
        distance < kHorizontalShiftM - epsilon || distance > kMaximumD2DM + epsilon)
    {
        throw std::runtime_error("UMi fixed geometry outside expected height/distance range");
    }
}
}
#endif
