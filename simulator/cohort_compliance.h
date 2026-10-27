#pragma once

#include <algorithm>
#include <cstdint>

namespace deadline_cohort
{

// The legacy partial_compliant CSV field checks the nominal goodput SLA.
// loadAdjustedCompliant and the native observation streak retain strict
// comparisons using min(nominal goodput SLA, offered goodput). Terminal H
// shares that target convention but retains its separate floating tolerances.
// A warm-up or empty cohort exports -1 for both flags: it is not evidence of
// compliance. Empty cohorts nevertheless break the observation streak.
struct Compliance
{
    std::uint32_t nextStreak;
    int nominalCompliant;
    int loadAdjustedCompliant;
};

inline Compliance
Evaluate(std::uint64_t offered,
         std::uint64_t timely,
         double timelyGoodputBps,
         double reliabilityTarget,
         double nominalGoodputTargetBps,
         double offeredGoodputBps,
         std::uint32_t previousStreak,
         bool warmup)
{
    if (offered == 0)
    {
        return {0, -1, -1};
    }
    const double reliability = static_cast<double>(timely) / offered;
    const bool nominalCompliant = reliability >= reliabilityTarget &&
                                  timelyGoodputBps >= nominalGoodputTargetBps;
    const bool loadAdjustedCompliant =
        reliability >= reliabilityTarget &&
        timelyGoodputBps >= std::min(nominalGoodputTargetBps, offeredGoodputBps);
    return {loadAdjustedCompliant ? 0 : previousStreak + 1,
            warmup ? -1 : static_cast<int>(nominalCompliant),
            warmup ? -1 : static_cast<int>(loadAdjustedCompliant)};
}

} // namespace deadline_cohort
