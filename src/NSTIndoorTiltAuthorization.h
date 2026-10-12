#pragma once


#include "NSTIndoorSurfaceValidation.h"
#include "NSTSharedSlopeValidation.h"
#include <array>
#include <cstdint>

namespace NSTIndoorTilt {
enum class Reason : std::uint8_t {
    kDisabled, kProbeDisabled, kInvalidActorCount,
    kUnidentifiedStaticOwner, kSharedSlopeRejected, kAuthorized
};
inline const char* ReasonName(Reason r) noexcept {
    switch (r) {
    case Reason::kDisabled: return "DISABLED";
    case Reason::kProbeDisabled: return "PROBE_DISABLED";
    case Reason::kInvalidActorCount: return "INVALID_ACTOR_COUNT";
    case Reason::kUnidentifiedStaticOwner: return "STATIC_OWNER_UNVERIFIED";
    case Reason::kSharedSlopeRejected: return "SHARED_SLOPE_REJECTED";
    case Reason::kAuthorized: return "AUTHORIZED";
    }
    return "UNKNOWN";
}
struct Authorization { Reason reason{Reason::kDisabled}; bool authorized{}; };
inline Authorization Authorize(const NSTSharedSlope::Verdict& slope,
    const std::array<NSTIndoor::Hit,5>& hits, bool enabled, bool probeEnabled,
    std::uint32_t actorCount) noexcept {
    if (!enabled) return {Reason::kDisabled,false};
    if (!probeEnabled) return {Reason::kProbeDisabled,false};
    if (actorCount<1 || actorCount>5) return {Reason::kInvalidActorCount,false};


    for (const auto& hit : hits) {
        if (!hit.valid || hit.collisionLayer!=1 || !hit.ownerResolved ||
            !hit.staticReference || hit.actorReference || hit.furnitureReference)
            return {Reason::kUnidentifiedStaticOwner,false};
    }
    if (!slope.valid()) return {Reason::kSharedSlopeRejected,false};
    return {Reason::kAuthorized,true};
}
}
