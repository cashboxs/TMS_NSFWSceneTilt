#pragma once


#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace NSTSharedSlope {
enum class Reason : std::uint8_t {
    kValid, kInvalidSample, kActorGroundOffset, kDiscontinuousStep,
    kSteepNormal, kSlopeLimit
};
inline const char* ReasonName(Reason r) noexcept {
    switch (r) {
    case Reason::kValid: return "VALID";
    case Reason::kInvalidSample: return "INVALID_SAMPLE";
    case Reason::kActorGroundOffset: return "ACTOR_GROUND_OFFSET";
    case Reason::kDiscontinuousStep: return "DISCONTINUOUS_STEP";
    case Reason::kSteepNormal: return "STEEP_NORMAL";
    case Reason::kSlopeLimit: return "SLOPE_LIMIT";
    }
    return "UNKNOWN";
}
struct Verdict {
    Reason reason{Reason::kInvalidSample};
    float slopeForward{};
    float slopeRight{};
    float pitchDeg{};
    float rollDeg{};
    float normalZ{};
    float actorOffset{};
    float maxCenterStep{};
    bool valid() const noexcept { return reason==Reason::kValid; }
};
inline Verdict Evaluate(const std::array<float,5>& z, float actorZ,
    float radius, float maxSlopeDegrees) noexcept {
    constexpr float kRad=0.017453292519943295769f;
    Verdict v{};
    if (!std::isfinite(actorZ) || !std::isfinite(radius) || radius<=0.f ||
        !std::isfinite(maxSlopeDegrees) || maxSlopeDegrees<=0.f) return v;
    for (auto height : z) if (!std::isfinite(height)) return v;
    v.actorOffset=actorZ-z[0];
    if (std::fabs(v.actorOffset)>64.f) { v.reason=Reason::kActorGroundOffset; return v; }
    for (std::size_t i=1;i<z.size();++i) {
        const float step=std::fabs(z[i]-z[0]);
        v.maxCenterStep=std::max(v.maxCenterStep,step);
        if (step>110.f) { v.reason=Reason::kDiscontinuousStep; return v; }
    }
    v.slopeForward=(z[1]-z[2])/(2.f*radius);
    v.slopeRight=(z[4]-z[3])/(2.f*radius);
    v.normalZ=1.f/std::sqrt(1.f+v.slopeForward*v.slopeForward+v.slopeRight*v.slopeRight);
    if (!std::isfinite(v.normalZ) || v.normalZ<0.80f) {
        v.reason=Reason::kSteepNormal; return v;
    }
    v.pitchDeg=std::atan(v.slopeForward)/kRad;
    v.rollDeg=-std::atan(v.slopeRight)/kRad;
    if (!std::isfinite(v.pitchDeg) || !std::isfinite(v.rollDeg) ||
        std::fabs(v.pitchDeg)>maxSlopeDegrees ||
        std::fabs(v.rollDeg)>maxSlopeDegrees) {
        v.reason=Reason::kSlopeLimit; return v;
    }
    v.reason=Reason::kValid;
    return v;
}
}
