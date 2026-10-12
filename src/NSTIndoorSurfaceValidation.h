#pragma once


#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace NSTIndoor {
struct Vec3 { float x{}, y{}, z{}; };
struct Hit {
    bool valid{};
    float height{};
    Vec3 normal{};
    std::uint32_t collisionLayer{};
    bool actorReference{};
    bool furnitureReference{};
    bool ownerResolved{};
    bool staticReference{};
};
struct Limits {
    float maxPlaneResidual{8.0f};
    float maxCenterStep{80.0f};
    float maxActorOffset{48.0f};
    float minUpNormalZ{0.65f};
    float maxNormalDeviationDeg{25.0f};
};
enum class Result : std::uint8_t {
    kInsufficientHits,
    kInvalidInput,
    kActorOrFurniture,
    kUntrustedLayer,
    kSteepHitNormal,
    kNormalMismatch,
    kActorGroundOffset,
    kDiscontinuousStep,
    kNonPlanar,
    kOverSlopeLimit,
    kGeometryCandidateOnly
};
inline const char* ResultName(Result r) noexcept {
    switch (r) {
    case Result::kInsufficientHits: return "INSUFFICIENT_HITS";
    case Result::kInvalidInput: return "INVALID_INPUT";
    case Result::kActorOrFurniture: return "ACTOR_OR_FURNITURE";
    case Result::kUntrustedLayer: return "UNTRUSTED_LAYER";
    case Result::kSteepHitNormal: return "STEEP_HIT_NORMAL";
    case Result::kNormalMismatch: return "NORMAL_MISMATCH";
    case Result::kActorGroundOffset: return "ACTOR_GROUND_OFFSET";
    case Result::kDiscontinuousStep: return "DISCONTINUOUS_STEP";
    case Result::kNonPlanar: return "NONPLANAR";
    case Result::kOverSlopeLimit: return "OVER_SLOPE_LIMIT";
    case Result::kGeometryCandidateOnly: return "GEOMETRY_CANDIDATE_UNVERIFIED";
    }
    return "UNKNOWN";
}
inline bool IsConservativeSurfaceLayer(std::uint32_t l) noexcept {


    return l==1 || l==13 || l==17;
}
inline bool Finite(float f) noexcept { return std::isfinite(f); }
inline bool Finite(Vec3 v) noexcept { return Finite(v.x) && Finite(v.y) && Finite(v.z); }
inline float Dot(Vec3 a, Vec3 b) noexcept { return a.x*b.x+a.y*b.y+a.z*b.z; }
inline Vec3 Unit(Vec3 v) noexcept {
    const auto len=std::sqrt(Dot(v,v));
    return Finite(len) && len>0.00001f ? Vec3{v.x/len,v.y/len,v.z/len}:Vec3{};
}
struct Verdict {
    Result result{Result::kInsufficientHits};
    int hitCount{};
    float pitchDeg{};
    float rollDeg{};
    float maxStep{};
    float centerActorDelta{};
    float planeZ{};
    float centerResidual{};
    float maxResidual{};
    float rmsResidual{};
    float maxNormalDeviationDeg{};
    Vec3 planeNormal{};
    bool candidate() const noexcept { return result==Result::kGeometryCandidateOnly; }
};
inline Verdict Evaluate(const std::array<Hit,5>& hits, float radius,
    float actorZ, Vec3 right, Vec3 forward, float maxSlopeDeg,
    const Limits& lim={}) noexcept {
    constexpr float rad=0.017453292519943295769f;
    Verdict out{};
    for (const auto& hit : hits) if (hit.valid) ++out.hitCount;
    if (out.hitCount!=5) return out;
    if (!Finite(radius) || radius<=0 || !Finite(actorZ) || !Finite(maxSlopeDeg) ||
        !Finite(right) || !Finite(forward) || !Finite(lim.maxPlaneResidual) ||
        !Finite(lim.maxCenterStep) || !Finite(lim.maxActorOffset) ||
        !Finite(lim.minUpNormalZ) || !Finite(lim.maxNormalDeviationDeg)) {
        out.result=Result::kInvalidInput; return out;
    }
    for (const auto& hit:hits) {
        if (!Finite(hit.height) || !Finite(hit.normal)) {
            out.result=Result::kInvalidInput; return out;
        }
    }

    const float sf=(hits[1].height-hits[2].height)/(2.f*radius);
    const float sr=(hits[4].height-hits[3].height)/(2.f*radius);
    out.pitchDeg=std::atan(sf)/rad;
    out.rollDeg=-std::atan(sr)/rad;
    out.planeZ=0.f;
    for (const auto& h:hits) out.planeZ+=h.height/5.f;
    const std::array<float,5> predicted{
        out.planeZ, out.planeZ+sf*radius,out.planeZ-sf*radius,
        out.planeZ-sr*radius,out.planeZ+sr*radius
    };
    float squares=0.f;
    for (std::size_t i=0;i<5;++i) {
        const float residual=hits[i].height-predicted[i];
        out.maxResidual=std::max(out.maxResidual,std::fabs(residual));
        squares+=residual*residual;
        if (i) out.maxStep=std::max(out.maxStep,std::fabs(hits[i].height-hits[0].height));
    }
    out.rmsResidual=std::sqrt(squares/5.f);
    out.centerResidual=hits[0].height-out.planeZ;
    out.centerActorDelta=actorZ-hits[0].height;
    out.planeNormal=Unit({-sr*right.x-sf*forward.x,-sr*right.y-sf*forward.y,1.f});
    if (!Finite(out.pitchDeg) || !Finite(out.rollDeg) || !Finite(out.planeZ) ||
        !Finite(out.maxResidual) || !Finite(out.maxStep) ||
        out.planeNormal.z<0.f) { out.result=Result::kInvalidInput; return out; }
    for (const auto& h:hits) {
        if (h.actorReference || h.furnitureReference) {
            out.result=Result::kActorOrFurniture; return out;
        }
        if (!IsConservativeSurfaceLayer(h.collisionLayer)) {
            out.result=Result::kUntrustedLayer; return out;
        }
    }
    for (const auto& h:hits) {
        const auto n=Unit(h.normal);
        if (n.z<lim.minUpNormalZ) {
            out.result=Result::kSteepHitNormal; return out;
        }
        const float dot=std::clamp(Dot(n,out.planeNormal),-1.f,1.f);
        const float err=std::acos(dot)/rad;
        out.maxNormalDeviationDeg=std::max(out.maxNormalDeviationDeg,err);
    }
    if (out.maxNormalDeviationDeg>lim.maxNormalDeviationDeg) {
        out.result=Result::kNormalMismatch; return out;
    }
    if (std::fabs(out.centerActorDelta)>lim.maxActorOffset) {
        out.result=Result::kActorGroundOffset; return out;
    }
    if (out.maxStep>lim.maxCenterStep) {
        out.result=Result::kDiscontinuousStep; return out;
    }
    if (out.maxResidual>lim.maxPlaneResidual) {
        out.result=Result::kNonPlanar; return out;
    }
    if (std::fabs(out.pitchDeg)>maxSlopeDeg || std::fabs(out.rollDeg)>maxSlopeDeg) {
        out.result=Result::kOverSlopeLimit; return out;
    }

    out.result=Result::kGeometryCandidateOnly;
    return out;
}
}
