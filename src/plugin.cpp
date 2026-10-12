

#include "PCH.h"
#include "SKSEMenuFramework.h"
#include "OStimLegacyThreadAPI.h"
#include "NSTActorTiltInteropState.h"
#include "NSTIndoorSurfaceValidation.h"
#include "NSTIndoorTiltAuthorization.h"
#include "NSTSharedSlopeValidation.h"
#include "RE/B/bhkPickData.h"
#include "RE/B/bhkWorld.h"
#include "RE/C/CollisionLayers.h"
#include "RE/H/hkpCollidable.h"
#include "RE/H/hkpWorldObject.h"
#include <xmmintrin.h>
#include "TMSPoseWakeTimer.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <utility>
#include <functional>
#include <cstring>
#include <cctype>
#include <cwchar>
#include <vector>
#include <string_view>
#include <unordered_set>
#include <algorithm>

namespace {
constexpr const char* kSkeletonRoot = "NPC Root [Root]";
constexpr std::uint32_t kMaxHierarchyDepth = 48;
constexpr std::uint32_t kNoChild = std::numeric_limits<std::uint32_t>::max();
constexpr std::uint32_t kPlayerID = 0x14;
constexpr float kRad = 0.017453292519943295769f;
constexpr float kMaxPairDistance = 2048.f;
constexpr float kMaxLocalMove = 2048.f;

enum class Framework : std::uint8_t { kNone, kOStim, kSexLab };
const char* FrameworkName(Framework f);

void EnqueueGameTask(std::function<void()> fn);
std::atomic<std::uint64_t> eventEpoch{0};
std::atomic<std::uint32_t> activeThread{0};
std::atomic<Framework> activeFramework{Framework::kNone};
std::atomic<bool> activeSession{false};
std::atomic<std::uint64_t> activeStartEpoch{0};
std::atomic<std::uint32_t> actorBID{0};

std::atomic<std::uint64_t> pendingStartEpoch{0};
std::atomic<std::uint32_t> pendingThread{0};
std::atomic<bool> autoTerrainEnabled{true};
std::atomic<bool> graphAuditOnly{false};
std::atomic<bool> auditOnlyActive{false};
std::atomic<bool> localAuditEnabled{true};
std::atomic<std::int32_t> localAuditDelayMs{600};
std::atomic<bool> syncGuardEnabled{true};
std::atomic<std::int32_t> syncDelayMs{48};
std::atomic<std::int32_t> syncMaxRecoveries{8};
std::atomic<float> syncMaxRootGap{64.f};
constexpr float kTerrainSampleRadius = 140.f;

std::atomic<bool> indoorProbeEnabled{true};
std::atomic<float> indoorProbeRadius{64.f};
std::atomic<float> indoorRayAbove{48.f};
std::atomic<float> indoorRayBelow{128.f};
std::atomic<float> indoorMaxPlaneResidual{8.f};
std::atomic<float> indoorMaxCenterStep{80.f};
std::atomic<float> indoorMaxActorOffset{48.f};
std::atomic<float> indoorMinNormalUpZ{0.65f};
std::atomic<float> indoorMaxNormalDeviationDeg{25.f};
std::atomic<bool> indoorTiltEnabled{true};
std::atomic<float> appliedRoll{0.f};
std::atomic<bool> pairApplied{false};
std::atomic<bool> fatalGraphFault{false};
std::atomic<float> appliedPitch{0.f};
std::atomic<bool> sexLabEnabled{false};
std::atomic<std::int32_t> configuredLogLevel{static_cast<std::int32_t>(spdlog::level::info)};
std::atomic<std::uint32_t> legacyThreadsABI{0};
constexpr const char* kNSTVersion = "1.0.0";
constexpr const wchar_t* kNSTIniPath = L"Data/SKSE/Plugins/TMS_NSFWSceneTilt.ini";

const char* LogLevelName(std::int32_t level) {
    switch (static_cast<spdlog::level::level_enum>(level)) {
    case spdlog::level::trace: return "trace";
    case spdlog::level::debug: return "debug";
    case spdlog::level::info: return "info";
    case spdlog::level::warn: return "warn";
    case spdlog::level::err: return "error";
    case spdlog::level::critical: return "critical";
    case spdlog::level::off: return "off";
    default: return "info";
    }
}

std::int32_t ReadLogLevel(const std::filesystem::path& ini) {
    wchar_t text[32]{};
    ::GetPrivateProfileStringW(L"Logging", L"Level", L"", text, 32, ini.c_str());
    if (text[0]) {
        static constexpr const wchar_t* names[]{ L"trace", L"debug", L"info", L"warn", L"error", L"critical", L"off" };
        for (std::int32_t i=0;i<static_cast<std::int32_t>(std::size(names));++i) {
            if (::_wcsicmp(text,names[i])==0) return i;
        }
        spdlog::warn("[Logging] Invalid Level in INI; using info");
        return static_cast<std::int32_t>(spdlog::level::info);
    }

    const bool wasEnabled=::GetPrivateProfileIntW(L"Logging",L"bEnableLog",1,ini.c_str())!=0;
    const bool wasDebug=::GetPrivateProfileIntW(L"Logging",L"bDebugLog",0,ini.c_str())!=0;
    return !wasEnabled ? static_cast<std::int32_t>(spdlog::level::off) :
        (wasDebug ? static_cast<std::int32_t>(spdlog::level::debug) : static_cast<std::int32_t>(spdlog::level::info));
}

void SetLogLevel(std::int32_t level) {
    level=std::clamp<std::int32_t>(level,static_cast<std::int32_t>(spdlog::level::trace),
        static_cast<std::int32_t>(spdlog::level::off));
    configuredLogLevel.store(level);
    spdlog::set_level(static_cast<spdlog::level::level_enum>(level));
}

std::atomic<bool> furnitureAbiWarningShown{false};
std::atomic<bool> papyrusFallbackEnabled{false};
std::atomic<std::int32_t> maxStartAgeMs{200};
std::atomic<std::int32_t> minPoseAgeMs{32};
std::atomic<std::int32_t> poseSampleSpacingMs{16};
std::atomic<float> maxUnsettledRootGap{64.f};
std::atomic<std::uint64_t> nativeNodeSerial{0};
std::atomic<std::int64_t> signalAtMs{0};
std::atomic<bool> nativeAPIReady{false};
std::atomic<bool> multiEnabled{true};
std::atomic<float> multiMaxSlopeDegrees{35.f};
std::atomic<bool> furnitureProbeEnabled{true};
std::atomic<bool> skipBoundFurniture{true};
std::atomic<std::uint32_t> furnitureRefID{0};
std::atomic<std::uint32_t> multiActiveCount{0};
std::atomic<bool> soloApplied{false};
constexpr std::size_t kMaxActors=5;
NSTActorTiltInteropState nstInteropState;
bool nstPairWorldVerified=false;
void RevokeNSTInterop(const char* reason) {
    if (nstInteropState.Revoke()) {
        spdlog::debug("[NSTInterop] INACTIVE reason={}", reason);
    }
}

OStimLegacy::ThreadInterface* ostimAPI=nullptr;
using SteadyClock = std::chrono::steady_clock;
std::int64_t NowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(SteadyClock::now().time_since_epoch()).count();
}
std::int64_t SignalAgeMs() {
    const auto t=signalAtMs.load();
    return t ? std::max<std::int64_t>(0,NowMs()-t) : -1;
}
void Timeline(const char* stage, std::uint32_t thread, std::uint64_t epoch) {
    spdlog::debug("[Timeline] stage={} thread={} epoch={} elapsedMs={} limitMs={} fadeState=UNVERIFIED",
        stage,thread,epoch,SignalAgeMs(),maxStartAgeMs.load());
}
TMSPoseWakeTimer& LocalAuditWakeTimer();
TMSPoseWakeTimer& SyncWakeTimer();
void SyncPairOnGameThread(std::uint32_t thread, std::uint64_t epoch);
void SyncSoloOnGameThread(std::uint32_t thread, std::uint64_t epoch);
void SyncMultiOnGameThread(std::uint32_t thread, std::uint64_t epoch);
void ProbeMultiPoseOnGameThread(std::uint32_t thread,std::uint64_t epoch);
void AuditMulti(const char* phase);
void PublishNSTInterop(std::uint32_t tid, std::uint64_t epoch);
void ReadFurnitureOnGameThread(std::uint32_t tid, std::uint32_t refID, const char* stage);

bool WithinStartDeadline(std::uint32_t thread,std::uint64_t epoch,const char* stage) {
    const auto age=SignalAgeMs();
    if (age<0 || age>maxStartAgeMs.load()) {
        spdlog::warn("[StartGuard] SKIP_LATE stage={} thread={} epoch={} elapsedMs={} deadlineMs={} (no late Actor3D write)",
            stage,thread,epoch,age,maxStartAgeMs.load());

        return false;
    }
    return true;
}
bool eventSinkInstalled=false;
bool menuRegistered=false;

std::mutex callbackMutex;
bool ostimSignalActive=false;
bool ostimCanonicalSeen=false;
std::uint32_t ostimSignalThread=0;
std::uint64_t ostimSignalEpoch=0;

RE::NiMatrix3 Identity() {
    RE::NiMatrix3 out{};
    out.entry[0][0] = out.entry[1][1] = out.entry[2][2] = 1.0f;
    return out;
}
RE::NiMatrix3 Transpose3(const RE::NiMatrix3& a) {
    RE::NiMatrix3 out{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out.entry[i][j] = a.entry[j][i];
    return out;
}
RE::NiMatrix3 Multiply(const RE::NiMatrix3& a, const RE::NiMatrix3& b) {
    RE::NiMatrix3 out{};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            out.entry[i][j] = 0.f;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                out.entry[i][j] += a.entry[i][k] * b.entry[k][j];
    return out;
}
RE::NiPoint3 Add(RE::NiPoint3 a, RE::NiPoint3 b) { return { a.x+b.x, a.y+b.y, a.z+b.z }; }
RE::NiPoint3 Sub(RE::NiPoint3 a, RE::NiPoint3 b) { return { a.x-b.x, a.y-b.y, a.z-b.z }; }
RE::NiPoint3 Mul(RE::NiPoint3 a, float s) { return { a.x*s, a.y*s, a.z*s }; }
RE::NiPoint3 Rotate(const RE::NiMatrix3& m, RE::NiPoint3 v) {
    return { m.entry[0][0]*v.x + m.entry[0][1]*v.y + m.entry[0][2]*v.z,
             m.entry[1][0]*v.x + m.entry[1][1]*v.y + m.entry[1][2]*v.z,
             m.entry[2][0]*v.x + m.entry[2][1]*v.y + m.entry[2][2]*v.z };
}
float Distance(RE::NiPoint3 a, RE::NiPoint3 b) {
    const auto d = Sub(a,b);
    return std::sqrt(d.x*d.x+d.y*d.y+d.z*d.z);
}
float RotationError(const RE::NiMatrix3& a, const RE::NiMatrix3& b) {
    float worst = 0.f;
    for (int i=0; i<3; ++i)
        for (int j=0; j<3; ++j)
            worst = std::max(worst, std::fabs(a.entry[i][j]-b.entry[i][j]));
    return worst;
}
bool Finite(RE::NiPoint3 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
bool Finite(const RE::NiMatrix3& r) {
    for (int i=0;i<3;++i)
        for (int j=0;j<3;++j)
            if (!std::isfinite(r.entry[i][j])) return false;
    return true;
}
RE::NiMatrix3 Pitch(float degrees) {
    auto r = Identity();
    const float a = degrees * kRad;
    r.entry[1][1]=std::cos(a); r.entry[1][2]=-std::sin(a);
    r.entry[2][1]=std::sin(a); r.entry[2][2]=std::cos(a);
    return r;
}

RE::NiMatrix3 SharedMode6Rotation(const RE::NiMatrix3& parentA, float pitch) {
    return Multiply(Multiply(parentA, Pitch(pitch)), Transpose3(parentA));
}

RE::NiMatrix3 Roll(float degrees) {
    auto r=Identity();
    const float a=degrees*kRad;
    r.entry[0][0]=std::cos(a); r.entry[0][2]=std::sin(a);
    r.entry[2][0]=-std::sin(a); r.entry[2][2]=std::cos(a);
    return r;
}
RE::NiPoint3 Cross(RE::NiPoint3 a, RE::NiPoint3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
RE::NiPoint3 Normalized(RE::NiPoint3 v) {
    const float len=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
    return std::isfinite(len) && len>0.0001f ? Mul(v,1.f/len) : RE::NiPoint3{0,0,0};
}
struct TerrainTilt {
    RE::NiMatrix3 worldRotation{};
    RE::NiPoint3 normal{};
    float pitch{0};
    float roll{0};
    float centerZ{0};
    float frontZ{0};
    float backZ{0};
    float leftZ{0};
    float rightZ{0};
    float radius{0};
    bool valid{false};
};

TerrainTilt ProbeIndoorSurface(RE::Actor* actor, const RE::NiMatrix3& parentWorld,
    float maxSlopeDegrees, std::uint32_t actorCount) {
    TerrainTilt out{};
    if (!indoorProbeEnabled.load() || !actor) return out;
    const auto* cell=actor->GetParentCell();
    if (!cell || !cell->IsInteriorCell()) return out;
    auto* world=cell->GetbhkWorld();
    if (!world) {
        spdlog::info("[IndoorProbe] SKIP reason=NO_BHK_WORLD cell={:08X} READ_ONLY",cell->GetFormID());
        return out;
    }
    const float scale=RE::bhkWorld::GetWorldScale();
    if (!std::isfinite(scale) || scale<=0.f || scale>1.f) {
        spdlog::warn("[IndoorProbe] SKIP reason=INVALID_WORLD_SCALE scale={:.6f} READ_ONLY",scale);
        return out;
    }
    const auto center=actor->GetPosition();
    if (!Finite(center)) return out;
    const float radius=std::clamp(indoorProbeRadius.load(),24.f,160.f);
    const float above=std::clamp(indoorRayAbove.load(),16.f,192.f);
    const float below=std::clamp(indoorRayBelow.load(),48.f,384.f);
    const auto right=Normalized({parentWorld.entry[0][0],parentWorld.entry[1][0],0.f});
    if (std::fabs(right.x)+std::fabs(right.y)<0.01f) return out;
    const RE::NiPoint3 forward{-right.y,right.x,0.f};
    const std::array<RE::NiPoint3,5> offsets{
        RE::NiPoint3{0,0,0}, Mul(forward,radius), Mul(forward,-radius),
        Mul(right,-radius), Mul(right,radius)
    };
    constexpr const char* labels[5]={"C","F","B","L","R"};
    std::array<NSTIndoor::Hit,5> hits{};
    const auto start=SteadyClock::now();
    std::int64_t rayOnlyUs=0;
    for (std::size_t i=0;i<offsets.size();++i) {
        const auto origin=Add(center,offsets[i]);
        const float startZ=origin.z+above;
        const float endZ=origin.z-below;
        RE::bhkPickData pick{};
        pick.rayInput.from=RE::hkVector4(origin.x*scale,origin.y*scale,startZ*scale,0.f);
        pick.rayInput.to=RE::hkVector4(origin.x*scale,origin.y*scale,endZ*scale,0.f);

        pick.rayInput.filterInfo = static_cast<std::uint32_t>(RE::COL_LAYER::kLOS);
        pick.rayOutput.Reset();
        const auto rayStart=SteadyClock::now();
        const bool picked=world->PickObject(pick);
        rayOnlyUs+=std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now()-rayStart).count();
        const float fraction=pick.rayOutput.hitFraction;
        auto& h=hits[i];
        h.valid=picked && pick.rayOutput.HasHit() &&
            std::isfinite(fraction) && fraction>=0.f && fraction<=1.f;
        std::uint32_t formID=0;
        std::uint32_t baseType=0;
        const RE::hkpCollidable* collidable=nullptr;
        if (h.valid) {
            h.height=startZ+(endZ-startZ)*fraction;
            h.valid=std::isfinite(h.height);
            collidable=pick.rayOutput.rootCollidable;
            if (collidable) {
                h.collisionLayer=static_cast<std::uint32_t>(collidable->GetCollisionLayer());

                if (auto* owner=collidable->GetOwner<RE::hkpWorldObject>()) {
                    if (auto* ref=owner->GetUserData()) {
                        formID=ref->GetFormID();
                        h.ownerResolved=(formID!=0);
                        h.actorReference=ref->As<RE::Actor>()!=nullptr;
                        if (auto* base=ref->GetBaseObject()) {
                            baseType=static_cast<std::uint32_t>(base->GetFormType());

                            h.staticReference=(base->GetFormType()==RE::FormType::Static);
                            h.furnitureReference=base->As<RE::TESFurniture>()!=nullptr;
                        }
                    }
                }
            }

            alignas(16) float n[4]{};
            _mm_storeu_ps(n,pick.rayOutput.normal.quad);
            h.normal={n[0],n[1],n[2]};
        }
        spdlog::debug("[IndoorProbe] HIT sample={} hit={} xyz=({:.2f},{:.2f},{:.2f}) rayZ=({:.2f},{:.2f}) hitFraction={:.5f} hitZ={:.2f} layer={} ref={:08X} baseType={} actor={} furniture={} normal=({:.3f},{:.3f},{:.3f}) collidable={} READ_ONLY",
            labels[i],h.valid,origin.x,origin.y,origin.z,startZ,endZ,fraction,
            h.valid?h.height:0.f,h.collisionLayer,formID,baseType,
            h.actorReference,h.furnitureReference,h.normal.x,h.normal.y,h.normal.z,
            static_cast<const void*>(collidable));
    }
    const auto micros=std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now()-start).count();
    NSTIndoor::Limits lim{};
    lim.maxPlaneResidual=std::clamp(indoorMaxPlaneResidual.load(),1.f,64.f);
    lim.maxCenterStep=std::clamp(indoorMaxCenterStep.load(),4.f,150.f);
    lim.maxActorOffset=std::clamp(indoorMaxActorOffset.load(),4.f,96.f);
    lim.minUpNormalZ=std::clamp(indoorMinNormalUpZ.load(),0.1f,1.f);
    lim.maxNormalDeviationDeg=std::clamp(indoorMaxNormalDeviationDeg.load(),1.f,89.f);
    const auto v=NSTIndoor::Evaluate(hits,radius,center.z,
        {right.x,right.y,0.f},{forward.x,forward.y,0.f},maxSlopeDegrees,lim);
    spdlog::debug("[IndoorProbe] VALIDATE cell={:08X} rays={}/5 radius={:.1f} rawPitch={:.2f} rawRoll={:.2f} maxStep={:.2f} actorOffset={:.2f} planeZ={:.2f} centerResidual={:.2f} maxResidual={:.2f} rmsResidual={:.2f} maxNormalErrDeg={:.2f} planeNormal=({:.3f},{:.3f},{:.3f}) rayOnlyUs={} probeElapsedUs={} result={} STATUS={} PROBE_READ_ONLY",
        cell->GetFormID(),v.hitCount,radius,v.pitchDeg,v.rollDeg,v.maxStep,
        v.centerActorDelta,v.planeZ,v.centerResidual,v.maxResidual,v.rmsResidual,
        v.maxNormalDeviationDeg,v.planeNormal.x,v.planeNormal.y,v.planeNormal.z,
        rayOnlyUs,micros,NSTIndoor::ResultName(v.result),
        v.candidate()?"DIAGNOSTIC_CANDIDATE_NOT_PROVEN":"DIAGNOSTIC_REJECTED_NOT_A_WRITE_GATE");

    const std::array<float,5> heights{hits[0].height,hits[1].height,hits[2].height,
        hits[3].height,hits[4].height};
    const auto shared=NSTSharedSlope::Evaluate(heights,center.z,radius,maxSlopeDegrees);
    const auto auth=NSTIndoorTilt::Authorize(shared,hits,indoorTiltEnabled.load(),
        indoorProbeEnabled.load(),actorCount);
    spdlog::debug("[IndoorTilt] REVIEW cell={:08X} scope=ALL_INTERIORS actors={} enabled={} authorized={} reason={} "
        "sharedSlope={} pitch={:.2f} roll={:.2f} normalZ={:.4f} actorOffset={:.2f} "
        "maxCenterStep={:.2f} diagnosticResidual={:.2f} diagnosticNormalErr={:.2f} NO_GRAPH_WRITES_YET",
        cell->GetFormID(),actorCount,indoorTiltEnabled.load(),
        auth.authorized,NSTIndoorTilt::ReasonName(auth.reason),NSTSharedSlope::ReasonName(shared.reason),
        shared.pitchDeg,shared.rollDeg,shared.normalZ,shared.actorOffset,
        shared.maxCenterStep,v.maxResidual,v.maxNormalDeviationDeg);
    if (!auth.authorized) {
        spdlog::info("[Terrain] SKIP_INDOOR actors={} reason={} slope={}",
            actorCount,NSTIndoorTilt::ReasonName(auth.reason),NSTSharedSlope::ReasonName(shared.reason));
        return out;
    }
    const RE::NiMatrix3 yaw=[&] {
        auto basis=Identity();
        basis.entry[0][0]=right.x; basis.entry[1][0]=right.y;
        basis.entry[0][1]=forward.x; basis.entry[1][1]=forward.y;
        return basis;
    }();
    out.pitch=shared.pitchDeg;
    out.roll=shared.rollDeg;
    out.centerZ=hits[0].height;
    out.frontZ=hits[1].height;
    out.backZ=hits[2].height;
    out.leftZ=hits[3].height;
    out.rightZ=hits[4].height;
    out.radius=radius;
    const RE::NiPoint3 frontTangent{forward.x,forward.y,shared.slopeForward};
    const RE::NiPoint3 rightTangent{right.x,right.y,shared.slopeRight};
    out.normal=Normalized(Cross(rightTangent,frontTangent));
    if (out.normal.z<0.f) out.normal=Mul(out.normal,-1.f);

    out.worldRotation=Multiply(Multiply(yaw,Multiply(Pitch(out.pitch),Roll(out.roll))),Transpose3(yaw));
    out.valid=Finite(out.worldRotation) && Finite(out.normal);
    spdlog::debug("[IndoorTilt] {} prepared {}P pitch={:.2f} roll={:.2f} cell={:08X} "
        "(original pair/multi transaction still required)",
        out.valid?"AUTHORIZED":"REJECT_MATRIX",actorCount,out.pitch,out.roll,cell->GetFormID());
    return out;
}

TerrainTilt SampleOutdoorLand(RE::Actor* actor, const RE::NiMatrix3& aParentWorld,
    float maxSlopeDegrees=30.f, std::uint32_t actorCount=2) {
    TerrainTilt out{};
    if (!actor || !actor->GetParentCell()) {
        spdlog::info("[Terrain] SKIP reason=NO_ACTOR_OR_CELL NO_GRAPH_WRITES");
        return out;
    }
    if (actor->GetParentCell()->IsInteriorCell()) {
        spdlog::debug("[Terrain] INTERIOR_ROUTE cell={:08X} LAND_SKIPPED actorCount={} indoorEnabled={}",
            actor->GetParentCell()->GetFormID(),actorCount,indoorTiltEnabled.load());
        return ProbeIndoorSurface(actor,aParentWorld,maxSlopeDegrees,actorCount);
    }
    auto* tes=RE::TES::GetSingleton();
    if (!tes) return out;
    const auto center=actor->GetPosition();
    if (!Finite(center)) return out;
    const float radius=kTerrainSampleRadius;

    const auto right=Normalized({aParentWorld.entry[0][0],aParentWorld.entry[1][0],0.f});
    if (std::fabs(right.x)+std::fabs(right.y)<0.01f) return out;
    const RE::NiPoint3 forward{-right.y,right.x,0.f};
    const auto at=[&](RE::NiPoint3 dir,float factor) {
        return Add(center,Mul(dir,factor*radius));
    };
    if (!tes->GetLandHeight(center,out.centerZ) ||
        !tes->GetLandHeight(at(forward,1),out.frontZ) ||
        !tes->GetLandHeight(at(forward,-1),out.backZ) ||
        !tes->GetLandHeight(at(right,-1),out.leftZ) ||
        !tes->GetLandHeight(at(right,1),out.rightZ)) {
        spdlog::info("[Terrain] SKIP missing land height sample");
        return out;
    }
    if (!std::isfinite(out.centerZ) || !std::isfinite(out.frontZ) ||
        !std::isfinite(out.backZ) || !std::isfinite(out.leftZ) ||
        !std::isfinite(out.rightZ)) return out;

    const std::array<float,5> heights{out.centerZ,out.frontZ,out.backZ,out.leftZ,out.rightZ};
    const auto shared=NSTSharedSlope::Evaluate(heights,center.z,radius,maxSlopeDegrees);
    if (!shared.valid()) {
        spdlog::info("[Terrain] SKIP sharedSlope={} actorOffset={:.2f} maxCenterStep={:.2f} normalZ={:.4f} pitch={:.2f} roll={:.2f}",
            NSTSharedSlope::ReasonName(shared.reason),shared.actorOffset,shared.maxCenterStep,
            shared.normalZ,shared.pitchDeg,shared.rollDeg);
        return out;
    }
    const RE::NiPoint3 frontTangent{forward.x, forward.y, shared.slopeForward};
    const RE::NiPoint3 rightTangent{right.x,right.y,shared.slopeRight};
    out.normal=Normalized(Cross(rightTangent,frontTangent));
    if (out.normal.z<0.f) out.normal=Mul(out.normal,-1.f);
    out.pitch=shared.pitchDeg;
    out.roll=shared.rollDeg;
    RE::NiMatrix3 yaw=Identity();
    yaw.entry[0][0]=right.x; yaw.entry[1][0]=right.y;
    yaw.entry[0][1]=forward.x; yaw.entry[1][1]=forward.y;
    out.worldRotation=Multiply(Multiply(yaw,Multiply(Pitch(out.pitch),Roll(out.roll))),Transpose3(yaw));
    out.radius=radius;
    out.valid=Finite(out.worldRotation);
    spdlog::debug("[Terrain] fivePoint land centerZ={:.2f} frontZ={:.2f} backZ={:.2f} leftZ={:.2f} rightZ={:.2f} radius={:.1f} normal=({:.4f},{:.4f},{:.4f}) pitch={:.2f} roll={:.2f} valid={}",
        out.centerZ,out.frontZ,out.backZ,out.leftZ,out.rightZ,radius,out.normal.x,out.normal.y,out.normal.z,out.pitch,out.roll,out.valid);
    return out;
}
std::uint32_t ChildIndex(RE::NiNode* parent, RE::NiAVObject* child) {
    if (!parent || !child) return kNoChild;
    const auto& children = parent->GetChildren();
    for (std::uint32_t i=0; i<children.capacity(); ++i)
        if (children[i].get() == child) return i;
    return kNoChild;
}
const char* NodeName(const RE::NiAVObject* node) {
    if (!node) return "<null>";
    const char* s = node->name.c_str();
    return s && *s ? s : "<unnamed>";
}
struct RootLocation { RE::NiNode* parent{nullptr}; std::uint32_t depth{0}; bool valid{false}; };
RootLocation LocateRoot(RE::NiAVObject* outer, RE::NiAVObject* root, std::uint32_t id, bool verbose=true) {
    if (!outer || !outer->AsNode() || !root || root==outer) return {};
    std::array<RE::NiAVObject*, kMaxHierarchyDepth> seen{};
    auto* n = root;
    std::uint32_t depth = 0;
    while (n && depth<kMaxHierarchyDepth) {
        if (n==outer) {
            if (verbose) spdlog::debug("[PairHierarchy] actor={:08X} VALID depth={} rootParent='{}'", id, depth, NodeName(root->parent));
            return {root->parent, depth, root->parent!=nullptr};
        }
        for (std::uint32_t i=0; i<depth; ++i) if (seen[i]==n) return {};
        seen[depth]=n;
        auto* p = n->parent;
        if (!p || ChildIndex(p,n)==kNoChild) return {};
        if (verbose && depth<5) spdlog::debug("[PairHierarchy] actor={:08X} link={} child='{}' parent='{}'", id, depth, NodeName(n), NodeName(p));
        n=p;
        ++depth;
    }
    if (verbose) spdlog::warn("[PairHierarchy] actor={:08X} INVALID root not owned by Actor3D", id);
    return {};
}
bool AncestorOwned(RE::NiAVObject* outer, RE::NiAVObject* child) {
    for (std::uint32_t d=0; child && d<kMaxHierarchyDepth; ++d) {
        if (child==outer) return true;
        auto* p=child->parent;
        if (!p || ChildIndex(p,child)==kNoChild) return false;
        child=p;
    }
    return false;
}

struct LocalState {
    std::uint32_t actorID{0};
    RE::NiPointer<RE::NiAVObject> actor3D{};
    RE::NiPointer<RE::NiAVObject> npcRoot{};
    RE::NiPointer<RE::NiNode> npcParent{};
    RE::NiPointer<RE::NiNode> carrierParent{};
    RE::NiPointer<RE::NiNode> actorParent{};
    std::uint32_t npcSlot{kNoChild};
    std::uint32_t carrierSlot{kNoChild};
    std::uint32_t actorSlot{kNoChild};
    RE::NiTransform before{};
    RE::NiTransform applied{};
    RE::NiMatrix3 carrierWorldAfter{};
    RE::NiMatrix3 actorWorldAfter{};
    RE::NiPoint3 actorWorldStartPos{};
    bool valid{false};
};
LocalState pair[2]{};

std::uint32_t syncRecoveryCount=0;
bool syncFailClosed=false;
float syncBaselineRootGap=0.f;
RE::NiMatrix3 syncSharedWorldRotation{};
RE::NiPoint3 syncSharedPivot{};

void RefreshOnce(RE::NiAVObject* node) {
    if (!node) return;
    RE::NiUpdateData data{};
    node->UpdateDownwardPass(data,0);
}
bool CurrentActor3D(const LocalState& s) {
    if (!s.valid || !s.actor3D) return false;
    auto* actor=RE::TESForm::LookupByID<RE::Actor>(s.actorID);
    return actor && actor->Is3DLoaded() && actor->Get3D()==s.actor3D.get();
}
bool OuterParentUnchanged(const LocalState& s) {
    if (!CurrentActor3D(s) || s.actor3D->parent!=s.actorParent.get()) return false;
    return !s.actorParent || ChildIndex(s.actorParent.get(),s.actor3D.get())==s.actorSlot;
}
bool NpcTopologyUnchanged(const LocalState& s) {

    return OuterParentUnchanged(s) && s.npcRoot && s.npcParent && s.carrierParent &&
        s.npcParent->parent==s.carrierParent.get() &&
        ChildIndex(s.carrierParent.get(),s.npcParent.get())==s.carrierSlot &&
        s.npcRoot->parent==s.npcParent.get() &&
        ChildIndex(s.npcParent.get(),s.npcRoot.get())==s.npcSlot &&
        s.actor3D->GetObjectByName(kSkeletonRoot)==s.npcRoot.get();
}
bool BothOwned() { return OuterParentUnchanged(pair[0]) && OuterParentUnchanged(pair[1]); }
void ClearPair() {
    pair[0]={}; pair[1]={};
    syncRecoveryCount=0;
    syncFailClosed=false;
    syncBaselineRootGap=0.f;
    syncSharedWorldRotation=Identity();
    syncSharedPivot={};
    pairApplied.store(false);
    appliedPitch.store(0.f);
    appliedRoll.store(0.f);
}

LocalState solo{};
RE::NiMatrix3 soloWorldTilt{};
RE::NiPoint3 soloPivot{};
std::uint32_t soloRecoveries=0;
bool soloFailClosed=false;
bool soloWorldVerified=false;
void ClearSolo() {
    solo={};
    soloWorldTilt=Identity();
    soloPivot={};
    soloRecoveries=0;
    soloFailClosed=false;
    soloWorldVerified=false;
    soloApplied.store(false);
    pairApplied.store(false);
    appliedPitch.store(0.f);
    appliedRoll.store(0.f);
}

std::array<LocalState,kMaxActors> multi{};
std::array<std::uint32_t,kMaxActors> multiRoster{};
std::size_t multiCount=0;
std::uint32_t multiRecoveries=0;
bool multiFailClosed=false;
RE::NiMatrix3 multiWorldTilt{};
RE::NiPoint3 multiPivot{};
float multiBaselineMaxGap=0.f;
struct MultiPoseSample {
    std::array<RE::NiPoint3,kMaxActors> parent{},root{};
    std::array<RE::NiMatrix3,kMaxActors> rotation{};
    std::int64_t atMs{0};
    float maxGap{0.f};
    bool valid{false};
};
struct MultiProbeState {
    std::uint32_t thread{0}, attempts{0}, stable{0};
    std::uint64_t epoch{0}, initialNodeSerial{0};
    bool everAligned{false}, committed{false};
    MultiPoseSample qualified{},last{};
};
MultiProbeState multiProbe{};
void ClearMulti() {
    for (auto& s:multi) s={};
    multiCount=0;
    multiRoster={};
    multiRecoveries=0;
    multiFailClosed=false;
    multiWorldTilt=Identity();
    multiPivot={};
    multiBaselineMaxGap=0.f;
    multiActiveCount.store(0);
    pairApplied.store(false);
    appliedPitch.store(0.f);
    appliedRoll.store(0.f);
    multiProbe={};
}
bool MultiHasState() {
    if (multiCount) return true;
    for (const auto& s:multi) if (s.valid) return true;
    return false;
}
struct Candidate {
    std::uint32_t id{0};
    RE::NiAVObject* outer{nullptr};
    RE::NiAVObject* root{nullptr};
    RE::NiNode* parent{nullptr};
    std::uint32_t slot{kNoChild};
    std::uint32_t depth{0};
    bool valid{false};
};
Candidate Inspect(std::uint32_t id, bool verbose=true) {
    Candidate c{}; c.id=id;
    auto* actor=RE::TESForm::LookupByID<RE::Actor>(id);
    if (!actor || !actor->Is3DLoaded()) return c;
    c.outer=actor->Get3D();
    if (!c.outer || !c.outer->AsNode()) return c;
    c.root=c.outer->GetObjectByName(kSkeletonRoot);
    if (!c.root) return c;
    const auto location=LocateRoot(c.outer,c.root,id,verbose);
    c.parent=location.parent; c.depth=location.depth;
    c.slot=ChildIndex(c.parent,c.root);

    c.valid=location.valid && c.depth==2 && c.slot!=kNoChild &&
        c.parent && c.parent->parent &&
        ChildIndex(c.parent->parent,c.parent)!=kNoChild &&
        (!c.outer->parent || ChildIndex(c.outer->parent,c.outer)!=kNoChild);
    return c;
}
struct LocalPlan {
    RE::NiMatrix3 rotation{};
    RE::NiPoint3 translation{};
    RE::NiPoint3 neutralRootWorld{};
    RE::NiPoint3 expectedRootWorld{};
    RE::NiMatrix3 expectedRootRotation{};
    float move{0.f};
    bool valid{false};
};

LocalPlan PlanNPCParent(const Candidate& c, const RE::NiMatrix3& commonWorld,
                        RE::NiPoint3 pivot) {
    LocalPlan plan{};
    if (!c.valid || !Finite(commonWorld) || !Finite(pivot)) return plan;
    auto* carrier=c.parent;
    auto* parent=carrier ? carrier->parent : nullptr;
    if (!parent) return plan;
    const auto& world=carrier->world;
    const auto& ancestor=parent->world;
    const float scale=ancestor.scale;
    if (!Finite(world.translate) || !Finite(world.rotate) ||
        !Finite(ancestor.translate) || !Finite(ancestor.rotate) ||
        !std::isfinite(scale) || std::fabs(scale)<0.01f) return plan;
    const auto wantedRotation=Multiply(commonWorld,world.rotate);
    const auto wantedTranslation=Add(pivot,Rotate(commonWorld,Sub(world.translate,pivot)));
    const auto inverseAncestor=Transpose3(ancestor.rotate);
    plan.rotation=Multiply(inverseAncestor,wantedRotation);
    plan.translation=Mul(Rotate(inverseAncestor,Sub(wantedTranslation,ancestor.translate)),1.f/scale);
    const auto& rootLocal=c.root->local;
    if (!Finite(rootLocal.translate) || !Finite(rootLocal.rotate) ||
        !std::isfinite(world.scale) || std::fabs(world.scale)<0.01f) return plan;
    plan.neutralRootWorld=Add(world.translate,Rotate(world.rotate,Mul(rootLocal.translate,world.scale)));
    plan.expectedRootWorld=Add(pivot,Rotate(commonWorld,Sub(plan.neutralRootWorld,pivot)));
    plan.expectedRootRotation=Multiply(commonWorld,Multiply(world.rotate,rootLocal.rotate));
    plan.move=Distance(plan.translation,carrier->local.translate);
    plan.valid=Finite(plan.rotation) && Finite(plan.translation) &&
        Finite(plan.expectedRootWorld) && Finite(plan.expectedRootRotation) &&
        std::isfinite(plan.move) && plan.move<=kMaxLocalMove;
    return plan;
}

bool ApplyOneLocal(const Candidate& c, const LocalPlan& plan, LocalState& result) {
    if (!c.valid || !plan.valid || result.valid) return false;
    auto* carrier=c.parent;
    auto* ancestor=carrier ? carrier->parent : nullptr;
    if (!ancestor || ChildIndex(ancestor,carrier)==kNoChild) return false;
    result.actorID=c.id;
    result.actor3D.reset(c.outer);
    result.npcRoot.reset(c.root);
    result.npcParent.reset(carrier);
    result.carrierParent.reset(ancestor);
    result.actorParent.reset(c.outer->parent);
    result.npcSlot=c.slot;
    result.carrierSlot=ChildIndex(ancestor,carrier);
    result.actorSlot=c.outer->parent ? ChildIndex(c.outer->parent,c.outer) : kNoChild;
    result.before=carrier->local;
    result.applied=result.before;
    result.applied.rotate=plan.rotation;
    result.applied.translate=plan.translation;
    result.valid=true;
    carrier->local.rotate=result.applied.rotate;
    carrier->local.translate=result.applied.translate;
    RefreshOnce(carrier);
    result.carrierWorldAfter=carrier->world.rotate;
    result.actorWorldAfter=c.outer->world.rotate;
    result.actorWorldStartPos=c.outer->world.translate;
    spdlog::debug("[LocalCommit] APPLY actor={:08X} carrier='{}' carrierPtr=0x{:X} carrierParent='{}' rootParent='{}' npcSlot={} beforePos=({:.3f},{:.3f},{:.3f}) appliedPos=({:.3f},{:.3f},{:.3f}) move={:.3f} parentLinksWritten=0 actor3DLocalWritten=0",
        c.id,NodeName(carrier),reinterpret_cast<std::uintptr_t>(carrier),NodeName(ancestor),NodeName(c.root->parent),c.slot,
        result.before.translate.x,result.before.translate.y,result.before.translate.z,
        result.applied.translate.x,result.applied.translate.y,result.applied.translate.z,plan.move);
    return true;
}
void AuditOneLocal(const LocalState& s, const char* phase) {
    if (!localAuditEnabled.load() || !s.valid) return;
    if (!CurrentActor3D(s)) {
        spdlog::warn("[LocalAudit] actor={:08X} phase={} actor3D=REPLACED_OR_UNLOADED noStaleDereference=true",s.actorID,phase);
        return;
    }
    const bool topo=NpcTopologyUnchanged(s);
    if (!topo) {
        spdlog::warn("[LocalAudit] TOPOLOGY_CHANGED actor={:08X} phase={} carrierIdentityLost=true READ_ONLY",s.actorID,phase);
        return;
    }
    const auto& now=s.npcParent->local;
    const auto posDelta=Distance(now.translate,s.applied.translate);
    const auto rotDelta=RotationError(now.rotate,s.applied.rotate);

    const auto worldDelta=RotationError(s.npcParent->world.rotate,s.carrierWorldAfter);
    const auto outerDelta=RotationError(s.actor3D->world.rotate,s.actorWorldAfter);
    spdlog::debug("[LocalAudit] actor={:08X} phase={} carrier=NPC carrierPtr=0x{:X} rotationDelta={:.5f} translationDelta={:.4f} rotationHeld={} translationHeld={} worldRotDelta={:.5f} actorWorldRotDelta={:.5f} npcTopologySame=true actorParentSame=true localOverwriteDetected={} READ_ONLY",
        s.actorID,phase,reinterpret_cast<std::uintptr_t>(s.npcParent.get()),rotDelta,posDelta,
        rotDelta<=0.01f,posDelta<=0.10f,worldDelta,outerDelta,rotDelta>0.01f || posDelta>0.10f);
    if (rotDelta>0.01f || posDelta>0.10f)
        spdlog::warn("[LocalAudit] CARRIER_OVERWRITTEN actor={:08X} phase={} rotationDelta={:.5f} translationDelta={:.4f} nowPos=({:.3f},{:.3f},{:.3f}) basePos=({:.3f},{:.3f},{:.3f}) baseRotationDelta={:.5f} syncGuardEventOnly=true",
            s.actorID,phase,rotDelta,posDelta,
            now.translate.x,now.translate.y,now.translate.z,
            s.before.translate.x,s.before.translate.y,s.before.translate.z,
            RotationError(now.rotate,s.before.rotate));
}
void AuditLocalPair(const char* phase) {
    AuditOneLocal(pair[0],phase);
    AuditOneLocal(pair[1],phase);
}

bool RestoreOneLocal(LocalState& s, const char* reason) {
    if (!s.valid) return true;
    if (!CurrentActor3D(s) || !NpcTopologyUnchanged(s)) {
        spdlog::warn("[LocalRestore] SKIP actor={:08X} reason={} status=ACTOR3D_OR_TOPOLOGY_CHANGED restored=false discarded=true sessionStateCleaned=true",s.actorID,reason);
        s={};
        return true;
    }
    auto* carrier=s.npcParent.get();
    const auto rotErr=RotationError(carrier->local.rotate,s.applied.rotate);
    const auto posErr=Distance(carrier->local.translate,s.applied.translate);
    const bool ownsRot=rotErr<=0.01f;
    const bool ownsPos=posErr<=0.10f;

    const bool restoreRot=ownsRot && RotationError(carrier->local.rotate,s.before.rotate)>0.0001f;
    const bool restorePos=ownsPos && Distance(carrier->local.translate,s.before.translate)>0.001f;
    if (restoreRot) carrier->local.rotate=s.before.rotate;
    if (restorePos) carrier->local.translate=s.before.translate;
    if (restoreRot || restorePos) RefreshOnce(carrier);
    spdlog::debug("[LocalRestore] RESULT actor={:08X} reason={} restoreRotation={} restoreTranslation={} engineOwnedRotation={} engineOwnedTranslation={} topologySame=true graphWrites={} completed=true",
        s.actorID,reason,restoreRot,restorePos,!ownsRot,!ownsPos,
        static_cast<int>(restoreRot)+static_cast<int>(restorePos));
    s={};
    return true;
}
bool RestorePairLocal(const char* reason) {
    AuditLocalPair("END_PRE_RESTORE");
    const bool b=RestoreOneLocal(pair[1],reason);
    const bool a=RestoreOneLocal(pair[0],reason);
    ClearPair();
    spdlog::debug("[LocalRestore] PAIR_CLEAN A={} B={} reason={} nextSceneReady=true",a,b,reason);

    return a && b;
}
float MultiMaxRootGap() {
    if (multiCount<3 || multiCount>kMaxActors) return -1.f;
    float maxGap=0.f;
    for (std::size_t i=0;i<multiCount;++i) {
        if (!multi[i].valid || !multi[i].npcRoot || !Finite(multi[i].npcRoot->world.translate)) return -1.f;
        for (std::size_t j=0;j<i;++j)
            maxGap=std::max(maxGap,Distance(multi[i].npcRoot->world.translate,multi[j].npcRoot->world.translate));
    }
    return maxGap;
}

float WorldUpAngleDegrees(RE::NiPoint3 a, RE::NiPoint3 b) {
    a=Normalized(a);
    b=Normalized(b);
    const float lenA=std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
    const float lenB=std::sqrt(b.x*b.x+b.y*b.y+b.z*b.z);
    if (lenA<0.5f || lenB<0.5f) return -1.f;
    return std::acos(std::clamp(a.x*b.x+a.y*b.y+a.z*b.z,-1.f,1.f))/kRad;
}
void AuditMultiWorld(const char* phase) {
    if (!localAuditEnabled.load() || multiCount<3 || multiCount>kMaxActors) return;
    const RE::NiPoint3 up{0.f,0.f,1.f};
    float maxTiltError=0.f, maxUpError=-1.f, maxActorMove=0.f;
    RE::NiPoint3 firstExpectedUp{},firstObservedUp{};
    float maxExpectedUpSpread=0.f, maxObservedUpSpread=0.f;
    bool first=true;
    std::size_t valid=0;
    for (std::size_t i=0;i<multiCount;++i) {
        const auto& s=multi[i];
        if (!s.valid || !CurrentActor3D(s) || !NpcTopologyUnchanged(s)) {
            spdlog::warn("[MultiWorldAudit] phase={} slot={} actor={:08X} INVALID_STATE READ_ONLY",phase,i,multiRoster[i]);
            continue;
        }
        const auto& anc=s.carrierParent->world;
        const auto& observed=s.npcParent->world;
        if (!Finite(anc.rotate) || !Finite(observed.rotate) ||
            !Finite(s.before.rotate) || !Finite(s.npcRoot->world.rotate)) {
            spdlog::warn("[MultiWorldAudit] phase={} slot={} actor={:08X} NONFINITE_ROTATION READ_ONLY",phase,i,s.actorID);
            continue;
        }
        const auto neutralWorld=Multiply(anc.rotate,s.before.rotate);
        const auto expectedWorld=Multiply(multiWorldTilt,neutralWorld);
        const auto actualDelta=Multiply(observed.rotate,Transpose3(neutralWorld));
        const float deltaErr=RotationError(actualDelta,multiWorldTilt);
        const auto expectedUp=Rotate(expectedWorld,up);
        const auto observedUp=Rotate(observed.rotate,up);
        const auto rootUp=Rotate(s.npcRoot->world.rotate,up);
        const float upError=WorldUpAngleDegrees(expectedUp,observedUp);
        const float rootVsCarrier=WorldUpAngleDegrees(observedUp,rootUp);
        const float actorMove=Distance(s.actor3D->world.translate,s.actorWorldStartPos);
        if (first) { firstExpectedUp=expectedUp; firstObservedUp=observedUp; first=false; }
        maxExpectedUpSpread=std::max(maxExpectedUpSpread,WorldUpAngleDegrees(firstExpectedUp,expectedUp));
        maxObservedUpSpread=std::max(maxObservedUpSpread,WorldUpAngleDegrees(firstObservedUp,observedUp));
        maxTiltError=std::max(maxTiltError,deltaErr);
        maxUpError=std::max(maxUpError,upError);
        maxActorMove=std::max(maxActorMove,actorMove);
        ++valid;
        spdlog::debug("[MultiWorldAudit] phase={} slot={} actor={:08X} tiltDeltaErr={:.5f} worldUpErrDeg={:.3f} neutralUpZ={:.5f} expectedUp=({:.4f},{:.4f},{:.4f}) observedUp=({:.4f},{:.4f},{:.4f}) rootVsCarrierUpDeg={:.2f} actor3DMove={:.2f} rootPos=({:.2f},{:.2f},{:.2f}) READ_ONLY",
            phase,i,s.actorID,deltaErr,upError,Rotate(neutralWorld,up).z,
            expectedUp.x,expectedUp.y,expectedUp.z,
            observedUp.x,observedUp.y,observedUp.z,
            rootVsCarrier,actorMove,
            s.npcRoot->world.translate.x,s.npcRoot->world.translate.y,s.npcRoot->world.translate.z);
    }
    spdlog::debug("[MultiWorldAudit] SUMMARY phase={} valid={}/{} maxTiltDeltaErr={:.5f} maxWorldUpErrDeg={:.3f} expectedUpSpreadDeg={:.3f} observedUpSpreadDeg={:.3f} maxActor3DMove={:.2f} rootGap={:.2f} READ_ONLY",
        phase,valid,multiCount,maxTiltError,maxUpError,maxExpectedUpSpread,maxObservedUpSpread,maxActorMove,MultiMaxRootGap());
}
void AuditMulti(const char* phase) {
    for (std::size_t i=0;i<multiCount;++i) AuditOneLocal(multi[i],phase);
    AuditMultiWorld(phase);
}
bool RestoreMulti(const char* reason) {
    AuditMulti("END_PRE_RESTORE");
    const auto count=multiCount;
    bool ok=true;
    for (std::size_t i=count;i>0;--i) ok=RestoreOneLocal(multi[i-1],reason) && ok;
    ClearMulti();
    spdlog::debug("[MultiRestore] CLEAN actors={} success={} nextSceneReady=true",count,ok);
    return ok;
}
bool CommitMultiLocal(std::uint32_t tid,std::uint64_t epoch) {
    if (multiCount<3 || multiCount>kMaxActors || pair[0].valid || pair[1].valid) return false;
    if (graphAuditOnly.load()) {
        auditOnlyActive.store(true);
        spdlog::debug("[MultiCommit] READ_ONLY actors={} graphWrites=0",multiCount);
        return true;
    }
    if (!autoTerrainEnabled.load()) return false;
    std::array<Candidate,kMaxActors> candidates{};
    std::array<LocalPlan,kMaxActors> plans{};
    for (std::size_t i=0;i<multiCount;++i) {
        candidates[i]=Inspect(multiRoster[i]);
        if (!candidates[i].valid) {
            spdlog::warn("[MultiCommit] INVALID_CARRIER slot={} actor={:08X}",i,multiRoster[i]);
            return false;
        }
        for (std::size_t j=0;j<i;++j)
            if (candidates[i].outer==candidates[j].outer || candidates[i].root==candidates[j].root) return false;
    }
    auto* player=RE::TESForm::LookupByID<RE::Actor>(kPlayerID);

    if (player && player->GetParentCell() && player->GetParentCell()->IsInteriorCell()) {
        auto* cell=player->GetParentCell();
        for (std::size_t i=0;i<multiCount;++i) {
            auto* participant=RE::TESForm::LookupByID<RE::Actor>(multiRoster[i]);
            if (!participant || participant->GetParentCell()!=cell) {
                spdlog::warn("[IndoorTilt] REJECT_DIFFERENT_CELL actor={:08X} cell={:08X} playerCell={:08X} NO_GRAPH_WRITES",
                    multiRoster[i],participant && participant->GetParentCell()?participant->GetParentCell()->GetFormID():0u,
                    cell->GetFormID());
                return false;
            }
        }
    }
    const auto slope=SampleOutdoorLand(player,candidates[0].parent->world.rotate,
        multiMaxSlopeDegrees.load(),static_cast<std::uint32_t>(multiCount));
    if (!slope.valid) { spdlog::debug("[MultiCommit] SKIP_NO_LAND NO_GRAPH_WRITES"); return false; }
    const auto pivot=candidates[0].outer->world.translate;
    for (std::size_t i=0;i<multiCount;++i) {
        if (Distance(candidates[0].outer->world.translate,candidates[i].outer->world.translate)>kMaxPairDistance) return false;
        plans[i]=PlanNPCParent(candidates[i],slope.worldRotation,pivot);
        if (!plans[i].valid) {
            spdlog::warn("[MultiCommit] PLAN_REJECT slot={} actor={:08X}",i,multiRoster[i]);
            return false;
        }
    }
    if (!WithinStartDeadline(tid,epoch,"multi-commit") || eventEpoch.load()!=epoch) return false;
    if (player && player->GetParentCell() && player->GetParentCell()->IsInteriorCell()) {
        spdlog::debug("[IndoorTilt] COMMIT_ATTEMPT thread={} epoch={} cell={:08X} actors={} originalMultiTransaction=true",
            tid,epoch,player->GetParentCell()->GetFormID(),multiCount);
    }
    spdlog::debug("[MultiCommit] PREPARED thread={} epoch={} actors={} pitch={:.2f} roll={:.2f} pivot=({:.2f},{:.2f},{:.2f})",
        tid,epoch,multiCount,slope.pitch,slope.roll,pivot.x,pivot.y,pivot.z);
    for (std::size_t i=0;i<multiCount;++i) {
        if (!ApplyOneLocal(candidates[i],plans[i],multi[i])) {
            spdlog::error("[MultiCommit] ROLLBACK failedSlot={} actorsApplied={}",i,i);
            for (std::size_t j=i;j>0;--j) RestoreOneLocal(multi[j-1],"multi-rollback");
            ClearMulti();
            return false;
        }
    }
    bool geometryOK=true;
    for (std::size_t i=0;i<multiCount;++i) {
        const auto rootErr=Distance(multi[i].npcRoot->world.translate,plans[i].expectedRootWorld);
        const auto rotErr=RotationError(multi[i].npcRoot->world.rotate,plans[i].expectedRootRotation);
        spdlog::debug("[MultiCommit] GEOMETRY slot={} ref={:08X} rootErr={:.4f} rotationErr={:.5f}",
            i,multi[i].actorID,rootErr,rotErr);
        if (rootErr>0.5f || rotErr>0.025f) geometryOK=false;
    }
    if (!geometryOK) {
        for (std::size_t j=multiCount;j>0;--j) RestoreOneLocal(multi[j-1],"multi-geometry-reject");
        ClearMulti();
        spdlog::warn("[MultiCommit] GEOMETRY_REJECT rolledBack=true");
        return false;
    }
    multiBaselineMaxGap=MultiMaxRootGap();
    multiWorldTilt=slope.worldRotation;
    multiPivot=pivot;
    multiRecoveries=0;
    multiFailClosed=false;
    pairApplied.store(true);
    multiActiveCount.store(static_cast<std::uint32_t>(multiCount));
    appliedPitch.store(slope.pitch);
    appliedRoll.store(slope.roll);
    spdlog::debug("[MultiCommit] APPLIED actors={} maxRootGap={:.3f} topologyWrites=0 followWrites=0",multiCount,multiBaselineMaxGap);
    AuditMulti("POST_APPLY");
    return true;
}

bool CommitSoloLocal(std::uint32_t tid,std::uint64_t epoch) {
    soloWorldVerified=false;
    if (solo.valid || pair[0].valid || pair[1].valid || MultiHasState()) return false;
    const auto c=Inspect(kPlayerID);
    if (!c.valid) { spdlog::warn("[SoloStage] INVALID_PLAYER_CARRIER"); return false; }
    if (graphAuditOnly.load()) {
        auditOnlyActive.store(true);
        spdlog::debug("[SoloStage] READ_ONLY thread={} epoch={} noGraphWrites=true",tid,epoch);
        return true;
    }
    if (!autoTerrainEnabled.load()) return false;
    auto* player=RE::TESForm::LookupByID<RE::Actor>(kPlayerID);
    const auto slope=SampleOutdoorLand(player,c.parent->world.rotate,30.f,1);
    if (!slope.valid) {
        spdlog::debug("[SoloStage] SKIP_UNSAFE_TERRAIN thread={} epoch={} NO_GRAPH_WRITES",tid,epoch);
        return false;
    }
    const auto pivot=c.outer->world.translate;
    const auto plan=PlanNPCParent(c,slope.worldRotation,pivot);
    if (!plan.valid) { spdlog::warn("[SoloStage] INVALID_PLAN noGraphWrites=true"); return false; }
    if (!WithinStartDeadline(tid,epoch,"solo-commit") || eventEpoch.load()!=epoch) return false;
    if (player && player->GetParentCell() && player->GetParentCell()->IsInteriorCell())
        spdlog::debug("[IndoorTilt] COMMIT_ATTEMPT thread={} epoch={} cell={:08X} actorCount=1 soloTransaction=true",
            tid,epoch,player->GetParentCell()->GetFormID());
    if (!ApplyOneLocal(c,plan,solo)) return false;
    const float posErr=Distance(solo.npcRoot->world.translate,plan.expectedRootWorld);
    const float rotErr=RotationError(solo.npcRoot->world.rotate,plan.expectedRootRotation);
    if (!std::isfinite(posErr) || !std::isfinite(rotErr) || posErr>0.5f || rotErr>0.025f) {
        spdlog::warn("[SoloStage] GEOMETRY_REJECT posErr={:.4f} rotErr={:.5f} rollingBack=true",posErr,rotErr);
        RestoreOneLocal(solo,"solo-geometry-rollback");
        ClearSolo();
        return false;
    }
    soloWorldTilt=slope.worldRotation;
    soloPivot=pivot;
    soloRecoveries=0;
    soloFailClosed=false;
    soloWorldVerified=true;
    soloApplied.store(true);
    pairApplied.store(true);
    appliedPitch.store(slope.pitch);
    appliedRoll.store(slope.roll);
    spdlog::debug("[SoloStage] APPLIED thread={} epoch={} actor={:08X} pitch={:.2f} roll={:.2f} posErr={:.4f} rotErr={:.5f} noReparent=true",
        tid,epoch,kPlayerID,slope.pitch,slope.roll,posErr,rotErr);
    AuditOneLocal(solo,"SOLO_POST_APPLY");
    return true;
}
bool RestoreSoloLocal(const char* reason) {
    AuditOneLocal(solo,"SOLO_END_PRE_RESTORE");
    const bool ok=RestoreOneLocal(solo,reason);
    ClearSolo();
    spdlog::debug("[SoloStage] END_CLEAN success={} nextSceneReady=true",ok);
    return ok;
}
bool CommitBothLocal(std::uint32_t thread,std::uint64_t epoch) {
    nstPairWorldVerified=false;
    if (pair[0].valid || pair[1].valid) {
        spdlog::critical("[LocalCommit] BLOCKED previous local state still captured");
        return false;
    }
    const auto bid=actorBID.load();
    if (!bid || bid==kPlayerID) return false;
    const auto a=Inspect(kPlayerID);
    const auto b=Inspect(bid);
    if (!a.valid || !b.valid || a.outer==b.outer || a.root==b.root) {
        spdlog::warn("[LocalCommit] INVALID_ACTORS A={} B={} B={:08X}",a.valid,b.valid,bid);
        return false;
    }
    const auto distance=Distance(a.outer->world.translate,b.outer->world.translate);
    if (!std::isfinite(distance) || distance>kMaxPairDistance) return false;
    if (graphAuditOnly.load()) {
        auditOnlyActive.store(true);
        spdlog::debug("[LocalControl] START thread={} epoch={} A={:08X} B={:08X} NO_GRAPH_WRITES",thread,epoch,kPlayerID,bid);
        return true;
    }
    if (!autoTerrainEnabled.load()) {
        spdlog::debug("[LocalCommit] SKIP_AUTO_OFF no 3D writes thread={} epoch={}",thread,epoch);
        return false;
    }
    auto* player=RE::TESForm::LookupByID<RE::Actor>(kPlayerID);

    if (player && player->GetParentCell() && player->GetParentCell()->IsInteriorCell()) {
        auto* partner=RE::TESForm::LookupByID<RE::Actor>(bid);
        if (!partner || partner->GetParentCell()!=player->GetParentCell()) {
            spdlog::warn("[IndoorTilt] REJECT_DIFFERENT_CELL player={:08X} partner={:08X} NO_GRAPH_WRITES",
                player->GetParentCell()->GetFormID(),partner && partner->GetParentCell()?partner->GetParentCell()->GetFormID():0u);
            return false;
        }
    }
    const auto slope=SampleOutdoorLand(player,a.parent->world.rotate,30.f,2);
    if (!slope.valid) {
        spdlog::debug("[LocalCommit] SKIP_NO_LAND thread={} epoch={} NO_GRAPH_WRITES",thread,epoch);
        return false;
    }
    const auto pivot=a.outer->world.translate;
    const auto pa=PlanNPCParent(a,slope.worldRotation,pivot);
    const auto pb=PlanNPCParent(b,slope.worldRotation,pivot);
    if (!pa.valid || !pb.valid) {
        spdlog::warn("[LocalCommit] SKIP_INVALID_PLAN A={} B={} moveA={:.3f} moveB={:.3f}",pa.valid,pb.valid,pa.move,pb.move);
        return false;
    }
    if (player && player->GetParentCell() && player->GetParentCell()->IsInteriorCell()) {
        spdlog::debug("[IndoorTilt] COMMIT_ATTEMPT thread={} epoch={} cell={:08X} actorCount=2 originalPairTransaction=true",
            thread,epoch,player->GetParentCell()->GetFormID());
    }
    Timeline("PREPARED",thread,epoch);
    if (eventEpoch.load()!=epoch || !WithinStartDeadline(thread,epoch,"pre-local-commit")) return false;
    spdlog::debug("[LocalCommit] PREPARED carrier=NPC thread={} epoch={} pivot=({:.2f},{:.2f},{:.2f}) pitch={:.2f} roll={:.2f} gapBefore={:.4f} gapPredicted={:.4f}",
        thread,epoch,pivot.x,pivot.y,pivot.z,slope.pitch,slope.roll,
        Distance(pa.neutralRootWorld,pb.neutralRootWorld),Distance(pa.expectedRootWorld,pb.expectedRootWorld));
    if (!ApplyOneLocal(a,pa,pair[0])) return false;
    if (!ApplyOneLocal(b,pb,pair[1])) {
        spdlog::error("[LocalCommit] B_FAILED rollback A");
        RestoreOneLocal(pair[0],"rollback-B");
        ClearPair();
        return false;
    }
    pairApplied.store(true);
    syncBaselineRootGap=Distance(pair[0].npcRoot->world.translate,pair[1].npcRoot->world.translate);
    syncSharedWorldRotation=slope.worldRotation;
    syncSharedPivot=pivot;
    syncRecoveryCount=0;
    syncFailClosed=false;
    appliedPitch.store(slope.pitch);
    appliedRoll.store(slope.roll);
    const float errA=Distance(pair[0].npcRoot->world.translate,pa.expectedRootWorld);
    const float errB=Distance(pair[1].npcRoot->world.translate,pb.expectedRootWorld);
    const float rotA=RotationError(pair[0].npcRoot->world.rotate,pa.expectedRootRotation);
    const float rotB=RotationError(pair[1].npcRoot->world.rotate,pb.expectedRootRotation);
    spdlog::debug("[LocalCommit] ONE_SHOT_APPLY thread={} epoch={} errA={:.4f} errB={:.4f} rotA={:.5f} rotB={:.5f} rootGapAfter={:.4f} parentLinksWritten=0 followWrites=0",
        thread,epoch,errA,errB,rotA,rotB,Distance(pair[0].npcRoot->world.translate,pair[1].npcRoot->world.translate));
    nstPairWorldVerified=errA<=0.5f && errB<=0.5f && rotA<=0.02f && rotB<=0.02f;
    if (!nstPairWorldVerified)
        spdlog::warn("[NSTInterop] NOT_PUBLISHED pair geometry mismatch");
    if (errA>0.5f || errB>0.5f || rotA>0.02f || rotB>0.02f)
        spdlog::warn("[LocalCommit] WORLD_MISMATCH_INSTANT thread={} errA={:.4f} errB={:.4f} rotA={:.5f} rotB={:.5f} (local may not drive skeleton world; no retry)",
            thread,errA,errB,rotA,rotB);
    AuditLocalPair("POST_APPLY");
    Timeline("COMMITTED",thread,epoch);

    return true;
}

struct PoseSample {
    RE::NiPoint3 parentA{}, parentB{}, rootA{}, rootB{};
    RE::NiMatrix3 rotateA{}, rotateB{};
    std::int64_t atMs{0};
    float rootGap{0.f};
    bool valid{false};
};
struct PoseProbeState {
    std::uint64_t epoch{0};
    std::uint32_t thread{0}, partner{0};
    std::uint32_t attempts{0}, stableSamples{0};
    PoseSample qualified{};
    PoseSample lastObserved{};
    bool everAligned{false};
    bool committed{false};
    RE::NiMatrix3 commitParentA{}, commitParentB{};
    std::uint64_t initialNodeSerial{0};
};
struct SoloPoseSample {
    RE::NiPoint3 parent{}, root{};
    RE::NiMatrix3 rotation{};
    std::int64_t atMs{};
    bool valid{};
};
struct SoloPoseProbeState {
    std::uint64_t epoch{}, lastNodeSerial{};
    std::uint32_t thread{}, attempts{}, stable{};
    SoloPoseSample qualified{},last{};
    bool committed{};
};
SoloPoseProbeState soloProbe{};
void ProbeSoloPoseOnGameThread(std::uint32_t tid,std::uint64_t epoch);
PoseProbeState poseProbe{};
constexpr std::uint32_t kMaxPoseTaskAttempts = 128;
void ProbePoseOnGameThread(std::uint32_t tid, std::uint64_t epoch);

TMSPoseWakeTimer& PoseWakeTimer() {
    static TMSPoseWakeTimer timer([](std::uint32_t tid, std::uint64_t epoch) {
        EnqueueGameTask([tid, epoch] {
            if (soloProbe.epoch==epoch && soloProbe.thread==tid && pendingStartEpoch.load()==epoch && !soloProbe.committed)
                ProbeSoloPoseOnGameThread(tid,epoch);
            else if (multiProbe.epoch==epoch && multiProbe.thread==tid && multiCount>=3) ProbeMultiPoseOnGameThread(tid,epoch);
            else ProbePoseOnGameThread(tid,epoch);
        });
    });
    return timer;
}
TMSPoseWakeTimer& LocalAuditWakeTimer() {

    static TMSPoseWakeTimer timer([](std::uint32_t tid, std::uint64_t epoch) {
        EnqueueGameTask([tid,epoch] {
            if (activeSession.load() && pairApplied.load() &&
                activeThread.load()==tid && activeStartEpoch.load()==epoch) {
                if (soloApplied.load()) AuditOneLocal(solo,"SOLO_DELAYED_ONCE");
                else if (multiActiveCount.load()) AuditMulti("DELAYED_ONCE");
                else AuditLocalPair("DELAYED_ONCE");
            }
        });
    });
    return timer;
}
TMSPoseWakeTimer& SyncWakeTimer() {

    static TMSPoseWakeTimer timer([](std::uint32_t tid, std::uint64_t epoch) {
        EnqueueGameTask([tid,epoch] {
            if (soloApplied.load()) SyncSoloOnGameThread(tid,epoch);
            else if (multiActiveCount.load()) SyncMultiOnGameThread(tid,epoch);
            else SyncPairOnGameThread(tid,epoch);
        });
    });
    return timer;
}
void SchedulePoseProbe(std::uint32_t tid, std::uint64_t epoch) {
    const auto delay = std::chrono::milliseconds(poseSampleSpacingMs.load());
    PoseWakeTimer().Arm(tid, epoch, delay);
}

PoseSample ReadUnmodifiedPose(std::uint32_t partner) {
    PoseSample s{};
    s.atMs=NowMs();
    const auto a=Inspect(kPlayerID,false);
    const auto b=Inspect(partner,false);
    if (!a.valid || !b.valid || !a.parent || !b.parent) return s;
    const auto& aw=a.parent->world;
    const auto& bw=b.parent->world;
    const auto& al=a.root->local;
    const auto& bl=b.root->local;
    if (!Finite(aw.translate) || !Finite(bw.translate) || !Finite(aw.rotate) ||
        !Finite(bw.rotate) || !Finite(al.translate) || !Finite(bl.translate) ||
        !std::isfinite(aw.scale) || !std::isfinite(bw.scale) ||
        std::fabs(aw.scale)<0.01f || std::fabs(bw.scale)<0.01f) return s;
    s.parentA=aw.translate;
    s.parentB=bw.translate;
    s.rotateA=aw.rotate;
    s.rotateB=bw.rotate;

    s.rootA=Add(aw.translate,Rotate(aw.rotate,Mul(al.translate,aw.scale)));
    s.rootB=Add(bw.translate,Rotate(bw.rotate,Mul(bl.translate,bw.scale)));
    s.rootGap=Distance(s.rootA,s.rootB);
    s.valid=Finite(s.rootA) && Finite(s.rootB) && std::isfinite(s.rootGap);
    return s;
}
float ParentRotateDelta(const PoseSample& a, const PoseSample& b) {
    return std::max(RotationError(a.rotateA,b.rotateA),RotationError(a.rotateB,b.rotateB));
}
bool SimilarPose(const PoseSample& a, const PoseSample& b) {
    return a.valid && b.valid &&
        Distance(a.parentA,b.parentA)<=1.5f && Distance(a.parentB,b.parentB)<=1.5f &&
        Distance(a.rootA,b.rootA)<=2.f && Distance(a.rootB,b.rootB)<=2.f &&
        ParentRotateDelta(a,b)<=0.012f;
}
bool PoseTokenLive(std::uint32_t thread,std::uint64_t epoch) {
    if (!epoch || fatalGraphFault.load() || eventEpoch.load()!=epoch ||
        pendingStartEpoch.load()!=epoch || pendingThread.load()!=thread ||
        activeSession.load() || solo.valid || pair[0].valid || pair[1].valid || MultiHasState()) return false;
    std::scoped_lock lock(callbackMutex);
    return ostimSignalActive && ostimSignalEpoch==epoch && ostimSignalThread==thread;
}
void ProbePoseOnGameThread(std::uint32_t tid,std::uint64_t epoch) {
    if (!PoseTokenLive(tid,epoch) || poseProbe.epoch!=epoch || poseProbe.thread!=tid) return;
    const auto age=SignalAgeMs();
    if (age<0 || age>maxStartAgeMs.load() || poseProbe.attempts>=kMaxPoseTaskAttempts) {
        const char* reason=poseProbe.attempts>=kMaxPoseTaskAttempts ? "TASK_BUDGET" : "DEADLINE";
        spdlog::warn("[PoseGate] SKIP_UNSETTLED thread={} epoch={} reason={} elapsedMs={} tries={} stable={} gap={:.3f} nodeDelta={} NO_GRAPH_WRITES",
            tid,epoch,reason,age,poseProbe.attempts,poseProbe.stableSamples,
            poseProbe.lastObserved.valid?poseProbe.lastObserved.rootGap:-1.f,
            nativeNodeSerial.load()-poseProbe.initialNodeSerial);
        PoseWakeTimer().Cancel();
        pendingStartEpoch.store(0);
        actorBID.store(0);

        return;
    }
    ++poseProbe.attempts;
    const auto previous=poseProbe.lastObserved;
    const auto current=ReadUnmodifiedPose(poseProbe.partner);
    spdlog::debug("[PoseGate] SAMPLE thread={} epoch={} elapsedMs={} tries={} deltaMs={} valid={} gap={:.3f} aligned={} stable={}",
        tid,epoch,age,poseProbe.attempts,
        previous.valid ? current.atMs-previous.atMs : -1,
        current.valid,current.valid ? current.rootGap : -1.f,
        current.valid && current.rootGap<=maxUnsettledRootGap.load(),poseProbe.stableSamples);
    if (poseProbe.attempts==1) {
        spdlog::debug("[PoseGate] PRE_ALIGN thread={} epoch={} elapsedMs={} valid={} neutralGap={:.3f} maxGap={:.1f} nodeSerial={}",
            tid,epoch,age,current.valid,current.valid?current.rootGap:-1.f,
            maxUnsettledRootGap.load(),nativeNodeSerial.load());
        Timeline("PRE_ALIGN",tid,epoch);
    }
    if (current.valid && poseProbe.lastObserved.valid) {
        const auto rd=ParentRotateDelta(current,poseProbe.lastObserved);
        const auto positionDelta=std::max(Distance(current.rootA,poseProbe.lastObserved.rootA),
                                           Distance(current.rootB,poseProbe.lastObserved.rootB));
        if (rd>0.012f || positionDelta>2.f) {
            spdlog::debug("[PoseGate] PARENT_CHANGED thread={} epoch={} elapsedMs={} parentRotDelta={:.5f} rootMove={:.3f} gap={:.3f}",
                tid,epoch,age,rd,positionDelta,current.rootGap);
        }
    }
    if (current.valid) poseProbe.lastObserved=current;
    const bool aligned=current.valid && current.rootGap<=maxUnsettledRootGap.load();
    if (aligned && !poseProbe.everAligned) {
        poseProbe.everAligned=true;
        spdlog::debug("[PoseGate] POST_ALIGN thread={} epoch={} elapsedMs={} gap={:.3f} nodeDelta={}",
            tid,epoch,age,current.rootGap,nativeNodeSerial.load()-poseProbe.initialNodeSerial);
        Timeline("POST_ALIGN",tid,epoch);
    }
    if (!aligned) {
        poseProbe.stableSamples=0;
        poseProbe.qualified={};
    } else if (!poseProbe.qualified.valid) {
        poseProbe.qualified=current;
        poseProbe.stableSamples=1;
    } else if (current.atMs-poseProbe.qualified.atMs>=poseSampleSpacingMs.load()) {
        poseProbe.stableSamples=SimilarPose(current,poseProbe.qualified) ? poseProbe.stableSamples+1 : 1;
        poseProbe.qualified=current;
    }
    if (aligned && poseProbe.stableSamples>=2 && age>=minPoseAgeMs.load()) {
        spdlog::debug("[PoseGate] POSE_STABLE thread={} epoch={} elapsedMs={} attempts={} samples={} neutralGap={:.3f} nodeDelta={}",
            tid,epoch,age,poseProbe.attempts,poseProbe.stableSamples,current.rootGap,
            nativeNodeSerial.load()-poseProbe.initialNodeSerial);
        Timeline("POSE_STABLE",tid,epoch);

        PoseWakeTimer().Cancel();
        if (PoseTokenLive(tid,epoch) && WithinStartDeadline(tid,epoch,"pose-stable-commit") && CommitBothLocal(tid,epoch)) {
            pendingStartEpoch.store(0);
            activeStartEpoch.store(epoch);
            activeThread.store(tid);
            activeFramework.store(Framework::kOStim);
            if (auditOnlyActive.load()) {
                poseProbe.committed=false;
                activeSession.store(true);
                spdlog::debug("[SceneStart] CONTROL_ONLY framework=OStim thread={} epoch={} graphWrites=NONE",tid,epoch);
            } else {
                poseProbe.commitParentA=pair[0].npcParent->world.rotate;
                poseProbe.commitParentB=pair[1].npcParent->world.rotate;
                poseProbe.committed=true;
                activeSession.store(true);
                PublishNSTInterop(tid,epoch);
                spdlog::info("[Stage] APPLIED thread={} actors=2 pitch={:.2f} roll={:.2f}",
                    tid,appliedPitch.load(),appliedRoll.load());
                if (localAuditEnabled.load()) LocalAuditWakeTimer().Arm(tid,epoch,std::chrono::milliseconds(localAuditDelayMs.load()));
            }
        } else {
            pendingStartEpoch.store(0);
            actorBID.store(0);
            spdlog::warn("[PoseGate] COMMIT_ABORT thread={} epoch={} (no late corrective writes)",tid,epoch);
        }
        return;
    }

    SchedulePoseProbe(tid,epoch);
}
SoloPoseSample ReadSoloPose() {
    SoloPoseSample s{};
    s.atMs=NowMs();
    const auto c=Inspect(kPlayerID,false);
    if (!c.valid || !c.parent || !c.root) return s;
    const auto& w=c.parent->world;
    const auto& r=c.root->local;
    if (!Finite(w.translate) || !Finite(w.rotate) || !Finite(r.translate) ||
        !Finite(r.rotate) || !std::isfinite(w.scale) || std::fabs(w.scale)<0.01f) return s;
    s.parent=w.translate;
    s.rotation=w.rotate;
    s.root=Add(w.translate,Rotate(w.rotate,Mul(r.translate,w.scale)));
    s.valid=Finite(s.root);
    return s;
}
bool SimilarSoloPose(const SoloPoseSample& a,const SoloPoseSample& b) {
    return a.valid && b.valid &&
        Distance(a.parent,b.parent)<=1.5f && Distance(a.root,b.root)<=2.f &&
        RotationError(a.rotation,b.rotation)<=0.012f;
}
void ProbeSoloPoseOnGameThread(std::uint32_t tid,std::uint64_t epoch) {
    if (!PoseTokenLive(tid,epoch) || soloProbe.epoch!=epoch || soloProbe.thread!=tid) return;
    const auto age=SignalAgeMs();
    if (age<0 || age>maxStartAgeMs.load() || soloProbe.attempts>=kMaxPoseTaskAttempts) {
        spdlog::warn("[SoloPoseGate] SKIP_UNSETTLED thread={} epoch={} ageMs={} attempts={} stable={} NO_GRAPH_WRITES",
            tid,epoch,age,soloProbe.attempts,soloProbe.stable);
        PoseWakeTimer().Cancel();
        pendingStartEpoch.store(0);
        soloProbe={};
        return;
    }
    ++soloProbe.attempts;
    const auto sample=ReadSoloPose();
    const auto serial=nativeNodeSerial.load();

    if (serial!=soloProbe.lastNodeSerial) {
        soloProbe.stable=0;
        soloProbe.qualified={};
        soloProbe.lastNodeSerial=serial;
    }
    if (!sample.valid) {
        soloProbe.stable=0;
        soloProbe.qualified={};
    } else if (!soloProbe.qualified.valid) {
        soloProbe.qualified=sample;
        soloProbe.stable=1;
    } else if (sample.atMs-soloProbe.qualified.atMs>=poseSampleSpacingMs.load()) {
        soloProbe.stable=SimilarSoloPose(sample,soloProbe.qualified)?soloProbe.stable+1:1;
        soloProbe.qualified=sample;
    }
    soloProbe.last=sample;
    spdlog::debug("[SoloPoseGate] SAMPLE thread={} epoch={} ageMs={} valid={} stable={} nodeSerial={}",
        tid,epoch,age,sample.valid,soloProbe.stable,serial);
    const auto minAge=std::max<std::int64_t>(64,minPoseAgeMs.load());
    if (soloProbe.stable>=2 && age>=minAge) {
        spdlog::debug("[SoloPoseGate] STABLE thread={} epoch={} ageMs={} samples={} nodeSerial={}",
            tid,epoch,age,soloProbe.stable,serial);
        PoseWakeTimer().Cancel();
        if (PoseTokenLive(tid,epoch) && WithinStartDeadline(tid,epoch,"solo-stable-commit") && CommitSoloLocal(tid,epoch)) {
            pendingStartEpoch.store(0);
            activeStartEpoch.store(epoch);
            activeThread.store(tid);
            activeFramework.store(Framework::kOStim);
            soloProbe.committed=!auditOnlyActive.load();
            activeSession.store(true);
            if (soloProbe.committed) {
                PublishNSTInterop(tid,epoch);
                if (localAuditEnabled.load())
                    LocalAuditWakeTimer().Arm(tid,epoch,std::chrono::milliseconds(localAuditDelayMs.load()));
            }
            if (soloProbe.committed)
                spdlog::info("[Stage] APPLIED thread={} actors=1 pitch={:.2f} roll={:.2f}",
                    tid,appliedPitch.load(),appliedRoll.load());
            else
                spdlog::debug("[SceneStart] SOLO_CONTROL_ONLY thread={} epoch={}",tid,epoch);
        } else {
            pendingStartEpoch.store(0);
            soloProbe={};
            spdlog::warn("[SoloPoseGate] COMMIT_ABORT thread={} epoch={} NO_GRAPH_WRITES_REMAINING",tid,epoch);
        }
        return;
    }
    SchedulePoseProbe(tid,epoch);
}
void BeginSoloPoseProbe(std::uint32_t tid,std::uint64_t epoch) {
    if (!PoseTokenLive(tid,epoch)) return;
    soloProbe={};
    soloProbe.thread=tid;
    soloProbe.epoch=epoch;
    soloProbe.lastNodeSerial=nativeNodeSerial.load();
    spdlog::debug("[SoloPoseGate] BEGIN thread={} epoch={} minAgeMs={} spacingMs={} deadlineMs={} noWritesBeforeStable=true",
        tid,epoch,std::max<std::int64_t>(64,minPoseAgeMs.load()),poseSampleSpacingMs.load(),maxStartAgeMs.load());
    ProbeSoloPoseOnGameThread(tid,epoch);
}
void BeginPoseProbe(std::uint32_t tid, std::uint64_t epoch, std::uint32_t partner) {
    if (!PoseTokenLive(tid,epoch)) return;
    poseProbe={};
    poseProbe.epoch=epoch;
    poseProbe.thread=tid;
    poseProbe.partner=partner;
    poseProbe.initialNodeSerial=nativeNodeSerial.load();
    spdlog::debug("[PoseGate] BEGIN thread={} epoch={} minAgeMs={} sampleSpacingMs={} maxGap={:.1f} deadlineMs={} noWritesBeforeStable=true",
        tid,epoch,minPoseAgeMs.load(),poseSampleSpacingMs.load(),maxUnsettledRootGap.load(),maxStartAgeMs.load());
    ProbePoseOnGameThread(tid,epoch);
}

MultiPoseSample ReadMultiPose() {
    MultiPoseSample result{};
    result.atMs=NowMs();
    if (multiCount<3 || multiCount>kMaxActors) return result;
    for (std::size_t i=0;i<multiCount;++i) {
        const auto c=Inspect(multiRoster[i],false);
        if (!c.valid || !c.parent || !Finite(c.parent->world.translate) ||
            !Finite(c.parent->world.rotate) || !Finite(c.root->local.translate) ||
            !std::isfinite(c.parent->world.scale) || std::fabs(c.parent->world.scale)<0.01f) return result;
        result.parent[i]=c.parent->world.translate;
        result.rotation[i]=c.parent->world.rotate;
        result.root[i]=Add(result.parent[i],Rotate(result.rotation[i],Mul(c.root->local.translate,c.parent->world.scale)));
        if (!Finite(result.root[i])) return result;
        for (std::size_t j=0;j<i;++j)
            result.maxGap=std::max(result.maxGap,Distance(result.root[i],result.root[j]));
    }
    result.valid=std::isfinite(result.maxGap);
    return result;
}
bool SimilarMultiPose(const MultiPoseSample& a,const MultiPoseSample& b) {
    if (!a.valid || !b.valid) return false;
    for (std::size_t i=0;i<multiCount;++i) {
        if (Distance(a.parent[i],b.parent[i])>1.5f ||
            Distance(a.root[i],b.root[i])>2.f ||
            RotationError(a.rotation[i],b.rotation[i])>0.012f) return false;
    }
    return true;
}
void PublishNSTInterop(std::uint32_t tid, std::uint64_t epoch) {

    if (!activeSession.load() || activeFramework.load()!=Framework::kOStim ||
        activeThread.load()!=tid || activeStartEpoch.load()!=epoch ||
        eventEpoch.load()!=epoch || auditOnlyActive.load() || graphAuditOnly.load() ||
        !pairApplied.load() || syncFailClosed || multiFailClosed) return;
    std::array<std::uint32_t,kMaxActors> roster{};
    std::size_t count=0;
    if (soloApplied.load()) {
        if (!soloProbe.committed || !soloWorldVerified || !solo.valid) return;
        roster[0]=kPlayerID;
        count=1;
    } else if (multiActiveCount.load()>=3) {
        count=multiActiveCount.load();
        if (count>kMaxActors || !multiProbe.committed) return;
        for (std::size_t i=0;i<count;++i) roster[i]=multiRoster[i];
    } else {
        if (!poseProbe.committed || !actorBID.load() || !nstPairWorldVerified) return;
        roster[0]=kPlayerID;
        roster[1]=actorBID.load();
        count=2;
    }
    if (nstInteropState.Publish(tid,epoch,roster.data(),count)) {

        if (eventEpoch.load()!=epoch) { RevokeNSTInterop("STOP_RACE"); return; }
        spdlog::debug("[NSTInterop] ACTIVE thread={} epoch={} actors={} committed=true",tid,epoch,count);
    }
}
bool MultiProbeTokenLive(std::uint32_t tid,std::uint64_t epoch) {
    if (!epoch || eventEpoch.load()!=epoch || pendingStartEpoch.load()!=epoch ||
        pendingThread.load()!=tid || activeSession.load() || fatalGraphFault.load() ||
        multiCount<3 || multiCount>kMaxActors || pair[0].valid || pair[1].valid) return false;
    std::scoped_lock lock(callbackMutex);
    return ostimSignalActive && ostimSignalThread==tid && ostimSignalEpoch==epoch;
}
void ProbeMultiPoseOnGameThread(std::uint32_t tid,std::uint64_t epoch) {
    if (!MultiProbeTokenLive(tid,epoch) || multiProbe.epoch!=epoch || multiProbe.thread!=tid) return;
    const auto age=SignalAgeMs();
    if (age<0 || age>maxStartAgeMs.load() || multiProbe.attempts>=kMaxPoseTaskAttempts) {
        spdlog::warn("[MultiPoseGate] SKIP_UNSETTLED thread={} epoch={} ageMs={} attempts={} stable={} maxGap={:.2f}",
            tid,epoch,age,multiProbe.attempts,multiProbe.stable,
            multiProbe.last.valid?multiProbe.last.maxGap:-1.f);
        PoseWakeTimer().Cancel();
        pendingStartEpoch.store(0);
        ClearMulti();
        return;
    }
    ++multiProbe.attempts;
    const auto current=ReadMultiPose();
    const bool aligned=current.valid && current.maxGap<=128.f;
    spdlog::debug("[MultiPoseGate] SAMPLE thread={} epoch={} ageMs={} tries={} actors={} valid={} maxGap={:.3f} aligned={} stable={}",
        tid,epoch,age,multiProbe.attempts,multiCount,current.valid,current.valid?current.maxGap:-1.f,
        aligned,multiProbe.stable);
    if (aligned && !multiProbe.everAligned) {
        multiProbe.everAligned=true;
        spdlog::debug("[MultiPoseGate] POST_ALIGN thread={} epoch={} maxGap={:.3f}",tid,epoch,current.maxGap);
    }
    if (!aligned) {
        multiProbe.stable=0;
        multiProbe.qualified={};
    } else if (!multiProbe.qualified.valid) {
        multiProbe.qualified=current;
        multiProbe.stable=1;
    } else if (current.atMs-multiProbe.qualified.atMs>=poseSampleSpacingMs.load()) {
        multiProbe.stable=SimilarMultiPose(current,multiProbe.qualified)?multiProbe.stable+1:1;
        multiProbe.qualified=current;
    }
    multiProbe.last=current;
    if (aligned && multiProbe.stable>=2 && age>=minPoseAgeMs.load()) {
        const auto count=multiCount;
        spdlog::debug("[MultiPoseGate] STABLE thread={} epoch={} ageMs={} actors={} maxGap={:.3f}",
            tid,epoch,age,count,current.maxGap);
        PoseWakeTimer().Cancel();
        if (MultiProbeTokenLive(tid,epoch) && WithinStartDeadline(tid,epoch,"multi-stable-commit") && CommitMultiLocal(tid,epoch)) {
            pendingStartEpoch.store(0);
            activeStartEpoch.store(epoch);
            activeThread.store(tid);
            activeFramework.store(Framework::kOStim);
            activeSession.store(true);
            multiProbe.committed=!auditOnlyActive.load();
            PublishNSTInterop(tid,epoch);
            if (!auditOnlyActive.load())
                spdlog::info("[Stage] APPLIED thread={} actors={} pitch={:.2f} roll={:.2f}",
                    tid,count,appliedPitch.load(),appliedRoll.load());
            else
                spdlog::debug("[SceneStart] MULTI_CONTROL_ONLY thread={} epoch={} actors={}",tid,epoch,count);
            if (localAuditEnabled.load() && !auditOnlyActive.load())
                LocalAuditWakeTimer().Arm(tid,epoch,std::chrono::milliseconds(localAuditDelayMs.load()));
        } else {
            spdlog::warn("[MultiPoseGate] COMMIT_ABORT thread={} epoch={} safeSkip=true",tid,epoch);
            pendingStartEpoch.store(0);
            ClearMulti();
        }
        return;
    }
    SchedulePoseProbe(tid,epoch);
}
void BeginMultiPoseProbe(std::uint32_t tid,std::uint64_t epoch) {
    multiProbe={};
    multiProbe.thread=tid;
    multiProbe.epoch=epoch;
    multiProbe.initialNodeSerial=nativeNodeSerial.load();
    spdlog::debug("[MultiPoseGate] BEGIN thread={} epoch={} actors={} maxGap=128 deadlineMs={} noWritesBeforeStable=true",
        tid,epoch,multiCount,maxStartAgeMs.load());
    ProbeMultiPoseOnGameThread(tid,epoch);
}

void EnqueueGameTask(std::function<void()> fn) {
    if (auto* tasks=SKSE::GetTaskInterface()) tasks->AddTask(std::move(fn));
    else {
        spdlog::critical("[SceneTask] No SKSE task interface");
        fatalGraphFault.store(true);
    }
}
const char* FrameworkName(Framework f) {
    switch (f) {
    case Framework::kOStim: return "OStim";
    case Framework::kSexLab: return "SexLab";
    default: return "None";
    }
}
void AcceptActorsOnGameThread(std::uint32_t, std::uint64_t, std::vector<std::uint32_t>, std::string, bool);
void BeginPoseProbe(std::uint32_t tid, std::uint64_t epoch, std::uint32_t partner);
void BeginMultiPoseProbe(std::uint32_t tid,std::uint64_t epoch);

void StartOnGameThread(Framework framework, std::uint32_t tid, std::uint32_t senderID,
                       std::uint64_t epoch, std::vector<std::uint32_t> nativeRoster = {}) {
    if (eventEpoch.load()!=epoch || fatalGraphFault.load() ||
        activeSession.load() || solo.valid || pair[0].valid || pair[1].valid || MultiHasState()) {
        spdlog::debug("[SceneStart] CANCEL_OR_BUSY framework={} thread={} epoch={}",FrameworkName(framework),tid,epoch);
        return;
    }
    if (framework==Framework::kOStim && !autoTerrainEnabled.load()) {
        spdlog::debug("[SceneStart] SKIP_NST_DISABLED thread={} epoch={} NO_GRAPH_WRITES",tid,epoch);

        return;
    }
    if (framework!=Framework::kOStim) {

        spdlog::debug("[SceneStart] SL_BRIDGE_NOT_IMPLEMENTED; skip without actor list sender={:08X}",senderID);

        return;
    }
    pendingStartEpoch.store(epoch);
    pendingThread.store(tid);
    Timeline("START_TASK",tid,epoch);
    if (!WithinStartDeadline(tid,epoch,"start-task")) { pendingStartEpoch.store(0); return; }
    if (nativeAPIReady.load()) {

        if (nativeRoster.empty()) {
            pendingStartEpoch.store(0);
            spdlog::warn("[NativeAPI] EMPTY_SNAPSHOT thread={} epoch={} no graph writes",tid,epoch);

            return;
        }
        spdlog::debug("[NativeAPI] ACTORS_READY thread={} count={} source=LegacyThreadsSnapshot",tid,nativeRoster.size());
        Timeline("ROSTER_READY",tid,epoch);
        AcceptActorsOnGameThread(tid,epoch,std::move(nativeRoster),"",true);
        return;
    }
    if (!papyrusFallbackEnabled.load()) {
        pendingStartEpoch.store(0);
        spdlog::warn("[NativeAPI] UNAVAILABLE; Papyrus fallback disabled; no actor-local writes");

        return;
    }

    auto* vm=RE::BSScript::Internal::VirtualMachine::GetSingleton();
    if (!vm) {
        pendingStartEpoch.store(0);
        spdlog::error("[Bridge] VM unavailable thread={} epoch={}",tid,epoch);

        return;
    }
    RE::BSTSmartPointer<RE::BSScript::IStackCallbackFunctor> callback;
    auto* args=RE::MakeFunctionArguments(static_cast<std::int32_t>(tid),static_cast<std::int32_t>(epoch));
    const bool dispatched=vm->DispatchStaticCall("TMS_NSTBridge","QueryOStimActors",args,callback);
    spdlog::debug("[Bridge] DISPATCH framework=OStim thread={} epoch={} dispatched={}",tid,epoch,dispatched);
    if (!dispatched) {
        pendingStartEpoch.store(0);

    }
}

void AcceptActorsOnGameThread(std::uint32_t tid,std::uint64_t epoch,std::vector<std::uint32_t> ids,std::string furnitureType,bool fromNative) {
    if (fatalGraphFault.load() || !epoch || eventEpoch.load()!=epoch ||
        pendingStartEpoch.load()!=epoch || pendingThread.load()!=tid) {
        spdlog::debug("[Bridge] STALE actor report thread={} epoch={} current={} pending={}",tid,epoch,eventEpoch.load(),pendingStartEpoch.load());
        return;
    }
    {
        std::scoped_lock lock(callbackMutex);
        if (!ostimSignalActive || ostimSignalThread!=tid || ostimSignalEpoch!=epoch) {
            spdlog::debug("[Bridge] CLOSED event thread={} epoch={}",tid,epoch);
            return;
        }
    }

    if (activeSession.load() || solo.valid || pair[0].valid || pair[1].valid || MultiHasState()) return;
    if (!WithinStartDeadline(tid,epoch,"roster-to-pose-probe")) { pendingStartEpoch.store(0); return; }
    spdlog::debug("[Roster] ACTORS source={} thread={} count={}",fromNative?"LegacyThreads":"Papyrus",tid,ids.size());
    for (std::size_t i=0;i<ids.size();++i) spdlog::debug("[Bridge] actorPosition={} ref={:08X}",i,ids[i]);
    if (ids.empty() || ids.size()>kMaxActors ||
        std::count(ids.begin(),ids.end(),kPlayerID)!=1 ||
        std::any_of(ids.begin(),ids.end(),[](auto id){return id==0;})) {
        spdlog::warn("[Roster] INVALID count={} playerOccurrences={}",ids.size(),std::count(ids.begin(),ids.end(),kPlayerID));
        return;
    }
    std::unordered_set<std::uint32_t> unique(ids.begin(),ids.end());
    if (unique.size()!=ids.size()) { spdlog::warn("[Roster] DUPLICATE_ACTOR"); return; }
    for (auto id:ids) if (!RE::TESForm::LookupByID<RE::Actor>(id)) {
        spdlog::warn("[Roster] ACTOR_LOOKUP_FAILED ref={:08X}",id);
        return;
    }
    if (furnitureRefID.load() && skipBoundFurniture.load()) {
        spdlog::warn("[FurnitureProbe] SKIP_BOUND_SCENE ref={:08X} actors={} noActorOrFurnitureWrites=true",
            furnitureRefID.load(),ids.size());
        pendingStartEpoch.store(0);
        return;
    }
    if (ids.size()==1) {

        if (!fromNative) {
            auto f=furnitureType;
            std::transform(f.begin(),f.end(),f.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
            if (!(f.empty() || f=="none" || f=="ground" || f=="null")) {
                spdlog::info("[SoloStage] SKIP_FURNITURE type='{}'",f);
                pendingStartEpoch.store(0);
                return;
            }
        }
        spdlog::debug("[SceneStart] BEGIN_SOLO thread={} epoch={} actor={:08X} source={} virtualStage=true",
            tid,epoch,kPlayerID,fromNative?"LegacyThreads":"PapyrusOThread");
        BeginSoloPoseProbe(tid,epoch);
        return;
    }
    if (ids.size()>2) {
        if (!multiEnabled.load()) {
            spdlog::debug("[Multi] DISABLED actors={}",ids.size());
            pendingStartEpoch.store(0);
            return;
        }

        multiRoster={}; multiRoster[0]=kPlayerID;
        std::size_t slot=1;
        for (auto id:ids) if (id!=kPlayerID) multiRoster[slot++]=id;
        multiCount=ids.size();
        spdlog::debug("[SceneStart] BEGIN_MULTI thread={} epoch={} actorCount={} source={} playerFirst=true",
            tid,epoch,multiCount,fromNative?"LegacyThreads":"PapyrusOThread");
        BeginMultiPoseProbe(tid,epoch);
        return;
    }
    const auto other=ids[0]==kPlayerID ? ids[1] : ids[0];

    if (!fromNative) {
        auto f=furnitureType;
        std::transform(f.begin(),f.end(),f.begin(),[](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        if (!(f.empty() || f=="none" || f=="ground" || f=="null")) {
            spdlog::info("[Terrain] SKIP furnitureType='{}'",f);
            return;
        }
    }
    actorBID.store(other);
    spdlog::debug("[SceneStart] BEGIN framework=OStim thread={} epoch={} A={:08X} B={:08X} source={}",
        tid,epoch,kPlayerID,other,fromNative?"LegacyThreads":"PapyrusOThread");
    BeginPoseProbe(tid,epoch,other);
}
void ReportActors(RE::StaticFunctionTag*,std::int32_t tid,std::int32_t epoch,std::vector<RE::Actor*> actors,RE::BSFixedString furnitureType) {
    if (tid<0 || epoch<=0) return;
    std::vector<std::uint32_t> ids;
    ids.reserve(actors.size());
    for (auto* actor: actors) ids.push_back(actor ? actor->GetFormID() : 0u);
    const std::string furniture=furnitureType.c_str() ? furnitureType.c_str() : "";
    spdlog::info("[Bridge] REPORT thread={} epoch={} actors={} furniture='{}'",tid,epoch,ids.size(),furniture);
    EnqueueGameTask([thread=static_cast<std::uint32_t>(tid),e=static_cast<std::uint64_t>(epoch),roster=std::move(ids),furniture]() mutable {
        AcceptActorsOnGameThread(thread,e,std::move(roster),furniture,false);
    });
}
void ReportFailure(RE::StaticFunctionTag*,std::int32_t tid,std::int32_t epoch,std::int32_t reason) {
    spdlog::warn("[Bridge] FAILED thread={} epoch={} reason={} (OThread not ready/actors missing)",tid,epoch,reason);
    EnqueueGameTask([thread=static_cast<std::uint32_t>(tid),e=static_cast<std::uint64_t>(epoch)] {
        if (pendingStartEpoch.load()==e && pendingThread.load()==thread) {
            pendingStartEpoch.store(0);

        }
    });
}
bool RegisterPapyrus(RE::BSScript::IVirtualMachine* vm) {
    if (!vm) return false;
    vm->RegisterFunction("ReportActors","TMS_NSTNative",ReportActors);
    vm->RegisterFunction("ReportFailure","TMS_NSTNative",ReportFailure);
    return true;
}
void EndOnGameThread(Framework framework, std::uint32_t tid, std::uint64_t epoch) {
    if (framework==Framework::kOStim && activeThread.load()==tid) RevokeNSTInterop("END_TASK");
    if (framework==Framework::kOStim && furnitureRefID.load() &&
        (activeThread.load()==tid || pendingThread.load()==tid))
        ReadFurnitureOnGameThread(tid,furnitureRefID.load(),"END");

    if (framework==Framework::kOStim && pendingThread.load()==tid) {
        pendingStartEpoch.store(0);
        if (!activeSession.load()) {
            actorBID.store(0);
            soloProbe={};
            ClearMulti();
            furnitureRefID.store(0);
        }
    }
    if (!activeSession.load() || activeFramework.load()!=framework ||
        (framework==Framework::kOStim && tid!=activeThread.load()) ||
        epoch<activeStartEpoch.load()) {
        spdlog::debug("[SceneEnd] IGNORE_OR_PENDING framework={} thread={} active={} epoch={}",
            FrameworkName(framework),tid,activeSession.load(),epoch);
        return;
    }
    Timeline("END_TASK",tid,epoch);
    spdlog::debug("[SceneEnd] BEGIN framework={} thread={} NPC.local restore",FrameworkName(framework),tid);
    LocalAuditWakeTimer().Cancel();
    SyncWakeTimer().Cancel();
    furnitureRefID.store(0);
    const bool wasAuditOnly=auditOnlyActive.load();
    const auto endingActorCount=soloProbe.committed ? 1u :
        (multiActiveCount.load() ? multiActiveCount.load() : 2u);
    bool endOK=false;
    if (wasAuditOnly) {
        if (multiCount) ClearMulti();
        ClearSolo();
        auditOnlyActive.store(false);
        spdlog::debug("[LocalControl] END thread={} epoch={} NO_GRAPH_WRITES",tid,epoch);
        endOK=true;
    } else {
        endOK=soloProbe.committed ? RestoreSoloLocal(FrameworkName(framework)) :
            (multiActiveCount.load() ? RestoreMulti(FrameworkName(framework)) : RestorePairLocal(FrameworkName(framework)));
    }
    if (endOK) {
        activeSession.store(false);
        activeFramework.store(Framework::kNone);
        activeThread.store(0);
        activeStartEpoch.store(0);
        actorBID.store(0);
        soloProbe={};
        if (!wasAuditOnly)
            spdlog::info("[Stage] END thread={} actors={} cleaned=true",tid,endingActorCount);
        else
            spdlog::debug("[SceneEnd] CONTROL_ONLY_CLEAN thread={}",tid);
    }
}

enum class SyncCarrierState : std::uint8_t { kInvalid, kHeld, kNeutral, kBaseReset, kOther };
const char* SyncStateName(SyncCarrierState state) {
    switch (state) {
    case SyncCarrierState::kHeld: return "HELD";
    case SyncCarrierState::kNeutral: return "NEUTRAL";
    case SyncCarrierState::kBaseReset: return "BASE_RESET";
    case SyncCarrierState::kOther: return "OTHER";
    default: return "INVALID";
    }
}
bool NearTransform(const RE::NiTransform& a, const RE::NiTransform& b) {
    return RotationError(a.rotate,b.rotate)<=0.010f &&
           Distance(a.translate,b.translate)<=0.10f &&
           std::fabs(a.scale-b.scale)<=0.001f;
}
bool IsBaseReset(const RE::NiTransform& now,const RE::NiTransform& oldBase) {

    return Finite(now.translate) && Finite(now.rotate) &&
        std::isfinite(now.scale) && std::isfinite(oldBase.scale) &&
        std::fabs(now.scale-oldBase.scale)<=0.001f &&
        RotationError(now.rotate,oldBase.rotate)<=0.010f &&
        Distance(now.translate,oldBase.translate)<=32.f;
}
SyncCarrierState ReadCarrierState(const LocalState& s) {
    if (!s.valid || !CurrentActor3D(s) || !NpcTopologyUnchanged(s))
        return SyncCarrierState::kInvalid;
    const auto& now=s.npcParent->local;
    if (NearTransform(now,s.applied)) return SyncCarrierState::kHeld;
    if (NearTransform(now,s.before)) return SyncCarrierState::kNeutral;
    if (IsBaseReset(now,s.before)) return SyncCarrierState::kBaseReset;
    return SyncCarrierState::kOther;
}
void SoloFailClose(const char* reason) {
    if (soloFailClosed) return;
    soloFailClosed=true;
    RevokeNSTInterop("SOLO_FAIL_CLOSED");
    SyncWakeTimer().Cancel();
    const auto st=ReadCarrierState(solo);
    spdlog::warn("[SoloSync] FAIL_CLOSED reason={} carrierState={} action=RESTORE_HELD_ONLY",
        reason,SyncStateName(st));
    if (st==SyncCarrierState::kHeld) RestoreOneLocal(solo,"solo-sync-fail-closed");
    else solo={};
    soloApplied.store(false);
    pairApplied.store(false);
}
void SyncSoloOnGameThread(std::uint32_t tid,std::uint64_t epoch) {
    if (!syncGuardEnabled.load() || soloFailClosed || !soloApplied.load() ||
        !activeSession.load() || activeFramework.load()!=Framework::kOStim ||
        activeThread.load()!=tid || activeStartEpoch.load()!=epoch ||
        eventEpoch.load()!=epoch || !soloProbe.committed) return;
    const auto st=ReadCarrierState(solo);
    spdlog::debug("[SoloSync] CHECK thread={} epoch={} actor={:08X} state={} recoveries={}/{}",
        tid,epoch,kPlayerID,SyncStateName(st),soloRecoveries,syncMaxRecoveries.load());
    if (st==SyncCarrierState::kHeld) return;
    const bool reset=st==SyncCarrierState::kNeutral || st==SyncCarrierState::kBaseReset;
    if (!reset) { SoloFailClose("CARRIER_UNKNOWN_OR_TOPOLOGY_CHANGED"); return; }
    if (soloRecoveries>=static_cast<std::uint32_t>(syncMaxRecoveries.load())) {
        SoloFailClose("RECOVERY_BUDGET_EXHAUSTED"); return;
    }
    const auto outerDelta=RotationError(solo.actor3D->world.rotate,solo.actorWorldAfter);
    if (!std::isfinite(outerDelta) || outerDelta>0.03f) {
        SoloFailClose("OUTER_WORLD_ORIENTATION_CHANGED"); return;
    }
    const auto c=Inspect(kPlayerID,false);
    if (!c.valid || c.parent!=solo.npcParent.get() || c.root!=solo.npcRoot.get()) {
        SoloFailClose("CARRIER_CHANGED"); return;
    }
    const auto& ancestor=c.parent->parent->world;
    const auto newBase=c.parent->local;
    const auto calcPos=Add(ancestor.translate,Rotate(ancestor.rotate,Mul(newBase.translate,ancestor.scale)));
    const auto calcRot=Multiply(ancestor.rotate,newBase.rotate);
    if (!Finite(calcPos) || Distance(calcPos,c.parent->world.translate)>0.5f ||
        RotationError(calcRot,c.parent->world.rotate)>0.025f) {
        SoloFailClose("STALE_WORLD_PROPAGATION"); return;
    }
    const auto plan=PlanNPCParent(c,soloWorldTilt,soloPivot);
    if (!plan.valid) { SoloFailClose("INVALID_REBASE_PLAN"); return; }
    auto applied=newBase;
    applied.rotate=plan.rotation;
    applied.translate=plan.translation;
    c.parent->local.rotate=applied.rotate;
    c.parent->local.translate=applied.translate;
    RefreshOnce(c.parent);
    const float posErr=Distance(c.root->world.translate,plan.expectedRootWorld);
    const float rotErr=RotationError(c.root->world.rotate,plan.expectedRootRotation);
    const bool geometryOK=NearTransform(c.parent->local,applied) &&
        std::isfinite(posErr) && std::isfinite(rotErr) && posErr<=0.5f && rotErr<=0.025f;
    if (!geometryOK) {
        c.parent->local.rotate=newBase.rotate;
        c.parent->local.translate=newBase.translate;
        RefreshOnce(c.parent);
        SoloFailClose("REBASE_GEOMETRY_REJECTED"); return;
    }
    solo.before=newBase;
    solo.applied=applied;
    solo.carrierWorldAfter=c.parent->world.rotate;
    solo.actorWorldAfter=solo.actor3D->world.rotate;
    ++soloRecoveries;
    spdlog::info("[SoloSync] REBASE actor={:08X} baseZ={:.3f} posErr={:.4f} rotErr={:.5f} recoveries={}/{} topologyWrites=0",
        kPlayerID,newBase.translate.z,posErr,rotErr,soloRecoveries,syncMaxRecoveries.load());
}
float SyncRootGap() {
    if (!pair[0].valid || !pair[1].valid || !pair[0].npcRoot || !pair[1].npcRoot)
        return -1.f;
    const auto a=pair[0].npcRoot->world.translate;
    const auto b=pair[1].npcRoot->world.translate;
    return (Finite(a) && Finite(b)) ? Distance(a,b) : -1.f;
}
void SyncFailClose(const char* reason) {
    if (syncFailClosed) return;
    RevokeNSTInterop("PAIR_FAIL_CLOSED");
    syncFailClosed=true;
    SyncWakeTimer().Cancel();

    const auto a=ReadCarrierState(pair[0]);
    const auto b=ReadCarrierState(pair[1]);
    spdlog::warn("[SyncGuard] FAIL_CLOSED reason={} A={} B={} recovered={} action=RESTORE_STILL_OWNED_ONLY noFurtherReapply=true",
        reason,SyncStateName(a),SyncStateName(b),syncRecoveryCount);
    if (b==SyncCarrierState::kHeld) RestoreOneLocal(pair[1],"sync-fail-closed");
    if (a==SyncCarrierState::kHeld) RestoreOneLocal(pair[0],"sync-fail-closed");

}
void SyncPairOnGameThread(std::uint32_t tid, std::uint64_t epoch) {
    if (!syncGuardEnabled.load() || syncFailClosed ||
        !activeSession.load() || !pairApplied.load() ||
        activeFramework.load()!=Framework::kOStim ||
        activeThread.load()!=tid || activeStartEpoch.load()!=epoch ||
        eventEpoch.load()!=epoch || !poseProbe.committed)
        return;
    const auto a=ReadCarrierState(pair[0]);
    const auto b=ReadCarrierState(pair[1]);
    const float gap=SyncRootGap();
    spdlog::debug("[SyncGuard] PAIR_CHECK thread={} epoch={} player={} partner={} rootGap={:.3f} baselineGap={:.3f} recoveries={}/{}",
        tid,epoch,SyncStateName(a),SyncStateName(b),gap,syncBaselineRootGap,
        syncRecoveryCount,syncMaxRecoveries.load());
    if (a==SyncCarrierState::kHeld && b==SyncCarrierState::kHeld) return;
    const auto reset=[&](SyncCarrierState x) {
        return x==SyncCarrierState::kNeutral || x==SyncCarrierState::kBaseReset;
    };
    if (reset(a) && reset(b)) {
        RevokeNSTInterop("PAIR_BOTH_RESET");
        syncFailClosed=true;
        spdlog::info("[SyncGuard] BOTH_BASE_RESET noReapply=true sceneControlReleased=true");
        return;
    }
    const bool recoverA=reset(a) && b==SyncCarrierState::kHeld;
    const bool recoverB=reset(b) && a==SyncCarrierState::kHeld;
    if (!recoverA && !recoverB) {
        SyncFailClose("UNRECOGNIZED_OR_TOPOLOGY_CHANGE");
        return;
    }
    if (syncRecoveryCount>=static_cast<std::uint32_t>(syncMaxRecoveries.load())) {
        SyncFailClose("RECOVERY_BUDGET_EXHAUSTED");
        return;
    }
    auto& lost=recoverA ? pair[0] : pair[1];
    const auto outerDelta=RotationError(lost.actor3D->world.rotate,lost.actorWorldAfter);
    if (outerDelta>0.03f || !std::isfinite(outerDelta)) {
        SyncFailClose("OUTER_WORLD_ORIENTATION_CHANGED");
        return;
    }
    const auto candidate=Inspect(lost.actorID,false);
    if (!candidate.valid || candidate.parent!=lost.npcParent.get() ||
        candidate.root!=lost.npcRoot.get()) {
        SyncFailClose("CARRIER_CHANGED");
        return;
    }
    const auto& ancestor=candidate.parent->parent->world;
    const auto& neutral=candidate.parent->local;

    const auto calculatedWorldPos=Add(ancestor.translate,Rotate(ancestor.rotate,Mul(neutral.translate,ancestor.scale)));
    const auto calculatedWorldRot=Multiply(ancestor.rotate,neutral.rotate);
    if (!Finite(calculatedWorldPos) ||
        Distance(calculatedWorldPos,candidate.parent->world.translate)>0.5f ||
        RotationError(calculatedWorldRot,candidate.parent->world.rotate)>0.025f) {
        SyncFailClose("STALE_WORLD_PROPAGATION");
        return;
    }

    const auto plan=PlanNPCParent(candidate,syncSharedWorldRotation,syncSharedPivot);
    if (!plan.valid) {
        SyncFailClose("INVALID_REBASE_PLAN");
        return;
    }
    auto* carrier=lost.npcParent.get();
    const auto newBase=carrier->local;
    auto newApplied=newBase;
    newApplied.rotate=plan.rotation;
    newApplied.translate=plan.translation;
    carrier->local.rotate=newApplied.rotate;
    carrier->local.translate=newApplied.translate;
    RefreshOnce(carrier);
    const auto afterGap=SyncRootGap();
    const auto rootPosErr=Distance(lost.npcRoot->world.translate,plan.expectedRootWorld);
    const auto rootRotErr=RotationError(lost.npcRoot->world.rotate,plan.expectedRootRotation);

    const float immediateGapChange=std::fabs(afterGap-gap);
    const bool rootGeometryOK=NearTransform(carrier->local,newApplied) &&
        rootPosErr<=0.5f && rootRotErr<=0.025f;
    const bool separationOK=std::isfinite(gap) && std::isfinite(afterGap) &&
        gap>=0.f && afterGap>=0.f && afterGap<=syncMaxRootGap.load() &&
        immediateGapChange<=8.f;
    const bool geometryOK=rootGeometryOK && separationOK;
    spdlog::info("[SyncGuard] REBASE actor={:08X} target={} previousBaseZ={:.3f} currentBaseZ={:.3f} baseShift={:.3f} rootGapBefore={:.3f} rootGapAfter={:.3f} immediateGapChange={:.3f} previousObservedGap={:.3f} rootPosErr={:.4f} rootRotErr={:.5f} rootGeometryOK={} separationOK={} geometryOK={} topologyWrites=0",
        lost.actorID,recoverA?"PLAYER":"PARTNER",lost.before.translate.z,newBase.translate.z,
        Distance(newBase.translate,lost.before.translate),gap,afterGap,immediateGapChange,syncBaselineRootGap,
        rootPosErr,rootRotErr,rootGeometryOK,separationOK,geometryOK);
    if (!geometryOK) {

        carrier->local.rotate=newBase.rotate;
        carrier->local.translate=newBase.translate;
        RefreshOnce(carrier);
        SyncFailClose("REBASE_GEOMETRY_REJECTED");
        return;
    }
    lost.before=newBase;
    lost.applied=newApplied;
    lost.carrierWorldAfter=carrier->world.rotate;
    lost.actorWorldAfter=lost.actor3D->world.rotate;
    syncBaselineRootGap=afterGap;
    ++syncRecoveryCount;
    spdlog::debug("[SyncGuard] PAIR_RESYNCHRONIZED player={} partner={} recoveries={}/{} baseUpdated=true noFrameHook=true",
        SyncStateName(ReadCarrierState(pair[0])),SyncStateName(ReadCarrierState(pair[1])),
        syncRecoveryCount,syncMaxRecoveries.load());

}

void MultiFailClose(const char* reason) {
    if (multiFailClosed) return;
    RevokeNSTInterop("MULTI_FAIL_CLOSED");
    multiFailClosed=true;
    SyncWakeTimer().Cancel();
    spdlog::warn("[MultiSync] FAIL_CLOSED reason={} actors={} recoveries={} action=RESTORE_HELD_ONLY",
        reason,multiCount,multiRecoveries);

    for (std::size_t i=multiCount;i>0;--i)
        if (ReadCarrierState(multi[i-1])==SyncCarrierState::kHeld)
            RestoreOneLocal(multi[i-1],"multi-sync-fail-closed");
}
void SyncMultiOnGameThread(std::uint32_t tid,std::uint64_t epoch) {
    if (!multiEnabled.load() || !syncGuardEnabled.load() || multiFailClosed ||
        !activeSession.load() || activeFramework.load()!=Framework::kOStim ||
        activeThread.load()!=tid || activeStartEpoch.load()!=epoch ||
        eventEpoch.load()!=epoch || !multiProbe.committed ||
        multiCount<3 || multiCount>kMaxActors) return;
    std::array<SyncCarrierState,kMaxActors> states{};
    std::array<std::size_t,kMaxActors> lost{};
    std::size_t heldCount=0,lostCount=0;
    bool bad=false;
    for (std::size_t i=0;i<multiCount;++i) {
        states[i]=ReadCarrierState(multi[i]);
        if (states[i]==SyncCarrierState::kHeld) ++heldCount;
        else if (states[i]==SyncCarrierState::kNeutral || states[i]==SyncCarrierState::kBaseReset) lost[lostCount++]=i;
        else bad=true;
        spdlog::debug("[MultiSync] STATE slot={} actor={:08X} state={}",i,multiRoster[i],SyncStateName(states[i]));
    }
    spdlog::debug("[MultiSync] GROUP thread={} epoch={} actors={} held={} neutral={} maxGap={:.3f} recovered={}/{}",
        tid,epoch,multiCount,heldCount,lostCount,MultiMaxRootGap(),multiRecoveries,syncMaxRecoveries.load());
    if (bad) { MultiFailClose("UNRECOGNIZED_OR_TOPOLOGY_CHANGE"); return; }
    if (heldCount==multiCount) return;
    if (heldCount==0) {
        RevokeNSTInterop("ALL_RESET");
        multiFailClosed=true;
        spdlog::info("[MultiSync] ALL_RESET noReapply=true");
        return;
    }
    if (multiRecoveries+lostCount>static_cast<std::uint32_t>(syncMaxRecoveries.load())) {
        MultiFailClose("RECOVERY_BUDGET_EXHAUSTED"); return;
    }
    for (std::size_t n=0;n<lostCount;++n) {
        const auto idx=lost[n];
        auto& state=multi[idx];
        const auto outerDelta=RotationError(state.actor3D->world.rotate,state.actorWorldAfter);
        if (!std::isfinite(outerDelta) || outerDelta>0.03f) {
            MultiFailClose("OUTER_WORLD_ROTATION_CHANGED"); return;
        }
        const auto candidate=Inspect(state.actorID,false);
        if (!candidate.valid || candidate.parent!=state.npcParent.get() || candidate.root!=state.npcRoot.get()) {
            MultiFailClose("CARRIER_REPLACED"); return;
        }
        const auto& ancestor=candidate.parent->parent->world;
        const auto newBase=candidate.parent->local;
        const auto expectedWorldPos=Add(ancestor.translate,Rotate(ancestor.rotate,Mul(newBase.translate,ancestor.scale)));
        const auto expectedWorldRot=Multiply(ancestor.rotate,newBase.rotate);
        if (!Finite(expectedWorldPos) || Distance(expectedWorldPos,candidate.parent->world.translate)>0.5f ||
            RotationError(expectedWorldRot,candidate.parent->world.rotate)>0.025f) {
            MultiFailClose("STALE_WORLD_PROPAGATION"); return;
        }
        const auto plan=PlanNPCParent(candidate,multiWorldTilt,multiPivot);
        if (!plan.valid) { MultiFailClose("INVALID_REBASE_PLAN"); return; }
        auto applied=newBase;
        applied.rotate=plan.rotation;
        applied.translate=plan.translation;
        candidate.parent->local.rotate=applied.rotate;
        candidate.parent->local.translate=applied.translate;
        RefreshOnce(candidate.parent);
        const auto rootErr=Distance(candidate.root->world.translate,plan.expectedRootWorld);
        const auto rotErr=RotationError(candidate.root->world.rotate,plan.expectedRootRotation);
        const bool ok=NearTransform(candidate.parent->local,applied) && rootErr<=0.5f && rotErr<=0.025f;
        spdlog::info("[MultiSync] REBASE slot={} actor={:08X} priorBaseZ={:.3f} newBaseZ={:.3f} rootErr={:.4f} rotErr={:.5f} ok={}",
            idx,state.actorID,state.before.translate.z,newBase.translate.z,rootErr,rotErr,ok);
        if (!ok) {
            candidate.parent->local.rotate=newBase.rotate;
            candidate.parent->local.translate=newBase.translate;
            RefreshOnce(candidate.parent);
            MultiFailClose("REBASE_GEOMETRY_REJECTED"); return;
        }
        state.before=newBase;
        state.applied=applied;
        state.carrierWorldAfter=candidate.parent->world.rotate;
        state.actorWorldAfter=state.actor3D->world.rotate;
        ++multiRecoveries;
    }
    multiBaselineMaxGap=MultiMaxRootGap();
    spdlog::debug("[MultiSync] GROUP_RESYNCHRONIZED actors={} recoveries={}/{} maxRootGap={:.3f} topologyWrites=0 noFrameHook=true",
        multiCount,multiRecoveries,syncMaxRecoveries.load(),multiBaselineMaxGap);
    AuditMultiWorld("POST_RESYNC");
}

void AuditNodeAfterCommit(std::uint32_t tid, std::uint64_t epoch, std::uint64_t serial) {
    if (!activeSession.load() || activeThread.load()!=tid ||
        activeStartEpoch.load()!=epoch || !(soloProbe.committed || poseProbe.committed || multiProbe.committed)) return;
    spdlog::debug("[LocalAudit] NODE_CHANGED thread={} epoch={} serial={} syncGuardEnabled={}",tid,epoch,serial,syncGuardEnabled.load());
    if (soloProbe.committed) AuditOneLocal(solo,"SOLO_NODE_CHANGED");
    else if (multiActiveCount.load()) AuditMulti("NODE_CHANGED");
    else AuditLocalPair("NODE_CHANGED");
    if (furnitureRefID.load()) ReadFurnitureOnGameThread(tid,furnitureRefID.load(),"NODE_CHANGED");
    if (syncGuardEnabled.load() &&
        !(soloProbe.committed ? soloFailClosed : (multiActiveCount.load() ? multiFailClosed : syncFailClosed)) && !graphAuditOnly.load()) {
        spdlog::debug("[SyncGuard] WAKE_ARMED thread={} epoch={} serial={} delayMs={}",
            tid,epoch,serial,syncDelayMs.load());
        SyncWakeTimer().Arm(tid,epoch,std::chrono::milliseconds(syncDelayMs.load()));
    }
}

void HandleOStimModEvent(std::string_view, std::uint32_t, std::uint32_t, bool,
                         std::vector<std::uint32_t>, std::int64_t, std::uint32_t = 0);

void ReadFurnitureOnGameThread(std::uint32_t tid,std::uint32_t refID,const char* stage) {
    if (!furnitureProbeEnabled.load()) return;
    if (!refID) {
        spdlog::debug("[FurnitureProbe] thread={} stage={} ref=NONE (other visual props may still be actor-attached)",tid,stage);
        return;
    }
    auto* ref=RE::TESForm::LookupByID<RE::TESObjectREFR>(refID);
    if (!ref) {
        spdlog::warn("[FurnitureProbe] thread={} stage={} ref={:08X} status=UNRESOLVED noWrites=true",tid,stage,refID);
        return;
    }
    auto* base=ref->GetBaseObject();
    const auto* node=ref->Get3D();
    spdlog::debug("[FurnitureProbe] thread={} stage={} ref={:08X} base={:08X} has3D={} dynamicIDHint={} world=({:.2f},{:.2f},{:.2f}) noWrites=true",
        tid,stage,refID,base?base->GetFormID():0u,node!=nullptr,(refID>>24)==0xFF,
        ref->GetPosition().x,ref->GetPosition().y,ref->GetPosition().z);
}
class NativeStartListener final : public OStimLegacy::ThreadEventListener {
public:
    void threadRegistered(OStimLegacy::Thread* thread) override {
        const auto receivedAtMs = NowMs();
        if (!thread) { spdlog::warn("[NativeAPI] START_NULL_THREAD"); return; }
        const auto tidSigned = thread->getThreadID();
        const bool playerThread = thread->isPlayerThread();
        if (tidSigned < 0 || !playerThread) {
            spdlog::debug("[NativeAPI] IGNORE_NPC_THREAD start={} playerThread={}",tidSigned,playerThread);
            return;
        }
        const auto tid = static_cast<std::uint32_t>(tidSigned);
        const auto count = thread->getActorCount();
        if (count<1 || count>5) {
            spdlog::warn("[NativeAPI] SNAPSHOT_BAD_COUNT thread={} count={}",tid,count);
            return;
        }
        std::vector<std::uint32_t> ids;
        ids.reserve(count);
        for (std::uint32_t i=0;i<count;++i) {
            auto* entry = thread->getActor(i);
            auto* actor = entry ? static_cast<RE::Actor*>(entry->getGameActor()) : nullptr;
            const auto id = actor ? actor->GetFormID() : 0u;
            ids.push_back(id);
        }
        spdlog::debug("[NativeAPI] SNAPSHOT_READY thread={} count={} captureMs={} source=LegacyThreads",
            tid,ids.size(),NowMs()-receivedAtMs);
        std::uint32_t furnitureID=0;
        if (furnitureProbeEnabled.load() && ostimAPI) {
            const auto version=ostimAPI->getVersion();
            if (version>=3) {

                auto* ref=static_cast<RE::TESObjectREFR*>(thread->getFurnitureObject());
                furnitureID=ref ? ref->GetFormID() : 0u;
                spdlog::debug("[FurnitureProbe] SNAPSHOT thread={} abi={} ref={:08X} typePtr={} noWrites=true",
                    tid,version,furnitureID,thread->getFurnitureType());
            } else if (!furnitureAbiWarningShown.exchange(true)) {
                spdlog::debug("[FurnitureProbe] ABI_TOO_OLD version={} min=3 skipped=true skipBoundScenesEffective=false (furniture ref unavailable in this ABI)",version);
            }
        }
        HandleOStimModEvent("ostim_thread_start",tid,0,true,std::move(ids),receivedAtMs,furnitureID);
    }
};

class NativeNodeChangedListener final : public OStimLegacy::ThreadEventListener {
public:
    void threadRegistered(OStimLegacy::Thread* thread) override {
        if (!thread) return;
        const auto id=thread->getThreadID();
        if (id<0 || !thread->isPlayerThread()) return;
        const auto serial=nativeNodeSerial.fetch_add(1)+1;
        spdlog::debug("[NativeAPI] NODE_CHANGED thread={} serial={} startAgeMs={} (signal-only; no graph writes)",
            id,serial,SignalAgeMs());
        std::uint64_t epoch=0;
        {
            std::scoped_lock lock(callbackMutex);
            if (ostimSignalActive && ostimSignalThread==static_cast<std::uint32_t>(id)) epoch=ostimSignalEpoch;
        }
        if (epoch) EnqueueGameTask([tid=static_cast<std::uint32_t>(id),epoch,serial] {
            AuditNodeAfterCommit(tid,epoch,serial);
        });
    }
};
class NativeStopListener final : public OStimLegacy::ThreadEventListener {
public:
    void threadRegistered(OStimLegacy::Thread* thread) override {
        if (!thread) { spdlog::warn("[NativeAPI] STOP_NULL_THREAD"); return; }
        const auto tidSigned=thread->getThreadID();
        if (tidSigned<0) return;
        const auto tid=static_cast<std::uint32_t>(tidSigned);

        {
            std::scoped_lock lock(callbackMutex);
            if (!ostimSignalActive || ostimSignalThread!=tid) return;
        }
        HandleOStimModEvent("ostim_thread_end",tid,0,true,{},0);
    }
};
NativeStartListener nativeStartListener;
NativeStopListener nativeStopListener;
NativeNodeChangedListener nativeNodeChangedListener;
void ConnectOStimNativeAPI(const char* phase) {
    if (nativeAPIReady.load()) return;
    if (!::GetModuleHandleA("OStim.dll")) {
        spdlog::warn("[NativeAPI] OStim.dll not loaded at {}",phase);
        return;
    }
    auto* messaging=SKSE::GetMessagingInterface();
    if (!messaging) { spdlog::warn("[NativeAPI] SKSE messaging unavailable at {}",phase); return; }
    OStimLegacy::InterfaceExchangeMessage exchange{};
    const bool dispatched=messaging->Dispatch(
        OStimLegacy::InterfaceExchangeMessage::MESSAGE_TYPE,
        &exchange, static_cast<std::uint32_t>(sizeof(exchange)), "OStim");
    spdlog::debug("[NativeAPI] EXCHANGE phase={} dispatched={} map={} bytes={}",
        phase,dispatched,static_cast<void*>(exchange.interfaceMap),sizeof(exchange));
    if (!exchange.interfaceMap) {
        spdlog::warn("[NativeAPI] EXCHANGE_NO_MAP phase={} (will retry at kDataLoaded)",phase);
        return;
    }
    auto* iface=exchange.interfaceMap->queryInterface(OStimLegacy::ThreadInterface::NAME);
    if (!iface) {
        spdlog::warn("[NativeAPI] THREADS_NOT_FOUND phase={}",phase);
        return;
    }

    auto* api=static_cast<OStimLegacy::ThreadInterface*>(iface);
    api->registerThreadStartListener(&nativeStartListener);
    api->registerThreadStopListener(&nativeStopListener);
    api->registerNodeChangedListener(&nativeNodeChangedListener);
    ostimAPI=api;
    legacyThreadsABI.store(api->getVersion());
    nativeAPIReady.store(true);
    spdlog::debug("[NativeAPI] CONNECTED LEGACY_THREADS phase={} api={} listener=Start+Stop+NodeChanged",phase,static_cast<void*>(api));

}

void HandleOStimModEvent(std::string_view name, std::uint32_t tid, std::uint32_t senderID,
                          bool native, std::vector<std::uint32_t> nativeRoster,
                          std::int64_t nativeReceivedAtMs, std::uint32_t nativeFurnitureID) {
    if (!native && nativeAPIReady.load()) return;
    const bool canonicalStart=name=="ostim_thread_start";
    const bool canonicalEnd=name=="ostim_thread_end";
    const bool start=canonicalStart || name=="ostim_start";
    const bool end=canonicalEnd || name=="ostim_end";
    if (!start && !end) return;
    std::uint64_t epoch=0;
    bool doStart=false,doEnd=false;
    std::uint32_t actualID=tid;
    {
        std::scoped_lock lock(callbackMutex);

        if (canonicalStart && tid!=0 && !native) {
            spdlog::debug("[SceneEvent] SKIP_NONPLAYER_OR_UNVERIFIED thread={} (sender={:08X})",tid,senderID);
            return;
        }

        if (canonicalEnd && (!ostimSignalActive || ostimSignalThread!=tid)) return;
        if (canonicalStart || canonicalEnd) ostimCanonicalSeen=true;

        if (ostimCanonicalSeen && !canonicalStart && !canonicalEnd) return;
        if (start) {
            if (ostimSignalActive) {

                spdlog::debug("[SceneEvent] DUP_OR_PARALLEL_START name={} tid={} trackedThread={}",name,tid,ostimSignalThread);
                return;
            }
            ostimSignalActive=true;
            ostimSignalThread=tid;
            actualID=tid;
            epoch=eventEpoch.fetch_add(1)+1;
            ostimSignalEpoch=epoch;
            signalAtMs.store(nativeReceivedAtMs ? nativeReceivedAtMs : NowMs());
            doStart=true;
        } else {
            if (!ostimSignalActive) return;
            if (canonicalEnd && ostimSignalThread!=tid) {
                spdlog::debug("[SceneEvent] IGNORE_END unmatched thread={} expected={}",tid,ostimSignalThread);
                return;
            }
            actualID=ostimSignalThread;
            ostimSignalActive=false;
            epoch=eventEpoch.fetch_add(1)+1;
            ostimSignalEpoch=0;
            doEnd=true;
        }
    }
    spdlog::debug("[SceneEvent] {} framework=OStim source={} name={} thread={} epoch={} sender={:08X}",
        doStart?"START":"END",native?"LegacyThreads":"ModCallback",name,actualID,epoch,senderID);
    if (doStart) Timeline("EVENT_START",actualID,epoch);
    else { RevokeNSTInterop("END_SIGNAL"); Timeline("EVENT_END",actualID,epoch); }
    if (doStart) EnqueueGameTask([actualID,senderID,epoch,nativeFurnitureID,roster=std::move(nativeRoster)]() mutable {
        if (eventEpoch.load()!=epoch) return;
        furnitureRefID.store(nativeFurnitureID);
        if (furnitureProbeEnabled.load()) ReadFurnitureOnGameThread(actualID,nativeFurnitureID,"START");
        StartOnGameThread(Framework::kOStim,actualID,senderID,epoch,std::move(roster));
    });
    if (doEnd) EnqueueGameTask([=]{EndOnGameThread(Framework::kOStim,actualID,epoch);});
}
void HandleSexLabModEvent(std::string_view name,std::uint32_t senderID) {
    if (!sexLabEnabled.load()) return;
    if (name!="AnimationStart" && name!="AnimationEnd" &&
        name!="HookAnimationStart" && name!="HookAnimationEnd") return;

    spdlog::debug("[SexLab] OBSERVE_ONLY event={} sender={:08X} (actor bridge not yet implemented)",name,senderID);
}
class AdultEventSink final : public RE::BSTEventSink<SKSE::ModCallbackEvent> {
public:
    static AdultEventSink* GetSingleton() {
        static AdultEventSink instance;
        return &instance;
    }
    RE::BSEventNotifyControl ProcessEvent(const SKSE::ModCallbackEvent* event,
        RE::BSTEventSource<SKSE::ModCallbackEvent>*) override {
        if (!event) return RE::BSEventNotifyControl::kContinue;
        const char* raw=event->eventName.c_str();
        if (!raw || !*raw) return RE::BSEventNotifyControl::kContinue;
        const std::string_view name(raw);
        std::uint32_t senderID=0;
        if (event->sender) {
            if (auto* actor=event->sender->As<RE::Actor>()) senderID=actor->GetFormID();
        }
        if (name=="ostim_thread_start" || name=="ostim_thread_end" || name=="ostim_start" || name=="ostim_end") {
            const float n=event->numArg;

            const auto tid=std::isfinite(n) && n>=0.f && n<2147483648.f ? static_cast<std::uint32_t>(n) : 0u;
            HandleOStimModEvent(name,tid,senderID,false,{},0);
        } else if (sexLabEnabled.load()) {
            HandleSexLabModEvent(name,senderID);
        }
        return RE::BSEventNotifyControl::kContinue;
    }
};
void InstallEventSink() {
    if (eventSinkInstalled) return;
    auto* source=SKSE::GetModCallbackEventSource();
    if (!source) {
        spdlog::error("[SceneEvent] No SKSE ModCallbackEvent source");
        return;
    }
    source->AddEventSink(AdultEventSink::GetSingleton());
    eventSinkInstalled=true;
    spdlog::debug("[SceneEvent] SKSE ModCallback sink installed: OStim thread+legacy");
}
void __stdcall RenderMenu() {
    ImGuiMCP::SeparatorText("NSFW Scene Tilt");
    ImGuiMCP::TextWrapped("Tilts the virtual stage to match uneven ground during OStim scenes, keeping actors aligned.");
    const auto ini=std::filesystem::absolute(std::filesystem::path(kNSTIniPath));

    bool enabled=autoTerrainEnabled.load();
    if (ImGuiMCP::Checkbox("Enable NST",&enabled)) {
        if (::WritePrivateProfileStringW(L"General",L"EnableNST",enabled?L"1":L"0",ini.c_str())) {
            autoTerrainEnabled.store(enabled);
            spdlog::info("[Settings] NST enabled={} (applies to new scenes; active stage retained until End)",enabled);
        } else spdlog::error("[Settings] Failed to save EnableNST; unchanged");
    }

    static constexpr const char* levelNames[]{"Trace", "Debug", "Info", "Warn", "Error", "Critical", "Off"};
    std::int32_t level=configuredLogLevel.load();
    if (ImGuiMCP::Combo("Log level",&level,levelNames,static_cast<int>(std::size(levelNames)))) {
        const auto previous=configuredLogLevel.load();
        const auto value=LogLevelName(level);
        std::wstring wideValue(value,value+std::strlen(value));
        if (::WritePrivateProfileStringW(L"Logging",L"Level",wideValue.c_str(),ini.c_str())) {
            spdlog::info("[Logging] Level changed {} -> {}",LogLevelName(previous),value);
            SetLogLevel(level);
        } else spdlog::error("[Logging] Failed to save Level to INI; unchanged");
    }

    if (fatalGraphFault.load()) ImGuiMCP::TextWrapped("TASK/GRAPH FAULT: quit WITHOUT saving!");
}

void ResetOnLoad() {
    RevokeNSTInterop("LOAD_RESET");
    nstPairWorldVerified=false;
    if (solo.valid || pair[0].valid || pair[1].valid || MultiHasState()) spdlog::warn("[SceneReset] game loaded during local ownership; discard references without writing stale 3D");
    ClearPair();
    ClearMulti();
    ClearSolo();
    soloProbe={};
    furnitureRefID.store(0);
    auditOnlyActive.store(false);
    LocalAuditWakeTimer().Cancel();
    SyncWakeTimer().Cancel();
    PoseWakeTimer().Cancel();
    activeSession.store(false);
    activeFramework.store(Framework::kNone);
    activeThread.store(0);
    actorBID.store(0);
    pendingStartEpoch.store(0);
    pendingThread.store(0);
    poseProbe={};
    nativeNodeSerial.store(0);
    signalAtMs.store(0);
    activeStartEpoch.store(0);
    eventEpoch.fetch_add(1);
    {
        std::scoped_lock lock(callbackMutex);
        ostimSignalActive=false; ostimCanonicalSeen=false; ostimSignalEpoch=0;
    }
    appliedRoll.store(0.f);
    fatalGraphFault.store(false);

}
void OnMessage(SKSE::MessagingInterface::Message* message) {
    if (!message) return;
    if (message->type==SKSE::MessagingInterface::kPostLoad) ConnectOStimNativeAPI("kPostLoad");
    if (message->type==SKSE::MessagingInterface::kDataLoaded) {
        if (!nativeAPIReady.load()) ConnectOStimNativeAPI("kDataLoaded");
        InstallEventSink();
        if (!menuRegistered) {
            if (SKSEMenuFramework::IsInstalled() && GetMenuFrameworkModule()) {
                SKSEMenuFramework::SetSection("NSFW Scene Tilt");
                SKSEMenuFramework::AddSectionItem("Scene Tilt Settings",RenderMenu);
                menuRegistered=true;
                spdlog::debug("[SceneMenu] registered");
            } else spdlog::warn("[SceneMenu] SKSE Menu Framework unavailable");
        }
    }
    if (message->type==SKSE::MessagingInterface::kNewGame ||
        message->type==SKSE::MessagingInterface::kPostLoadGame) ResetOnLoad();
}
}

extern "C" __declspec(dllexport) std::uint32_t TMS_NST_QueryActorTiltV1(std::uint32_t actorFormID) noexcept {
    return nstInteropState.Contains(actorFormID) ? 1u : 0u;
}

SKSEPluginLoad(const SKSE::LoadInterface* skse) {
    SKSE::Init(skse);
    const auto log=SKSE::log::log_directory();
    if (!log) return false;
    auto logger=spdlog::basic_logger_mt("TMS_NSFWSceneTilt",(*log/"TMS_NSFWSceneTilt.log").string(),true);
    spdlog::set_default_logger(std::move(logger));
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");
    const auto ini=std::filesystem::absolute(std::filesystem::path(L"Data/SKSE/Plugins/TMS_NSFWSceneTilt.ini"));
    SetLogLevel(ReadLogLevel(ini));

    spdlog::flush_on(spdlog::level::warn);
    spdlog::flush_every(std::chrono::seconds(2));
    const auto ver=REL::Module::get().version();
    spdlog::info("============================================================");
    auto runtimeVersion=ver.string();
    std::replace(runtimeVersion.begin(),runtimeVersion.end(),'-','.');
    spdlog::info("TMS NSFW Scene Tilt v{} | Skyrim {} | Logging {}",kNSTVersion,runtimeVersion,LogLevelName(configuredLogLevel.load()));
    spdlog::info("============================================================");
    if (ver!=REL::Version(1,5,97,0) && ver!=REL::Version(1,6,1170,0)) {
        spdlog::error("Unsupported runtime {}",ver.string());
        return false;
    }
    const auto oldNSTEnabled=::GetPrivateProfileIntW(L"Terrain",L"EnableAuto",1,ini.c_str());
    autoTerrainEnabled.store(::GetPrivateProfileIntW(L"General",L"EnableNST",oldNSTEnabled,ini.c_str())!=0);
    spdlog::debug("[Settings] EnableNST={} TerrainRadius={:.0f} (internal)",
        autoTerrainEnabled.load(),kTerrainSampleRadius);
    localAuditEnabled.store(::GetPrivateProfileIntW(L"LocalAudit",L"Enabled",1,ini.c_str())!=0);
    localAuditDelayMs.store(std::clamp(static_cast<int>(::GetPrivateProfileIntW(L"LocalAudit",L"DelayMs",600,ini.c_str())),100,5000));
    graphAuditOnly.store(::GetPrivateProfileIntW(L"GraphAudit",L"OnlyNoGraphWrites",0,ini.c_str())!=0);
    multiEnabled.store(::GetPrivateProfileIntW(L"MultiActor",L"Enabled",1,ini.c_str())!=0);
    multiMaxSlopeDegrees.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"MultiActor",L"MaxSlopeDegrees",35,ini.c_str())),15.f,35.f));
    furnitureProbeEnabled.store(::GetPrivateProfileIntW(L"FurnitureProbe",L"Enabled",1,ini.c_str())!=0);
    skipBoundFurniture.store(::GetPrivateProfileIntW(L"FurnitureProbe",L"SkipBoundScenes",1,ini.c_str())!=0);
    syncGuardEnabled.store(::GetPrivateProfileIntW(L"SyncGuard",L"Enabled",1,ini.c_str())!=0);
    syncDelayMs.store(std::clamp(static_cast<int>(::GetPrivateProfileIntW(L"SyncGuard",L"DelayMs",48,ini.c_str())),16,250));
    syncMaxRecoveries.store(std::clamp(static_cast<int>(::GetPrivateProfileIntW(L"SyncGuard",L"MaxRecoveries",8,ini.c_str())),1,32));
    syncMaxRootGap.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"SyncGuard",L"MaxRootGap",64,ini.c_str())),16.f,256.f));
    indoorProbeEnabled.store(::GetPrivateProfileIntW(L"IndoorProbe",L"Enabled",1,ini.c_str())!=0);
    indoorProbeRadius.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"Radius",64,ini.c_str())),24.f,160.f));
    indoorRayAbove.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"RayAbove",48,ini.c_str())),16.f,192.f));
    indoorRayBelow.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"RayBelow",128,ini.c_str())),48.f,384.f));
    indoorMaxPlaneResidual.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"MaxPlaneResidual",8,ini.c_str())),1.f,64.f));
    indoorMaxCenterStep.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"MaxCenterStep",80,ini.c_str())),4.f,150.f));
    indoorMaxActorOffset.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"MaxActorOffset",48,ini.c_str())),4.f,96.f));
    indoorMinNormalUpZ.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"MinNormalUpPercent",65,ini.c_str()))/100.f,0.1f,1.f));
    indoorMaxNormalDeviationDeg.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"IndoorProbe",L"MaxNormalDeviationDeg",25,ini.c_str())),1.f,89.f));
    indoorTiltEnabled.store(::GetPrivateProfileIntW(L"IndoorTilt",L"Enabled",1,ini.c_str())!=0);
    maxStartAgeMs.store(std::clamp(static_cast<int>(::GetPrivateProfileIntW(L"Timing",L"MaxStartAgeMs",200,ini.c_str())),50,500));
    minPoseAgeMs.store(std::clamp(static_cast<int>(::GetPrivateProfileIntW(L"Timing",L"MinPoseAgeMs",32,ini.c_str())),0,150));
    poseSampleSpacingMs.store(std::clamp(static_cast<int>(::GetPrivateProfileIntW(L"Timing",L"PoseSampleSpacingMs",16,ini.c_str())),8,64));
    maxUnsettledRootGap.store(std::clamp(static_cast<float>(::GetPrivateProfileIntW(L"Timing",L"MaxUnsettledRootGap",64,ini.c_str())),8.f,256.f));
    papyrusFallbackEnabled.store(::GetPrivateProfileIntW(L"Framework",L"EnablePapyrusFallback",0,ini.c_str())!=0);
    spdlog::debug("[Timing] MaxStartAgeMs={} MinPoseAgeMs={} SampleSpacingMs={} MaxUnsettledRootGap={:.0f} fadeState=UNVERIFIED PapyrusFallback={}",
        maxStartAgeMs.load(),minPoseAgeMs.load(),poseSampleSpacingMs.load(),maxUnsettledRootGap.load(),papyrusFallbackEnabled.load());
    spdlog::debug("[SceneOption] terrainAuto={} radius={:.0f}",
        autoTerrainEnabled.load(),kTerrainSampleRadius);
    spdlog::debug("[LocalAudit] Carrier=NPC Enabled={} DelayMs={} OnlyNoGraphWrites={} topologyWrites=0",
        localAuditEnabled.load(),localAuditDelayMs.load(),graphAuditOnly.load());
    spdlog::debug("[SyncGuard] Enabled={} DelayMs={} MaxRecoveries={} MaxRootGap={:.0f} trigger=NODE_CHANGED outfitBaseAware=true failClosed=true noFrameHooks=true",
        syncGuardEnabled.load(),syncDelayMs.load(),syncMaxRecoveries.load(),syncMaxRootGap.load());
    spdlog::debug("[Features] Solo=true PlayerThreadOnly=true MultiActor={} MultiMaxSlopeDegrees={:.1f} (Solo/2P=30) FurnitureProbe={} (ABI v3 required, READ_ONLY)",multiEnabled.load(),multiMaxSlopeDegrees.load(),furnitureProbeEnabled.load());
    spdlog::debug("[FurnitureProbe] SkipBoundScenes={}",skipBoundFurniture.load());
    spdlog::debug("[IndoorProbe] Enabled={} Radius={:.0f} RayAbove={:.0f} RayBelow={:.0f} MaxResidual={:.0f} MaxStep={:.0f} MaxActorOffset={:.0f} MinNormalUpZ={:.2f} MaxNormalErrDeg={:.0f} ProbeReadOnly=true IndoorTiltConfiguredSeparately=true",
        indoorProbeEnabled.load(),indoorProbeRadius.load(),indoorRayAbove.load(),indoorRayBelow.load(),
        indoorMaxPlaneResidual.load(),indoorMaxCenterStep.load(),indoorMaxActorOffset.load(),
        indoorMinNormalUpZ.load(),indoorMaxNormalDeviationDeg.load());
    spdlog::debug("[IndoorTilt] Enabled={} Scope=ALL_INTERIORS ActorCount=1..5 "
        "SharedSlope=LAND_IDENTICAL(Offset64,Step110,NormalZ0.80,Solo2P30deg,Multi{}deg) "
        "StaticOwnerRequired=true OutdoorUnchanged=true",
        indoorTiltEnabled.load(),multiMaxSlopeDegrees.load());
    auto* papyrus=SKSE::GetPapyrusInterface();
    if (papyrusFallbackEnabled.load()) {
        if (!papyrus || !papyrus->Register(RegisterPapyrus)) {
            spdlog::warn("[Bridge] optional Papyrus fallback registration failed (native still available)");
        } else spdlog::info("[Bridge] optional Papyrus fallback registered");
    }
    auto* messaging=SKSE::GetMessagingInterface();
    return messaging && messaging->RegisterListener(OnMessage);
}
