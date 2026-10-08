#include "pch.h"
#include "CommandDispatcher.h"
#include "FluidWorld.h"
#include "RigidBodyWorld.h"
#include "SoftBodyWorld.h"
#include "FluidVolumeConverter.h"
#include "FluidMeshConverter.h"
#include "VolumeRenderer.h"
#include "FluidMeshRenderer.h"
#include "FluidPLYWriter.h"
#include "RenderBackground.h"
#include "GltfBodyRenderer.h"
#include "GltfSoftRenderer.h"
#include "SSFRPanel.h"
#include "FluidRenderer.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <unordered_set>

using namespace Phantom;

namespace {

bool parseFlt(std::string_view sv, float& out) {
    auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
    return ec == std::errc{};
}

bool parseInt(std::string_view sv, int& out) {
    auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), out);
    return ec == std::errc{};
}

// SSFR anisotropic-kernel settings addressable by SetSSFRKernelParam /
// GetSSFRKernelParam, with their accepted ranges.
struct KernelParamSpec {
    const char* name;
    float lo, hi;
    float SSFRKernelSettings::* field; // null => minNeighbors (int)
};
const KernelParamSpec kKernelParams[] = {
    { "searchScale",   1.0f,  16.0f, &SSFRKernelSettings::searchScale },
    { "maxRatio",      1.0f,  16.0f, &SSFRKernelSettings::maxRatio },
    { "isolatedScale", 0.05f, 1.0f,  &SSFRKernelSettings::isolatedScale },
    { "smoothing",     0.0f,  1.0f,  &SSFRKernelSettings::smoothing },
    { "minNeighbors",  1.0f,  256.0f, nullptr },
};

const KernelParamSpec* findKernelParam(std::string_view name) {
    for (const auto& p : kKernelParams)
        if (name == p.name) return &p;
    return nullptr;
}

// SoftBodyPreset names (see SoftBodyWorld.h) never overlap with
// ScenePreset (rigid-body) names, so "SetPreset:<name>" can be routed
// unambiguously by name alone.
bool isSoftBodyPresetName(std::string_view name) {
    static const std::unordered_set<std::string_view> kNames = {
        "ClothTwoPin", "ClothTopEdge", "ClothWithSphere", "ClothOnBox", "JellySelfOverlap",
        "RopeHanging", "RopePendulum", "RopeBothEndsPinned",
        "JellyDrop", "JellyOnBox", "TwoJelliesStacked", "Mixed",
    };
    return kNames.count(name) != 0;
}

std::vector<std::string_view> split(std::string_view sv, char delim) {
    std::vector<std::string_view> parts;
    size_t start = 0;
    while (true) {
        size_t pos = sv.find(delim, start);
        if (pos == std::string_view::npos) { parts.push_back(sv.substr(start)); break; }
        parts.push_back(sv.substr(start, pos - start));
        start = pos + 1;
    }
    return parts;
}

} // namespace

// ---- IScenarioDispatcher ------------------------------------------------

void CommandDispatcher::dispatch(const std::string& command) {
    queue_.submit(command);
}

std::vector<std::string> CommandDispatcher::collectResponses() {
    return queue_.collectResponses();
}

// Command catalog (help / completion). Built from the command names the PhysicsView
// scenarios exercise, so every entry is known to exist; the arguments are shown as an
// example taken from a scenario rather than a formal grammar (the sub-dispatchers --
// rigid, soft, flame, cloud -- each parse their own).
std::vector<CommandInfo> CommandDispatcher::commandCatalog() const {
    return {
        {"AddBox", "", "e.g. AddBox:0:8:0:0.5:0.5:0.5:1"},
        {"AddCloudSource", "", "e.g. AddCloudSource:150,250,300,150,0.1,3000,0,30"},
        {"AddEmitter", "", "e.g. AddEmitter:0,10,0,1.0,5,0,-1,0,10"},
        {"AddFlameBody", "", "e.g. AddFlameBody:box,0,0.5,0,0.08,0.06,0.08,0.006,1"},
        {"AddFloor", "", "e.g. AddFloor:-10"},
        {"AddOutflowRegion", "", "e.g. AddOutflowRegion:-8,-8,-8,8,-6,8"},
        {"AddSphere", "", "e.g. AddSphere:0:5:0:0.5:1"},
        {"BindFlameBody", "", "e.g. BindFlameBody:1,99"},
        {"ClearCloudSources", "", ""},
        {"ClearEmitters", "", ""},
        {"ClearMeshBoundary", "", ""},
        {"ClearOutflowRegions", "", ""},
        {"ClearRenderBackground", "", ""},
        {"ClearRenderEnvironment", "", ""},
        {"CloudAdvance", "", "e.g. CloudAdvance:60"},
        {"CloudReset", "", "e.g. CloudReset:7"},
        {"CloudStep", "", ""},
        {"ConvertToMesh", "", ""},
        {"ConvertToVolume", "", ""},
        {"FlameCombustionPreset", "", ""},
        {"FlameReset", "", ""},
        {"FlameCirculationPreset", "", "e.g. FlameCirculationPreset:cylinder,0.8,2.4,0.12"},
        {"FlameSpherePreset", "", "e.g. FlameSpherePreset:convection,0.3,0.06"},
        {"FlameStep", "", "e.g. FlameStep:240"},
        {"GetActiveCouplingMode", "", ""},
        {"GetAvgParticlePositionY", "", ""},
        {"GetBodyCount", "", ""},
        {"GetBodyPositionY", "", "e.g. GetBodyPositionY:1"},
        {"GetBodyVelocityY", "", "e.g. GetBodyVelocityY:1"},
        {"GetCloudCsvHeader", "", ""},
        {"GetCloudParam", "", "e.g. GetCloudParam:viscosity"},
        {"GetCloudParticleCount", "", ""},
        {"GetCloudPbvrStat", "", "e.g. GetCloudPbvrStat:accumulated"},
        {"GetCloudSimTime", "", ""},
        {"GetCloudSourceCount", "", ""},
        {"GetCloudStat", "", "e.g. GetCloudStat:simTime"},
        {"GetCloudStats", "", ""},
        {"GetContactCount", "", ""},
        {"GetEmitterCount", "", ""},
        {"GetFlameBodyCount", "", ""},
        {"GetFlameBodyStat", "", "e.g. GetFlameBodyStat:0,state"},
        {"GetFlameFuelBudget", "", ""},
        {"GetFlamePBVRStat", "", "e.g. GetFlamePBVRStat:displayedEnsembles"},
        {"GetFlamePBVRStats", "", ""},
        {"GetFlameParam", "", "e.g. GetFlameParam:fixedCarriers"},
        {"GetFlameParticleCount", "", ""},
        {"GetFlameRenderMode", "", ""},
        {"GetFlameRenderParam", "", "e.g. GetFlameRenderParam:carrierDebug"},
        {"GetFlameSecondaryCount", "", ""},
        {"GetFlameSimTime", "", ""},
        {"GetFlameStat", "", "e.g. GetFlameStat:count"},
        {"GetFlameStats", "", ""},
        {"GetMaxParticlePositionY", "", ""},
        {"GetMaxParticleSpeed", "", ""},
        {"GetMaxSpeed", "", ""},
        {"GetMeshBoundaryTriangleCount", "", ""},
        {"GetMeshTriangleCount", "", ""},
        {"GetMinParticlePositionY", "", ""},
        {"GetOutflowRegionCount", "", ""},
        {"GetParticleCount", "", ""},
        {"GetRenderSceneState", "", ""},
        {"GetRigidRenderMode", "", ""},
        {"GetSSFRAnisotropicKernel", "", ""},
        {"GetSSFRKernelParam", "", "e.g. GetSSFRKernelParam:maxRatio"},
        {"GetSSFRKernelStat", "", "e.g. GetSSFRKernelStat:active"},
        {"GetShadowEnabled", "", ""},
        {"GetSimulationType", "", ""},
        {"GetSoftBodyCount", "", ""},
        {"GetSoftMaxPositionY", "", ""},
        {"GetSoftMinInterBodyDistance", "", "e.g. GetSoftMinInterBodyDistance:0:1"},
        {"GetSoftMinNonEdgeDistance", "", "e.g. GetSoftMinNonEdgeDistance:0"},
        {"GetSoftMinPositionY", "", ""},
        {"GetSoftParticleCount", "", ""},
        {"GetSoftParticlePositionY", "", "e.g. GetSoftParticlePositionY:0:0"},
        {"GetSoftRenderMode", "", ""},
        {"GetSoftTotalVolume", "", "e.g. GetSoftTotalVolume:0"},
        {"GetStatus", "", ""},
        {"HairPreset", "", "e.g. HairPreset:Bundle"},
        {"HairStep", "", "e.g. HairStep:60"},
        {"HairReset", "", ""},
        {"HairClear", "", ""},
        {"SetHairPage", "", "e.g. SetHairPage:true"},
        {"SetHairRunning", "", "e.g. SetHairRunning:true"},
        {"IsHairPage", "", ""},
        {"IsHairRunning", "", ""},
        {"SetHairParam", "", "e.g. SetHairParam:shapeCompliance,0.02"},
        {"GetHairParam", "", "e.g. GetHairParam:substeps"},
        {"GetHairStat", "", "e.g. GetHairStat:lengthError"},
        {"GetVolumeVoxelCount", "", ""},
        {"IsCloudPage", "", ""},
        {"IsCloudRunning", "", ""},
        {"IsCouplingEnabled", "", ""},
        {"IsFlamePage", "", ""},
        {"IsFlameRunning", "", ""},
        {"IsMeshRenderEnabled", "", ""},
        {"IsRunning", "", ""},
        {"IsSSFREnabled", "", ""},
        {"IsSoftFluidCouplingEnabled", "", ""},
        {"IsUIVisible", "", ""},
        {"IsVolumeRenderEnabled", "", ""},
        {"LoadMeshBoundary", "", "e.g. LoadMeshBoundary:${repo_root}/Physics/PhysicsView/scenarios/models/..."},
        {"LoadRenderBackground", "", "e.g. LoadRenderBackground:${repo_root}/Physics/PhysicsView/scenarios/mod..."},
        {"New", "", ""},
        {"NewScene", "", ""},
        {"RemoveCloudSource", "", "e.g. RemoveCloudSource:4"},
        {"RemoveFlameBody", "", "e.g. RemoveFlameBody:999999"},
        {"Reset", "", ""},
        {"SaveMeshToObj", "", "e.g. SaveMeshToObj:testdata/fluid_to_mesh/isotropic.obj"},
        {"SaveScreenshot", "", "e.g. SaveScreenshot:testdata/screenshots/47_flame_normal.png"},
        {"SetCameraOrbit", "", "e.g. SetCameraOrbit:110,0,0.15"},
        {"SetCameraTarget", "", "e.g. SetCameraTarget:0,4,0"},
        {"SetCloudHumidity", "", "e.g. SetCloudHumidity:0.15"},
        {"SetCloudPage", "", "e.g. SetCloudPage:true"},
        {"SetCloudParam", "", "e.g. SetCloudParam:domainX,500"},
        {"SetCloudRunning", "", "e.g. SetCloudRunning:false"},
        {"SetCloudSource", "", "e.g. SetCloudSource:0,250,250,200,100,1.0,200,0,10"},
        {"SetCloudWind", "", "e.g. SetCloudWind:2,0,0"},
        {"SetCouplingEnabled", "", "e.g. SetCouplingEnabled:true"},
        {"SetCouplingMode", "", "e.g. SetCouplingMode:OneWay"},
        {"SetCrossBodyCollisionEnabled", "", "e.g. SetCrossBodyCollisionEnabled:false"},
        {"SetEnvironment", "", "e.g. SetEnvironment:${repo_root}/Physics/PhysicsView/scenarios/models/env"},
        {"SetFlameBodyParam", "", "e.g. SetFlameBodyParam:0,pyrolysisTemperature,700"},
        {"SetFlameBodyPose", "", "e.g. SetFlameBodyPose:1,0,0,0,1,0,0,0"},
        {"SetFlamePage", "", "e.g. SetFlamePage:true"},
        {"SetFlameParam", "", "e.g. SetFlameParam:buoyancyCoe,0"},
        {"SetFlameRenderMode", "", "e.g. SetFlameRenderMode:Fancy"},
        {"SetFlameRenderParam", "", "e.g. SetFlameRenderParam:carrierDebug,2"},
        {"SetFlameRigidVelocity", "", "e.g. SetFlameRigidVelocity:1,nan,0,0,0,0,0"},
        {"SetFlameRunning", "", "e.g. SetFlameRunning:false"},
        {"SetFluidBoundary", "", "e.g. SetFluidBoundary:-8:-8:-8:8:14:8"},
        {"SetFluidBounds", "", "e.g. SetFluidBounds:-2:8:-2:2:12:2"},
        {"SetFluidDensity", "", "e.g. SetFluidDensity:1.0"},
        {"SetFluidEffectLength", "", "e.g. SetFluidEffectLength:1.0"},
        {"SetFluidGravity", "", "e.g. SetFluidGravity:0:0:0"},
        {"SetFluidPressureCoeScale", "", "e.g. SetFluidPressureCoeScale:1960.0"},
        {"SetFluidRadius", "", "e.g. SetFluidRadius:0.15"},
        {"SetFluidStiffness", "", "e.g. SetFluidStiffness:0.001"},
        {"SetFluidTension", "", "e.g. SetFluidTension:0.0"},
        {"SetFluidTimeStep", "", "e.g. SetFluidTimeStep:0.005"},
        {"SetFluidViscosity", "", "e.g. SetFluidViscosity:0.0"},
        {"SetGravity", "", "e.g. SetGravity:0:0:0"},
        {"SetLight", "", "e.g. SetLight:-0.3,-1,-0.25,1,1,1,3"},
        {"SetMeshRenderEnabled", "", "e.g. SetMeshRenderEnabled:true"},
        {"SetPreset", "", "e.g. SetPreset:SphereDrop"},
        {"SetRenderBackgroundTransform", "", "e.g. SetRenderBackgroundTransform:0,0,0,0,0,0,1"},
        {"SetRenderUseIBL", "", "e.g. SetRenderUseIBL:1"},
        {"SetRigidRenderMode", "", "e.g. SetRigidRenderMode:shaded"},
        {"SetRunning", "", "e.g. SetRunning:true"},
        {"SetSSFRAnisotropicKernel", "", "e.g. SetSSFRAnisotropicKernel:1"},
        {"SetSSFREnabled", "", "e.g. SetSSFREnabled:true"},
        {"SetSSFRKernelParam", "", "e.g. SetSSFRKernelParam:maxRatio=1"},
        {"SetSSFRMode", "", "e.g. SetSSFRMode:3"},
        {"SetSelfCollision", "", "e.g. SetSelfCollision:false"},
        {"SetShadowEnabled", "", "e.g. SetShadowEnabled:0"},
        {"SetSimulationType", "", "e.g. SetSimulationType:DFSPH"},
        {"SetSoftFluidCouplingEnabled", "", "e.g. SetSoftFluidCouplingEnabled:false"},
        {"SetSoftRenderMode", "", "e.g. SetSoftRenderMode:shaded"},
        {"SetSolverIter", "", "e.g. SetSolverIter:2"},
        {"SetTimeStep", "", "e.g. SetTimeStep:0.02"},
        {"SetWhiteWaterParam", "", "e.g. SetWhiteWaterParam:foamDrag,0.9"},
        {"GetWhiteWaterParam", "", "e.g. GetWhiteWaterParam:foamDrag"},
        {"SetBaumgarteBeta", "", "e.g. SetBaumgarteBeta:0.2"},
        {"GetBaumgarteBeta", "", ""},
        {"GetSoftParam", "", "e.g. GetSoftParam:numIterations"},
        {"SetSoftParam", "", "e.g. SetSoftParam:numIterations,8"},
        {"SetUIVisible", "", "e.g. SetUIVisible:false"},
        {"SetVolumeCellLength", "", "e.g. SetVolumeCellLength:0.05"},
        {"SetVolumeIsoLevel", "", "e.g. SetVolumeIsoLevel:0.5"},
        {"SetVolumeKernel", "", "e.g. SetVolumeKernel:Isotropic"},
        {"SetVolumeParticleRadius", "", "e.g. SetVolumeParticleRadius:0.025"},
        {"SetVolumeRenderEnabled", "", "e.g. SetVolumeRenderEnabled:true"},
        {"Step", "", ""},
        {"StepSimulation", "", "e.g. StepSimulation:300 (same physics as repeated Step)"},
        {"StopFlameSource", "", ""},
        {"UnbindFlameBody", "", "e.g. UnbindFlameBody:1"},
    };
}

// ---- processQueue (render thread) ---------------------------------------

void CommandDispatcher::stepSimulation() {
    world_->stepOnce();
    // Coupled fluid stepping already advances the soft world exactly once.
    if (softWorld_ && !world_->isSoftCouplingEnabled()) softWorld_->stepForced();
}

void CommandDispatcher::processQueue() {
    // A FlameStep:<n> is spread over frames (see FlameCommandDispatcher::tick); its
    // answer is pushed here when the last step is done.
    flameDispatcher_.setDeferredDone([this](const std::string& r) { queue_.respond(r); });
    flameDispatcher_.tick(simulationBudgetMs_);
    if (simulationStepsRemaining_ > 0) {
        const auto start = std::chrono::steady_clock::now();
        do {
            stepSimulation();
            --simulationStepsRemaining_;
        } while (simulationStepsRemaining_ > 0 &&
            std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count() < simulationBudgetMs_);
        if (onWorldChanged_) onWorldChanged_();
        if (simulationStepsRemaining_ == 0) queue_.respond("OK");
    }

    std::queue<std::string> local = queue_.takeAll();
    while (!local.empty()) {
        if (flameDispatcher_.busy() || simulationStepsRemaining_ > 0) {
            // Keep command order: hold back everything behind the running FlameStep.
            queue_.requeueFront(local);
            break;
        }
        std::string cmd = std::move(local.front());
        local.pop();
        const bool uiCmd = takeUiMark(cmd);   // GUI-issued: no one reads the response

        auto resp = route(cmd);
        if (resp && resp->empty()) continue;   // deferred answer (FlameStep:<n>)
        if (!resp) {
            // Not a fluid command: delegate to the rigid-body command surface
            // (SetPreset, AddSphere, GetBodyCount, SaveScreenshot, ...).
            rigidDispatcher_.dispatch(cmd);
            rigidDispatcher_.processQueue();
            auto rigidResps = rigidDispatcher_.collectResponses();
            if (!rigidResps.empty()) {
                resp = std::move(rigidResps.front());
            } else if (cmd.rfind("SaveScreenshot:", 0) == 0) {
                // Legitimately silent: the response arrives later via
                // takePendingScreenshot()/signalScreenshotDone().
                continue;
            } else {
                resp = "Error:unknown command '" + cmd + "'";
            }
        }

        if (!uiCmd) queue_.respond(std::move(*resp));
    }
}

// ---- route --------------------------------------------------------------

std::optional<std::string> CommandDispatcher::route(const std::string& cmd) {
    if (!world_) return std::string("Error:world not set");

    const std::string_view sv(cmd);

    if (auto flameResp = flameDispatcher_.route(cmd)) return flameResp;
    if (auto cloudResp = cloudDispatcher_.route(cmd)) return cloudResp;
    if (auto hairResp = hairDispatcher_.route(cmd)) return hairResp;

    if (sv.rfind("SetCameraTarget:",0)==0) {
        if(!fluidRenderer_) return std::string("Error:camera not available");
        const auto parts=split(sv.substr(16),','); float x,y,z;
        if(parts.size()!=3 || !parseFlt(parts[0],x) || !parseFlt(parts[1],y) || !parseFlt(parts[2],z) ||
            !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return std::string("Error:expected x,y,z");
        fluidRenderer_->setCameraTarget({x,y,z}); return std::string("OK");
    }
    if (sv.rfind("SetCameraOrbit:", 0) == 0) {
        if (!fluidRenderer_) return std::string("Error:camera not available");
        const auto parts = split(sv.substr(15), ',');
        float d = 0.0f, yaw = 0.0f, pitch = 0.0f;
        if (parts.size() != 3 || !parseFlt(parts[0], d) || !parseFlt(parts[1], yaw) || !parseFlt(parts[2], pitch) ||
            !std::isfinite(d) || !std::isfinite(yaw) || !std::isfinite(pitch) || d <= 0.0f) {
            return std::string("Error:expected distance,yaw,pitch");
        }
        fluidRenderer_->setCameraOrbit(d, yaw, pitch);
        return std::string("OK");
    }
    if (cmd == "GetCameraOrbit") {
        if (!fluidRenderer_) return std::string("Error:camera not available");
        return std::to_string(fluidRenderer_->getCameraDistance()) + "," +
               std::to_string(fluidRenderer_->getCameraYaw()) + "," +
               std::to_string(fluidRenderer_->getCameraPitch());
    }
    if (cmd == "SetUIVisible:true" || cmd == "SetUIVisible:false") {
        if (!setUIVisible_) return std::string("Error:UI visibility hook not set");
        setUIVisible_(cmd == "SetUIVisible:true");
        return std::string("OK");
    }
    if (cmd == "IsUIVisible") {
        if (!isUIVisible_) return std::string("Error:UI visibility hook not set");
        return std::string(isUIVisible_() ? "true" : "false");
    }
    if (cmd == "NewScene" || cmd == "New") {
        if (!onNewScene_) return std::string("Error:new-scene handler not set");
        onNewScene_();
        if (onWorldChanged_) onWorldChanged_();
        return std::string("OK");
    }
    if (cmd == "Reset") {
        world_->reset();
        if (rigidWorld_) rigidWorld_->reset();
        if (softWorld_) softWorld_->reset();
        world_->refreshCoupling();      // rebind (new bodies from rigid reset)
        world_->refreshSoftCoupling();  // rebind (new bodies from soft reset)
        if (onWorldChanged_) onWorldChanged_();
        return std::string("OK");
    }
    if (sv.rfind("StepSimulation:", 0) == 0) {
        int n;
        const auto count = sv.substr(15);
        const auto [end, error] = std::from_chars(count.data(), count.data()+count.size(), n);
        if (error != std::errc{} || end != count.data()+count.size() || n < 0) return "Error:bad step count";
        if (n == 0) return "OK";
        simulationStepsRemaining_ = n;
        return std::string(); // deferred response, after every requested step
    }
    if (cmd == "Step") {
        stepSimulation();
        if (onWorldChanged_) onWorldChanged_();
        return std::string("OK");
    }
    if (sv.rfind("Step:", 0) == 0) {
        // Rigid-body stepping for this pattern is handled by the embedded
        // RigidBodyCommandDispatcher via the processQueue() fallback below
        // (nullopt keeps that fallthrough unchanged); only soft-body
        // stepping is added here, alongside it, with no coupling between
        // the two (see internal design notes;
        // fluid+soft coupled stepping goes through the plain "Step" command
        // instead, repeated via a scenario JSON's "repeat" field).
        // stepForced() (not step()) is used so this actually advances the
        // simulation regardless of SetRunning: state.
        if (softWorld_) {
            int n = 1;
            parseInt(sv.substr(5), n);
            for (int i = 0; i < n; ++i) softWorld_->stepForced();
        }
        return std::nullopt;
    }
    if (sv.rfind("SetRunning:", 0) == 0) {
        // Shared verb across fluid, rigid-body and soft-body scenarios;
        // applied to all three worlds directly (bypassing the sub-
        // dispatchers) since they don't interact. IsRunning (below) queries
        // world_ (fluid) alone, so it must be kept in sync here too.
        std::string_view val = sv.substr(11);
        const bool running = (val == "true" || val == "1");
        world_->setRunning(running);
        if (rigidWorld_) rigidWorld_->setRunning(running);
        if (softWorld_)  softWorld_->setRunning(running);
        return std::string("OK");
    }
    if (sv.rfind("SetPreset:", 0) == 0) {
        std::string_view name = sv.substr(10);
        if (isSoftBodyPresetName(name)) {
            if (!softWorld_) return std::string("Error:soft world not set");
            softDispatcher_.dispatch(cmd);
            softDispatcher_.processQueue();
            auto resps = softDispatcher_.collectResponses();
            return resps.empty() ? std::string("Error:no response") : resps.front();
        }
        return std::nullopt;  // rigid-body preset name: fall through as before
    }
    if (cmd == "GetSoftBodyCount" || cmd == "GetSoftParticleCount" ||
        cmd == "GetMaxSpeed" || sv.rfind("SetSelfCollision:", 0) == 0 ||
        sv.rfind("GetSoftParticlePositionY:", 0) == 0 ||
        sv.rfind("GetSoftTotalVolume:", 0) == 0 ||
        sv.rfind("GetSoftMinInterBodyDistance:", 0) == 0 ||
        sv.rfind("GetSoftMinNonEdgeDistance:", 0) == 0 ||
        sv.rfind("SetSoftParam:", 0) == 0 || sv.rfind("GetSoftParam:", 0) == 0 ||
        sv.rfind("SetCrossBodyCollisionEnabled:", 0) == 0) {
        if (!softWorld_) return std::string("Error:soft world not set");
        softDispatcher_.dispatch(cmd);
        softDispatcher_.processQueue();
        auto resps = softDispatcher_.collectResponses();
        return resps.empty() ? std::string("Error:no response") : resps.front();
    }
    if (cmd == "GetParticleCount") {
        return "Count:" + std::to_string(world_->getParticleCount());
    }
    if (cmd == "IsRunning") {
        return world_->isRunning() ? std::string("OK") : std::string("No");
    }

    if (cmd == "SetCouplingEnabled:true" || cmd == "SetCouplingEnabled:false") {
        if (cmd == "SetCouplingEnabled:true" && world_->rigid().hasExternalClock())
            return "Error:rigid world uses the Flame clock; unbind Flame bodies first";
        world_->setCouplingEnabled(cmd == "SetCouplingEnabled:true");
        return std::string("OK");
    }
    if (cmd == "SetCouplingMode:OneWay" || cmd == "SetCouplingMode:TwoWay") {
        world_->setCouplingMode(cmd == "SetCouplingMode:TwoWay"
            ? Phantom::Physics::CouplingMode::TwoWay
            : Phantom::Physics::CouplingMode::OneWay);
        return std::string("OK");
    }
    if (cmd == "IsCouplingEnabled") {
        return world_->isCouplingEnabled() ? std::string("OK") : std::string("No");
    }

    if (cmd == "SetSoftFluidCouplingEnabled:true" || cmd == "SetSoftFluidCouplingEnabled:false") {
        world_->setSoftCouplingEnabled(cmd == "SetSoftFluidCouplingEnabled:true");
        return std::string("OK");
    }
    if (cmd == "IsSoftFluidCouplingEnabled") {
        return world_->isSoftCouplingEnabled() ? std::string("OK") : std::string("No");
    }

    // ---- static mesh fluid boundary (One-Way SDF, Physics::MeshBoundaryShape) ----
    if (sv.rfind("LoadMeshBoundary:", 0) == 0) {
        if (!world_->loadMeshBoundary(std::string(sv.substr(17))))
            return std::string("Error:failed to load mesh boundary");
        return std::string("OK");
    }
    if (cmd == "ClearMeshBoundary") {
        world_->clearMeshBoundary();
        return std::string("OK");
    }
    if (cmd == "GetMeshBoundaryTriangleCount") {
        return std::to_string(world_->getMeshBoundaryTriangleCount());
    }

    // ---- Emitter (continuous particle generation, internal design notes) ----
    // "AddEmitter:cx,cy,cz,radius,rate,dirX,dirY,dirZ,speed" -- speedJitter
    // isn't exposed here (scenario-only surface for demo setup, mirrors the
    // other Set* commands above); tune it via the fluid's own addEmitter()
    // for finer control. particleRadius also isn't exposed -- unlike
    // speedJitter this isn't just an omitted knob, FluidWorld::addEmitter()
    // always forces it to match params().radius regardless of what's passed,
    // so spawned particles can't destabilize the solver with a mismatched
    // mass (see Emitter::particleRadius's doc comment).
    if (sv.rfind("AddEmitter:", 0) == 0) {
        const auto parts = split(sv.substr(11), ',');
        if (parts.size() != 9) return std::string("Error:AddEmitter needs 9 comma-separated values");
        float vals[9];
        for (size_t i = 0; i < 9; ++i) {
            if (!parseFlt(parts[i], vals[i])) return std::string("Error:bad float");
        }
        Phantom::Physics::Emitter e;
        e.center    = Phantom::Math::Vector3df(vals[0], vals[1], vals[2]);
        e.radius    = vals[3];
        e.rate      = vals[4];
        e.direction = Phantom::Math::Vector3df(vals[5], vals[6], vals[7]);
        e.speed     = vals[8];
        world_->addEmitter(e);
        return std::string("OK");
    }
    if (cmd == "ClearEmitters") {
        world_->clearEmitters();
        return std::string("OK");
    }
    if (cmd == "GetEmitterCount") {
        return std::to_string(world_->getEmitters().size());
    }

    // ---- Outflow region (optional particle removal; deletion counterpart
    // to the emitter commands above) ----
    // "AddOutflowRegion:minX,minY,minZ,maxX,maxY,maxZ"
    if (sv.rfind("AddOutflowRegion:", 0) == 0) {
        const auto parts = split(sv.substr(17), ',');
        if (parts.size() != 6) return std::string("Error:AddOutflowRegion needs 6 comma-separated values");
        float vals[6];
        for (size_t i = 0; i < 6; ++i) {
            if (!parseFlt(parts[i], vals[i])) return std::string("Error:bad float");
        }
        Phantom::Physics::OutflowRegion r;
        r.bounds = Phantom::Math::Box3df(
            Phantom::Math::Vector3df(vals[0], vals[1], vals[2]),
            Phantom::Math::Vector3df(vals[3], vals[4], vals[5]));
        world_->addOutflowRegion(r);
        return std::string("OK");
    }
    if (cmd == "ClearOutflowRegions") {
        world_->clearOutflowRegions();
        return std::string("OK");
    }
    if (cmd == "GetOutflowRegionCount") {
        return std::to_string(world_->getOutflowRegions().size());
    }
    if (cmd == "GetSoftMinPositionY") {
        if (!softWorld_) return std::string("Error:soft world not set");
        return std::to_string(softWorld_->getMinParticlePositionY());
    }
    if (cmd == "GetSoftMaxPositionY") {
        if (!softWorld_) return std::string("Error:soft world not set");
        return std::to_string(softWorld_->getMaxParticlePositionY());
    }

    // ---- fluid parameter setters (scenario-only surface for demo setup:
    // must be called before "Reset" so they take effect on world_->reset()) ----

    if (sv.rfind("SetSimulationType:", 0) == 0) {
        std::string_view name = sv.substr(18);
        FluidWorld::SimulationType t = FluidWorld::SimulationType::DFSPH;
        if      (name == "DFSPH")    t = FluidWorld::SimulationType::DFSPH;
        else if (name == "PBSPH")    t = FluidWorld::SimulationType::PBSPH;
        else if (name == "WCSPH" || name == "CSPH") t = FluidWorld::SimulationType::WCSPH;
        else if (name == "GPU_CSPH") t = FluidWorld::SimulationType::GPU_CSPH;
        else return std::string("Error:unknown simulation type '") + std::string(name) + "'";
        world_->setSimulationType(t);
        return std::string("OK");
    }
    if (sv.rfind("SetFluidEffectLength:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(21), v)) return std::string("Error:bad float");
        world_->params().effectLength = v;
        return std::string("OK");
    }
    if (sv.rfind("SetFluidStiffness:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(18), v)) return std::string("Error:bad float");
        world_->params().stiffness = v;
        return std::string("OK");
    }
    // Scale-invariant counterpart to SetFluidStiffness for WCSPH/DFSPH (see
    // FluidWorld::createWCSPH()/createDFSPH()/internal design notes):
    // both ignore params().stiffness and always derive their pressureCoe as
    // pressureCoeScale * effectLength instead.
    if (sv.rfind("SetFluidPressureCoeScale:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(25), v)) return std::string("Error:bad float");
        world_->params().pressureCoeScale = v;
        return std::string("OK");
    }
    // How much of a particle's wall-normal velocity the domain walls absorb on
    // contact (ISPHSolver::setBoundaryDampingRatio(); WCSPH/DFSPH only). 0 --
    // the default -- is the historical undamped penalty spring, which bounces
    // at restitution ~1 and turns a jet landing in an empty container into
    // spray. Takes effect on the next Reset, like the other params() setters.
    if (sv.rfind("SetFluidBoundaryDamping:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(24), v)) return std::string("Error:bad float");
        world_->params().boundaryDampingRatio = v;
        return std::string("OK");
    }
    if (sv.rfind("SetFluidDensity:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(16), v)) return std::string("Error:bad float");
        world_->params().density = v;
        return std::string("OK");
    }
    if (sv.rfind("SetFluidViscosity:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(18), v)) return std::string("Error:bad float");
        world_->params().viscosity = v;
        return std::string("OK");
    }
    // White water (spray / foam) parameters; read live by the simulation.
    if (sv.rfind("SetWhiteWaterParam:", 0) == 0) {
        const auto parts = split(sv.substr(19), ',');
        float v = 0.0f;
        if (parts.size() != 2 || !parseFlt(parts[1], v)) return std::string("Error:expected name,value");
        auto& w = world_->whiteWaterParams();
        const std::string_view n = parts[0];
        if      (n == "sprayVelThreshold")  w.sprayVelThreshold = v;
        else if (n == "sprayDensityRatio")  w.sprayDensityRatio = v;
        else if (n == "foamCurvThreshold")  w.foamCurvThreshold = v;
        else if (n == "foamNeighborCount")  w.foamNeighborCount = v;
        else if (n == "maxSprayParticles")  w.maxSprayParticles = static_cast<int>(v);
        else if (n == "maxFoamParticles")   w.maxFoamParticles = static_cast<int>(v);
        else if (n == "foamBuoyancy")       w.foamBuoyancy = v;
        else if (n == "foamDrag")           w.foamDrag = v;
        else return std::string("Error:unknown white water param '") + std::string(n) + "'";
        return std::string("OK");
    }
    if (sv.rfind("GetWhiteWaterParam:", 0) == 0) {
        const std::string_view n = sv.substr(19);
        const auto& w = world_->whiteWaterParams();
        float v;
        if      (n == "sprayVelThreshold")  v = w.sprayVelThreshold;
        else if (n == "sprayDensityRatio")  v = w.sprayDensityRatio;
        else if (n == "foamCurvThreshold")  v = w.foamCurvThreshold;
        else if (n == "foamNeighborCount")  v = w.foamNeighborCount;
        else if (n == "maxSprayParticles")  v = static_cast<float>(w.maxSprayParticles);
        else if (n == "maxFoamParticles")   v = static_cast<float>(w.maxFoamParticles);
        else if (n == "foamBuoyancy")       v = w.foamBuoyancy;
        else if (n == "foamDrag")           v = w.foamDrag;
        else return std::string("Error:unknown white water param '") + std::string(n) + "'";
        return std::to_string(v);
    }
    if (sv.rfind("SetFluidTension:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(16), v)) return std::string("Error:bad float");
        world_->params().tension = v;
        return std::string("OK");
    }
    if (sv.rfind("SetFluidRadius:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(15), v)) return std::string("Error:bad float");
        world_->params().radius = v;
        return std::string("OK");
    }
    if (sv.rfind("SetFluidTimeStep:", 0) == 0) {
        float v;
        if (!parseFlt(sv.substr(17), v)) return std::string("Error:bad float");
        world_->params().timeStep = v;
        return std::string("OK");
    }
    if (sv.rfind("SetFluidGravity:", 0) == 0) {
        auto parts = split(sv.substr(16), ':');
        float x, y, z;
        if (parts.size() < 3 ||
            !parseFlt(parts[0], x) || !parseFlt(parts[1], y) || !parseFlt(parts[2], z)) {
            return std::string("Error:bad SetFluidGravity args");
        }
        world_->params().gravity = { x, y, z };
        return std::string("OK");
    }
    if (sv.rfind("SetFluidBounds:", 0) == 0) {
        auto parts = split(sv.substr(15), ':');
        float xmin, ymin, zmin, xmax, ymax, zmax;
        if (parts.size() < 6 ||
            !parseFlt(parts[0], xmin) || !parseFlt(parts[1], ymin) || !parseFlt(parts[2], zmin) ||
            !parseFlt(parts[3], xmax) || !parseFlt(parts[4], ymax) || !parseFlt(parts[5], zmax)) {
            return std::string("Error:bad SetFluidBounds args");
        }
        world_->params().fluidBounds = Phantom::Math::Box3df(
            Phantom::Math::Vector3df(xmin, ymin, zmin),
            Phantom::Math::Vector3df(xmax, ymax, zmax));
        return std::string("OK");
    }
    if (sv.rfind("SetFluidBoundary:", 0) == 0) {
        auto parts = split(sv.substr(17), ':');
        float xmin, ymin, zmin, xmax, ymax, zmax;
        if (parts.size() < 6 ||
            !parseFlt(parts[0], xmin) || !parseFlt(parts[1], ymin) || !parseFlt(parts[2], zmin) ||
            !parseFlt(parts[3], xmax) || !parseFlt(parts[4], ymax) || !parseFlt(parts[5], zmax)) {
            return std::string("Error:bad SetFluidBoundary args");
        }
        world_->params().boundary = Phantom::Math::Box3df(
            Phantom::Math::Vector3df(xmin, ymin, zmin),
            Phantom::Math::Vector3df(xmax, ymax, zmax));
        return std::string("OK");
    }
    if (cmd == "GetMaxParticlePositionY") {
        const auto positions = world_->getParticlePositions();
        if (positions.empty()) return std::string("0");
        float maxY = positions.front().y;
        for (const auto& p : positions) maxY = std::max(maxY, p.y);
        return std::to_string(maxY);
    }
    if (cmd == "GetMinParticlePositionY") {
        const auto positions = world_->getParticlePositions();
        if (positions.empty()) return std::string("0");
        float minY = positions.front().y;
        for (const auto& p : positions) minY = std::min(minY, p.y);
        return std::to_string(minY);
    }
    // Distinguishes "settled" from "collapsed to a point" -- min/max alone
    // can't tell those apart (internal design notes
    // Phase 0 item 3).
    if (cmd == "GetAvgParticlePositionY") {
        const auto positions = world_->getParticlePositions();
        if (positions.empty()) return std::string("0");
        float sumY = 0.0f;
        for (const auto& p : positions) sumY += p.y;
        return std::to_string(sumY / static_cast<float>(positions.size()));
    }
    // Fluid-side counterpart to soft-body's GetMaxSpeed -- an instability
    // (NaN/explosion) detector that doesn't require the particle set to
    // still be inside any particular container box.
    if (cmd == "GetMaxParticleSpeed") {
        const auto velocities = world_->getParticleVelocities();
        if (velocities.empty()) return std::string("0");
        float maxSpeed = 0.0f;
        for (const auto& v : velocities) {
            const float speed = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
            maxSpeed = std::max(maxSpeed, speed);
        }
        return std::to_string(maxSpeed);
    }
    if (cmd == "GetSimulationType") {
        switch (world_->getSimulationType()) {
        case FluidWorld::SimulationType::DFSPH:    return std::string("DFSPH");
        case FluidWorld::SimulationType::PBSPH:    return std::string("PBSPH");
        case FluidWorld::SimulationType::WCSPH:    return std::string("WCSPH");
        case FluidWorld::SimulationType::GPU_CSPH: return std::string("GPU_CSPH");
        }
        return std::string("Error:unknown simulation type");
    }
    // Reports the *effective* coupling mode (see FluidWorld::activeCouplingMode()'s
    // doc comment) rather than the requested one, so a TwoWay request that
    // silently degrades to OneWay (solver doesn't supportTwoWayCoupling())
    // is actually observable from a scenario instead of passing unnoticed.
    if (cmd == "GetActiveCouplingMode") {
        return world_->activeCouplingMode() == Phantom::Physics::CouplingMode::TwoWay
            ? std::string("TwoWay") : std::string("OneWay");
    }

    // ---- fluid particles -> SparseVolume conversion (FluidVolumeConverter) ----

    if (sv.rfind("SetVolumeParticleRadius:", 0) == 0) {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        float v;
        if (!parseFlt(sv.substr(24), v)) return std::string("Error:bad float");
        volumeConverter_->params().particleRadius = v;
        return std::string("OK");
    }
    if (sv.rfind("SetVolumeCellLength:", 0) == 0) {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        float v;
        if (!parseFlt(sv.substr(20), v)) return std::string("Error:bad float");
        volumeConverter_->params().cellLength = v;
        return std::string("OK");
    }
    if (cmd == "SetVolumeKernel:Isotropic" || cmd == "SetVolumeKernel:Anisotropic") {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        volumeConverter_->params().kernelType = (cmd == "SetVolumeKernel:Anisotropic")
            ? Phantom::FluidVolumeConverter::KernelType::Anisotropic
            : Phantom::FluidVolumeConverter::KernelType::Isotropic;
        return std::string("OK");
    }
    if (cmd == "ConvertToVolume") {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        if (!volumeConverter_->convert(world_->getParticlePositions()))
            return std::string("Error:") + volumeConverter_->lastError();
        if (onVolumeChanged_) onVolumeChanged_();
        return std::string("OK");
    }
    if (cmd == "GetVolumeVoxelCount") {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        return "Count:" + std::to_string(volumeConverter_->getVoxelCount());
    }
    if (cmd == "SetVolumeRenderEnabled:true" || cmd == "SetVolumeRenderEnabled:false") {
        if (!volumeRenderer_) return std::string("Error:volume renderer not set");
        volumeRenderer_->setEnabled(cmd == "SetVolumeRenderEnabled:true");
        return std::string("OK");
    }
    if (cmd == "IsVolumeRenderEnabled") {
        if (!volumeRenderer_) return std::string("Error:volume renderer not set");
        return volumeRenderer_->isEnabled() ? std::string("OK") : std::string("No");
    }

    // ---- SparseVolume -> Mesh conversion (FluidMeshConverter) ----

    if (sv.rfind("SetVolumeIsoLevel:", 0) == 0) {
        if (!meshConverter_) return std::string("Error:mesh converter not set");
        float v;
        if (!parseFlt(sv.substr(18), v)) return std::string("Error:bad float");
        meshConverter_->params().isoLevel = v;
        return std::string("OK");
    }
    if (cmd == "ConvertToMesh") {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        if (!meshConverter_) return std::string("Error:mesh converter not set");
        const auto* volume = volumeConverter_->getVolume();
        if (!volume) return std::string("Error:no volume -- call ConvertToVolume first");
        if (!meshConverter_->convert(*volume))
            return std::string("Error:") + meshConverter_->lastError();
        if (onMeshChanged_) onMeshChanged_();
        return std::string("OK");
    }
    if (cmd == "GetMeshTriangleCount") {
        if (!meshConverter_) return std::string("Error:mesh converter not set");
        return "Count:" + std::to_string(meshConverter_->getTriangleCount());
    }
    if (sv.rfind("SaveMeshToObj:", 0) == 0) {
        if (!meshConverter_) return std::string("Error:mesh converter not set");
        if (!meshConverter_->saveToObj(std::string(sv.substr(14))))
            return std::string("Error:") + meshConverter_->lastError();
        return std::string("OK");
    }
    if (cmd == "SetMeshRenderEnabled:true" || cmd == "SetMeshRenderEnabled:false") {
        if (!meshRenderer_) return std::string("Error:mesh renderer not set");
        meshRenderer_->setEnabled(cmd == "SetMeshRenderEnabled:true");
        return std::string("OK");
    }
    if (cmd == "IsMeshRenderEnabled") {
        if (!meshRenderer_) return std::string("Error:mesh renderer not set");
        return meshRenderer_->isEnabled() ? std::string("OK") : std::string("No");
    }

    // ---- SPH showcase porting surface (see CommandDispatcher.h's class doc) ----

    if (sv.rfind("AddBoundarySphere:", 0) == 0) {
        const auto parts = split(sv.substr(18), ',');
        if (parts.size() != 5) return std::string("Error:AddBoundarySphere needs 5 comma-separated values");
        float vals[5];
        for (size_t i = 0; i < 5; ++i) {
            if (!parseFlt(parts[i], vals[i])) return std::string("Error:bad float");
        }
        world_->addBoundarySphere(Phantom::Physics::SphereBoundary(
            Phantom::Math::Vector3df(vals[0], vals[1], vals[2]), vals[3], vals[4]));
        return std::string("OK");
    }
    if (cmd == "ClearBoundarySpheres") {
        world_->clearBoundarySpheres();
        return std::string("OK");
    }
    if (cmd == "GetBoundarySphereCount") {
        return std::to_string(world_->getBoundarySpheres().size());
    }

    if (sv.rfind("AddFluidSourceBox:", 0) == 0) {
        const auto parts = split(sv.substr(18), ',');
        if (parts.size() != 6) return std::string("Error:AddFluidSourceBox needs 6 comma-separated values");
        float vals[6];
        for (size_t i = 0; i < 6; ++i) {
            if (!parseFlt(parts[i], vals[i])) return std::string("Error:bad float");
        }
        world_->addFluidSourceBox(Phantom::Math::Box3df(
            Phantom::Math::Vector3df(vals[0], vals[1], vals[2]),
            Phantom::Math::Vector3df(vals[3], vals[4], vals[5])));
        return std::string("OK");
    }
    if (sv.rfind("AddFluidSourceSphere:", 0) == 0) {
        const auto parts = split(sv.substr(21), ',');
        if (parts.size() != 4) return std::string("Error:AddFluidSourceSphere needs 4 comma-separated values");
        float vals[4];
        for (size_t i = 0; i < 4; ++i) {
            if (!parseFlt(parts[i], vals[i])) return std::string("Error:bad float");
        }
        world_->addFluidSourceSphere(Phantom::Math::Vector3df(vals[0], vals[1], vals[2]), vals[3]);
        return std::string("OK");
    }
    if (cmd == "ClearFluidSources") {
        world_->clearFluidSources();
        return std::string("OK");
    }
    if (cmd == "GetFluidSourceRegionCount") {
        return std::to_string(world_->getFluidSourceRegionCount());
    }

    if (sv.rfind("SetFluidMaxParticles:", 0) == 0) {
        int v;
        if (!parseInt(sv.substr(21), v)) return std::string("Error:bad int");
        world_->params().maxParticles = v;
        return std::string("OK");
    }

    if (sv.rfind("SetPLYOutputDir:", 0) == 0) {
        plyOutputDir_ = std::filesystem::path(std::string(sv.substr(16)));
        std::error_code ec;
        std::filesystem::create_directories(plyOutputDir_, ec);
        if (ec) return std::string("Error:could not create directory '") + plyOutputDir_.string() + "'";
        plyFrameCounter_ = 1;
        return std::string("OK");
    }
    if (sv.rfind("SavePLY:", 0) == 0) {
        const std::filesystem::path path(std::string(sv.substr(8)));
        if (!writeFluidParticlesToPLY(path, world_->getParticlePositions()))
            return std::string("Error:could not write '") + path.string() + "'";
        return std::string("OK");
    }
    if (sv.rfind("StepFrameAndSavePLY:", 0) == 0) {
        if (plyOutputDir_.empty()) return std::string("Error:SetPLYOutputDir must be called first");
        int substeps = 1;
        if (!parseInt(sv.substr(20), substeps) || substeps < 1)
            return std::string("Error:bad substep count");
        for (int i = 0; i < substeps; ++i) {
            world_->stepOnce();
            if (softWorld_ && !world_->isSoftCouplingEnabled()) softWorld_->stepForced();
            if (onWorldChanged_) onWorldChanged_();
        }
        char name[32];
        std::snprintf(name, sizeof(name), "frame_%04d.ply", plyFrameCounter_);
        const std::filesystem::path path = plyOutputDir_ / name;
        if (!writeFluidParticlesToPLY(path, world_->getParticlePositions()))
            return std::string("Error:could not write '") + path.string() + "'";
        ++plyFrameCounter_;
        return std::string("OK");
    }

    // ---- Vulkan-native glTF background / environment / light
    // (docs/todo/PLAN_physicsview_gltf_rendering.md Phase 1). Mirrors
    // FluidStudio's VkFluidRenderer command set. ----

    if (sv.rfind("LoadRenderBackground:", 0) == 0) {
        if (!renderBg_) return std::string("Error:render background not available");
        const std::string path(sv.substr(21));
        if (path.empty()) return std::string("Error:path is empty");
        std::error_code ec;
        const std::string abs = std::filesystem::absolute(path, ec).string();
        if (!renderBg_->loadBackground(ec ? path : abs))
            return std::string("Error:failed to load glTF background");
        return std::string("OK");
    }
    if (cmd == "ClearRenderBackground") {
        if (!renderBg_) return std::string("Error:render background not available");
        renderBg_->clearBackground();
        return std::string("OK");
    }
    if (sv.rfind("SetRenderBackgroundTransform:", 0) == 0) {
        if (!renderBg_) return std::string("Error:render background not available");
        const auto parts = split(sv.substr(29), ',');
        if (parts.size() != 7)
            return std::string("Error:SetRenderBackgroundTransform needs 7 comma-separated values (px,py,pz,rx,ry,rz,s)");
        float v[7];
        for (size_t i = 0; i < 7; ++i)
            if (!parseFlt(parts[i], v[i])) return std::string("Error:bad float");
        if (v[6] <= 0.f) return std::string("Error:scale must be > 0");
        renderBg_->setTransform({ v[0], v[1], v[2] }, { v[3], v[4], v[5] }, v[6]);
        return std::string("OK");
    }
    if (sv.rfind("SetEnvironment:", 0) == 0) {
        if (!renderBg_) return std::string("Error:render background not available");
        const std::string dir(sv.substr(15));
        if (dir.empty()) return std::string("Error:dir is empty");
        std::error_code ec;
        const std::string abs = std::filesystem::absolute(dir, ec).string();
        if (!renderBg_->setEnvironment(ec ? dir : abs))
            return std::string("Error:failed to load environment cube map");
        return std::string("OK");
    }
    if (cmd == "ClearRenderEnvironment") {
        if (!renderBg_) return std::string("Error:render background not available");
        renderBg_->clearEnvironment();
        return std::string("OK");
    }
    if (sv.rfind("SetLight:", 0) == 0) {
        if (!renderBg_) return std::string("Error:render background not available");
        const auto parts = split(sv.substr(9), ',');
        if (parts.size() != 7)
            return std::string("Error:SetLight needs 7 comma-separated values (dx,dy,dz,r,g,b,intensity)");
        float v[7];
        for (size_t i = 0; i < 7; ++i)
            if (!parseFlt(parts[i], v[i])) return std::string("Error:bad float");
        const glm::vec3 dir(v[0], v[1], v[2]);
        if (glm::length(dir) < 1.0e-6f) return std::string("Error:light direction is degenerate");
        if (v[3] < 0.f || v[4] < 0.f || v[5] < 0.f) return std::string("Error:light colour must be >= 0");
        if (v[6] < 0.f) return std::string("Error:light intensity must be >= 0");
        renderBg_->setLight(dir, { v[3], v[4], v[5] }, v[6]);
        return std::string("OK");
    }
    if (cmd == "SetRenderUseIBL:0" || cmd == "SetRenderUseIBL:1") {
        if (!renderBg_) return std::string("Error:render background not available");
        renderBg_->setUseIBL(cmd == "SetRenderUseIBL:1");
        return std::string("OK");
    }
    if (cmd == "SetShadowEnabled:0" || cmd == "SetShadowEnabled:1") {
        if (!renderBg_) return std::string("Error:render background not available");
        renderBg_->setCastShadows(cmd == "SetShadowEnabled:1");
        return std::string("OK");
    }
    if (cmd == "GetShadowEnabled") {
        if (!renderBg_) return std::string("Error:render background not available");
        return renderBg_->castShadows() ? std::string("1") : std::string("0");
    }
    if (cmd == "GetRenderSceneState") {
        if (!renderBg_) return std::string("{}");
        return renderBg_->sceneStateJson();
    }

    // Screen-space fluid rendering (Phase 5). Off = the SSFR composite still runs
    // as the HDR scene's final ACES tonemap pass (pure passthrough); on = the
    // fluid-surface reconstruction pre-passes run and the surface is composited
    // over the scene with depth occlusion + refraction.
    if (cmd == "SetSSFREnabled:true" || cmd == "SetSSFREnabled:false") {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        ssfrPanel_->setEnabled(cmd == "SetSSFREnabled:true");
        return std::string("OK");
    }
    if (cmd == "IsSSFREnabled") {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        return ssfrPanel_->isEnabled() ? std::string("1") : std::string("0");
    }
    if (sv.rfind("SetSSFRMode:", 0) == 0) {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        int idx = 0;
        auto s = sv.substr(12);
        auto [p, ec] = std::from_chars(s.data(), s.data() + s.size(), idx);
        if (ec != std::errc() || idx < 0 || idx > 5)
            return std::string("Error:SSFR mode index must be 0..5");
        ssfrPanel_->setModeIndex(idx);
        return std::string("OK");
    }

    // Anisotropic kernel (Yu & Turk ellipsoid splats).
    if (sv.rfind("SetSSFRAnisotropicKernel:", 0) == 0) {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        const auto v = sv.substr(25);
        if (v != "0" && v != "1" && v != "true" && v != "false")
            return std::string("Error:SetSSFRAnisotropicKernel expects 0|1");
        ssfrPanel_->setAnisotropicKernel(v == "1" || v == "true");
        return std::string("OK");
    }
    if (cmd == "GetSSFRAnisotropicKernel") {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        return ssfrPanel_->getAnisotropicKernel() ? std::string("1") : std::string("0");
    }
    if (sv.rfind("SetSSFRKernelParam:", 0) == 0) {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        const auto kv = sv.substr(19);
        const auto eq = kv.find('=');
        if (eq == std::string_view::npos) return std::string("Error:expected <key>=<value>");
        const auto* spec = findKernelParam(kv.substr(0, eq));
        if (!spec) return std::string("Error:unknown kernel param '") + std::string(kv.substr(0, eq)) + "'";
        float value = 0.0f;
        if (!parseFlt(kv.substr(eq + 1), value)) return std::string("Error:invalid value");
        if (!(value >= spec->lo && value <= spec->hi))
            return std::string("Error:") + spec->name + " out of range [" + std::to_string(spec->lo) +
                   ", " + std::to_string(spec->hi) + "]";
        auto& settings = ssfrPanel_->kernelSettings();
        if (spec->field) settings.*(spec->field) = value;
        else             settings.minNeighbors = static_cast<int>(value);
        ssfrPanel_->markKernelChanged();
        return std::string("OK");
    }
    if (sv.rfind("GetSSFRKernelParam:", 0) == 0) {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        const auto* spec = findKernelParam(sv.substr(19));
        if (!spec) return std::string("Error:unknown kernel param '") + std::string(sv.substr(19)) + "'";
        const auto& settings = ssfrPanel_->kernelSettings();
        return spec->field ? std::to_string(settings.*(spec->field))
                           : std::to_string(settings.minNeighbors);
    }
    if (sv.rfind("GetSSFRKernelStat:", 0) == 0) {
        if (!ssfrPanel_) return std::string("Error:SSFR panel not available");
        const auto key = sv.substr(18);
        const auto& st = ssfrPanel_->kernelStats();
        if (key == "computeMs")        return std::to_string(st.computeMs);
        if (key == "meanStretchRatio") return std::to_string(st.meanStretchRatio);
        if (key == "particleCount")    return std::to_string(st.particleCount);
        if (key == "anisotropicCount") return std::to_string(st.anisotropicCount);
        if (key == "active")           return ssfrPanel_->isAnisotropicKernelActive() ? std::string("1") : std::string("0");
        return std::string("Error:unknown kernel stat '") + std::string(key) + "'";
    }

    if (sv.rfind("SetRigidRenderMode:", 0) == 0) {
        if (!rigidBodyRenderer_) return std::string("Error:rigid body renderer not available");
        BodyRenderMode m;
        if (!parseBodyRenderMode(std::string(sv.substr(19)), m))
            return std::string("Error:mode must be wire|shaded|both");
        rigidBodyRenderer_->setMode(m);
        return std::string("OK");
    }
    if (cmd == "GetRigidRenderMode") {
        if (!rigidBodyRenderer_) return std::string("Error:rigid body renderer not available");
        return std::string(bodyRenderModeName(rigidBodyRenderer_->mode()));
    }

    if (sv.rfind("SetSoftRenderMode:", 0) == 0) {
        if (!softBodyRenderer_) return std::string("Error:soft body renderer not available");
        BodyRenderMode m;
        if (!parseBodyRenderMode(std::string(sv.substr(18)), m))
            return std::string("Error:mode must be wire|shaded|both");
        softBodyRenderer_->setMode(m);
        return std::string("OK");
    }
    if (cmd == "GetSoftRenderMode") {
        if (!softBodyRenderer_) return std::string("Error:soft body renderer not available");
        return std::string(bodyRenderModeName(softBodyRenderer_->mode()));
    }

    return std::nullopt;
}
