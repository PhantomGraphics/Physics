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

#include <algorithm>
#include <charconv>
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
    std::lock_guard<std::mutex> lk(mutex_);
    inputQueue_.push(command);
}

std::vector<std::string> CommandDispatcher::collectResponses() {
    std::vector<std::string> out;
    std::lock_guard<std::mutex> lk(mutex_);
    while (!outputQueue_.empty()) {
        out.push_back(std::move(outputQueue_.front()));
        outputQueue_.pop();
    }
    return out;
}

// ---- processQueue (render thread) ---------------------------------------

void CommandDispatcher::processQueue() {
    std::queue<std::string> local;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        std::swap(local, inputQueue_);
    }
    while (!local.empty()) {
        const std::string cmd = std::move(local.front());
        local.pop();

        auto resp = route(cmd);
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

        std::lock_guard<std::mutex> lk(mutex_);
        outputQueue_.push(std::move(*resp));
    }
}

// ---- route --------------------------------------------------------------

std::optional<std::string> CommandDispatcher::route(const std::string& cmd) {
    if (!world_) return std::string("Error:world not set");

    const std::string_view sv(cmd);

    if (cmd == "Reset") {
        world_->reset();
        if (rigidWorld_) rigidWorld_->reset();
        if (softWorld_) softWorld_->reset();
        world_->refreshCoupling();      // rebind (new bodies from rigid reset)
        world_->refreshSoftCoupling();  // rebind (new bodies from soft reset)
        if (onWorldChanged_) onWorldChanged_();
        return std::string("OK");
    }
    if (cmd == "Step") {
        world_->stepOnce();
        // When SoftBody-Fluid coupling is enabled, stepOnce() above already
        // advanced softWorld_ (see FluidWorld::stepOnce()) -- stepping it
        // again here would double-step it.
        if (softWorld_ && !world_->isSoftCouplingEnabled()) softWorld_->stepForced();
        if (onWorldChanged_) onWorldChanged_();
        return std::string("OK");
    }
    if (sv.rfind("Step:", 0) == 0) {
        // Rigid-body stepping for this pattern is handled by the embedded
        // RigidBodyCommandDispatcher via the processQueue() fallback below
        // (nullopt keeps that fallthrough unchanged); only soft-body
        // stepping is added here, alongside it, with no coupling between
        // the two (see docs/todo/PLAN_softbody_physics_integration.md;
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

    // ---- Emitter (continuous particle generation, docs/todo/PLAN_physics_fluid_emitter.md) ----
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
    // FluidWorld::createWCSPH()/createDFSPH()/docs/todo/PLAN_sph_scale_invariance.md):
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
    // can't tell those apart (docs/todo/PLAN_physics_scenario_test_rebuild.md
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
    if (sv.rfind("SetVolumeGridName:", 0) == 0) {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        volumeConverter_->params().gridName = std::string(sv.substr(18));
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
    if (sv.rfind("SaveVolumeToVdb:", 0) == 0) {
        if (!volumeConverter_) return std::string("Error:volume converter not set");
        if (!volumeConverter_->saveToVdb(std::string(sv.substr(16))))
            return std::string("Error:") + volumeConverter_->lastError();
        return std::string("OK");
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

    return std::nullopt;
}
