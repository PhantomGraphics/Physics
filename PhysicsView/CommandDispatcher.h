#pragma once
#include "../../CGLib/VkAppBase/ScenarioRunner/IScenarioDispatcher.h"

#include "RigidBodyCommandDispatcher.h"
#include "SoftBodyCommandDispatcher.h"

#include <optional>

namespace Phantom {
    class FluidWorld;
    class RigidBodyWorld;
    class SoftBodyWorld;
    class FluidVolumeConverter;
    class FluidMeshConverter;
    class VolumeRenderer;
    class FluidMeshRenderer;

    // Single IScenarioDispatcher for FluidApp, covering the fluid world,
    // (since the RigidBodyView -> FluidView merge) the rigid-body scene, and
    // (since the SoftBody -> Physics integration, see
    // internal design notes) the soft-body scene.
    // The three worlds are not physically coupled -- this class only routes
    // scenario command strings to whichever world they belong to.
    //
    // "Reset"/"Step"/"Step:N"/"SetRunning:" are handled here directly and
    // affect *all three* worlds, so existing scenarios keep working
    // unchanged. "SetPreset:<name>" is disambiguated by name (soft-body vs.
    // rigid-body preset names never overlap) and routed to the matching
    // sub-dispatcher. Soft-body-only commands (GetSoftBodyCount,
    // GetSoftParticleCount, GetMaxSpeed, SetSelfCollision:) are routed to
    // softDispatcher_ directly since their names don't collide with the
    // rigid-body surface. Every other rigid-body command (AddSphere,
    // GetBodyCount, ...) falls through to the embedded
    // RigidBodyCommandDispatcher unmodified, so existing RigidBodyView
    // scenario command strings keep working too.
    //
    // "SetCoupling*" commands drive FluidWorld's Rigid-Fluid coupling
    // (internal design notes Phase 8) -- see FluidWorld's
    // class doc. "SetSoftFluidCouplingEnabled"/"IsSoftFluidCouplingEnabled"
    // drive the analogous SoftBody-Fluid coupling (always Two-Way; no mode
    // toggle) added alongside it.
    //
    // "LoadMeshBoundary:<path>"/"ClearMeshBoundary"/"GetMeshBoundaryTriangleCount"
    // drive FluidWorld::loadMeshBoundary() -- a static One-Way (SDF penalty)
    // fluid boundary built from an STL mesh (Physics::MeshBoundaryShape),
    // independent of rigid()/the coupling commands above.
    //
    // "AddEmitter:cx,cy,cz,radius,rate,dirX,dirY,dirZ,speed"/"ClearEmitters"/
    // "GetEmitterCount" drive FluidWorld::addEmitter() (internal design notes) -- continuous particle generation on
    // whichever fluid type is currently active.
    //
    // "AddOutflowRegion:minX,minY,minZ,maxX,maxY,maxZ"/"ClearOutflowRegions"/
    // "GetOutflowRegionCount" drive FluidWorld::addOutflowRegion() -- an
    // optional AABB region that deletes any fluid particle that enters it
    // (the deletion counterpart to the emitter commands above), on whichever
    // fluid type is currently active.
    //
    // "SetVolume*"/"ConvertToVolume"/"GetVolumeVoxelCount"/"SaveVolumeToVdb"
    // drive FluidVolumeConverter, which turns the fluid's current particle
    // set into a Volume::SparseVolumef (Physics::SPHVolumeConverter) and
    // optionally writes it out as a .vdb file. Set* commands configure the
    // conversion (they take effect on the next ConvertToVolume, mirroring
    // how the SetFluid* setters above only take effect on the next Reset).
    //
    // "SetVolumeIsoLevel"/"ConvertToMesh"/"GetMeshTriangleCount"/
    // "SaveMeshToObj" drive FluidMeshConverter, the next stage of the same
    // pipeline: Marching Cubes over the volume produced by the last
    // ConvertToVolume, optionally written out as an .obj file.
    // "Set{Volume,Mesh}RenderEnabled"/"Is{Volume,Mesh}RenderEnabled" toggle
    // FluidVolumeRenderer/FluidMeshRenderer's on-screen visibility.
    //
    // ---- SPH showcase porting surface (scenarios/showcase/*.json) -- lets a
    // scenario reproduce one of the SPH showcase presets natively and export the
    // resulting per-frame particle positions as PLY. ----
    //
    // "AddBoundarySphere:cx,cy,cz,radius,maxPenetration"/"ClearBoundarySpheres"/
    // "GetBoundarySphereCount" drive FluidWorld::addBoundarySphere()
    // (ISPHSolver::setBoundarySpheres() is implemented by WCSPH/DFSPH/PBSPH; a
    // no-op only for GPU_CSPH, see SphereBoundary.h).
    //
    // "AddFluidSourceBox:xmin,ymin,zmin,xmax,ymax,zmax"/
    // "AddFluidSourceSphere:cx,cy,cz,radius"/"ClearFluidSources"/
    // "GetFluidSourceRegionCount" drive FluidWorld::addFluidSourceBox()/
    // addFluidSourceSphere() -- multiple simultaneous initial-seed regions
    // (e.g. a thin sheet plus several falling drops), sampled at
    // params().radius*2 spacing same as the single-box SetFluidBounds path.
    // Take effect on the next Reset, same as the SetFluid* setters above.
    //
    // "SetFluidMaxParticles:<n>" drives FluidWorld::Params::maxParticles --
    // the hard cap an emitter-fed fluid stops spawning at (WCSPHFluid/
    // DFSPHFluid/PBSPHFluid all default this to 50000 themselves; only scenes
    // that need a much larger emitter budget, like the water-sphere showcase,
    // need to raise it).
    //
    // "SetFluidBoundaryDamping:<ratio>" drives
    // FluidWorld::Params::boundaryDampingRatio -> ISPHSolver::
    // setBoundaryDampingRatio(): how much of a particle's wall-normal
    // velocity the domain walls absorb on contact. 0 (the default) is the
    // historical undamped penalty spring, which bounces at restitution ~1;
    // the water-sphere showcase uses 0.35. WCSPH/DFSPH only. Takes effect on
    // the next Reset, same as the SetFluid* setters above.
    //
    // "SetPLYOutputDir:<dir>" sets (and creates) the directory
    // "StepFrameAndSavePLY:<substeps>" writes into, resetting its frame
    // counter to 1. "SavePLY:<path>" writes the current particle positions to
    // an explicit path in one shot, independent of that counter/directory.
    // "StepFrameAndSavePLY:<substeps>" performs <substeps> plain physics
    // steps (same as that many "Step" commands) and then writes
    // "<dir>/frame_%04d.ply" using the counter, incrementing it -- one
    // scenario step with "repeat": frame_end reproduces an entire showcase
    // bake (frame_end x substeps_per_frame physics steps, one PLY per
    // frame) without a per-iteration templated filename.
    class CommandDispatcher : public IScenarioDispatcher {
    public:
        void setWorld(FluidWorld* w) { world_ = w; }
        void setRigidWorld(RigidBodyWorld* w) { rigidWorld_ = w; rigidDispatcher_.setWorld(w); }
        void setSoftWorld(SoftBodyWorld* w) { softWorld_ = w; softDispatcher_.setWorld(w); }
        void setVolumeConverter(FluidVolumeConverter* c) { volumeConverter_ = c; }
        void setMeshConverter(FluidMeshConverter* c) { meshConverter_ = c; }
        void setVolumeRenderer(VolumeRenderer* r) { volumeRenderer_ = r; }
        void setMeshRenderer(FluidMeshRenderer* r) { meshRenderer_ = r; }

        // Called after Reset or Step so the app can sync GPU buffers.
        void setOnWorldChanged(std::function<void()> cb) { onWorldChanged_ = std::move(cb); }
        void setOnRigidWorldChanged(std::function<void()> cb) { rigidDispatcher_.setOnWorldChanged(std::move(cb)); }
        void setOnSoftWorldChanged(std::function<void()> cb) { softDispatcher_.setOnWorldChanged(std::move(cb)); }

        // Called after a successful ConvertToVolume / ConvertToMesh so the
        // app can rebuild the corresponding renderer's GPU buffer.
        void setOnVolumeChanged(std::function<void()> cb) { onVolumeChanged_ = std::move(cb); }
        void setOnMeshChanged(std::function<void()> cb) { onMeshChanged_ = std::move(cb); }

        // Call from the render thread (onUpdate) every frame.
        void processQueue();

        // IScenarioDispatcher
        void dispatch(const std::string& command) override;
        std::vector<std::string> collectResponses() override;

        // Passthrough for RigidBodyViewApp's screenshot-on-scenario-command pattern.
        std::optional<std::filesystem::path> takePendingScreenshot() { return rigidDispatcher_.takePendingScreenshot(); }
        // Pushes straight onto this dispatcher's own outputQueue_ (not
        // rigidDispatcher_'s) -- collectResponses() below is what
        // ScenarioRunner::tick() actually polls, and nothing ever drains
        // rigidDispatcher_'s queue again after the "SaveScreenshot:" command
        // itself was dispatched (its route() intentionally returns no
        // response then; see CommandDispatcher::processQueue()). Delegating
        // this call to rigidDispatcher_.signalScreenshotDone() left the
        // response stranded there forever, hanging the scenario runner on
        // every SaveScreenshot step (internal design notes 1.8).
        void signalScreenshotDone(bool ok, const std::string& path) {
            std::lock_guard<std::mutex> lk(mutex_);
            outputQueue_.push(ok ? "OK" : "FAIL:" + path);
        }

    private:
        // Returns nullopt for commands not handled here, so processQueue()
        // can fall back to rigidDispatcher_.
        std::optional<std::string> route(const std::string& cmd);

        FluidWorld* world_ = nullptr;
        RigidBodyWorld* rigidWorld_ = nullptr;
        RigidBodyCommandDispatcher rigidDispatcher_;
        SoftBodyWorld* softWorld_ = nullptr;
        SoftBodyCommandDispatcher softDispatcher_;
        FluidVolumeConverter* volumeConverter_ = nullptr;
        FluidMeshConverter* meshConverter_ = nullptr;
        VolumeRenderer* volumeRenderer_ = nullptr;
        FluidMeshRenderer* meshRenderer_ = nullptr;
        std::function<void()> onWorldChanged_;
        std::function<void()> onVolumeChanged_;
        std::function<void()> onMeshChanged_;

        // PLY sequence export (SetPLYOutputDir/SavePLY/StepFrameAndSavePLY
        // below) -- see their doc comment at the route() call site.
        std::filesystem::path plyOutputDir_;
        int                   plyFrameCounter_ = 1;

        std::mutex              mutex_;
        std::queue<std::string> inputQueue_;
        std::queue<std::string> outputQueue_;
    };

}
