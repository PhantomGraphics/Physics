#pragma once

#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/ISPHSolver.h"
#include "../Physics/SphereBoundary.h"
#include "../Physics/PhysicsSolver.h"
#include "../Physics/SoftBoundaryParticles.h"
#include "../Physics/RigidBoundary.h"
#include "../Physics/MeshBoundaryShape.h"
#include "../Physics/WhiteWaterSystem.h"
#include "../Fluid_GPU_Vk/CSPHSolverVk.h"
#include "RigidBodyWorld.h"
#include "SoftBodyWorld.h"

#include "../../CGLib/Math/Box3d.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

#include <filesystem>

namespace Phantom {

/**
 * @brief Top-level scene for FluidApp: an SPH fluid plus an embedded
 * RigidBodyWorld, with an optional Rigid-Fluid coupling layer between
 * them (docs/todo/PLAN_rigid_fluid_coupling.md).
 *
 * rigid() is a plain, fully-functional rigid-body world in its own right --
 * this class adds nothing when coupling is disabled:
 *   - Fluid only:  use this object directly, leave coupling disabled.
 *   - Rigid only:  use rigid(), leave coupling disabled.
 *   - Both, uncoupled: use both side by side (their pre-existing behavior).
 *   - Both, coupled: setCouplingEnabled(true) -- rigid bodies act as SDF
 *     penalty boundaries (One-Way) or Akinci boundary particles (Two-Way)
 *     against the active fluid solver.
 *
 * Previously this composition (then named PhysicsSceneWorld) lived as a
 * separate class wrapping a plain FluidWorld; it was folded directly in here
 * since FluidWorld was always its only fluid half and FluidApp its only
 * caller -- there was no independent reason for the wrapper to exist apart
 * from FluidWorld itself.
 *
 * Owns the single Phantom::Physics::PhysicsSolver (physicsSolver_) that rigid_ and the
 * sibling SoftBodyWorld (set via setSoftBodyWorld()) both reference -- mirrors
 * CGApp/Universe/Entity/UniverseScene's single-PhysicsSolver ownership (see
 * Physics/CLAUDE.md). Rigid-Fluid/SoftBody-Fluid coupling below (refreshCoupling()/
 * refreshSoftCoupling()/step()/stepOnce()) still hand-orchestrates sync/step/apply-reaction
 * itself rather than calling PhysicsSolver::step()/stepUnconditional()/setRunning(), because
 * this class -- unlike UniverseScene -- needs fluid/rigid/soft to play/pause independently
 * (three separate isRunning() flags) and to couple only the subset that's actually running;
 * PhysicsSolver's step() ties all three to one running_ flag and always steps every part
 * together. physicsSolver_.rigidFluidSolver()/softFluidSolver() are used directly instead.
 */
class FluidWorld {
public:
    FluidWorld();

    enum class SimulationType {
        DFSPH    = 0,
        PBSPH    = 1,
        WCSPH    = 2,
        GPU_CSPH = 3,
    };

    struct Params {
        float timeStep = 0.01f;
        float radius = 1.0f;
        float effectLength = 2.25f;
        float density = 1.0f;
        // Raw pressure/PBF-relaxation coefficient. Only meaningful for
        // PBSPH's WCSPHFluid::setStiffness()-equivalent (PBSPHFluid::setStiffness()) --
        // already scale-invariant when held fixed (docs/todo/PLAN_sph_scale_invariance.md
        // Phase 1's PositionCorrectionOverRadiusIsScaleInvariant test). WCSPH
        // and DFSPH's Two-Way boundary coupling force instead derive their
        // pressureCoe from pressureCoeScale below (see createWCSPH()/
        // createDFSPH()); this field is unused for WCSPH/DFSPH/GPU_CSPH.
        float stiffness = 20.0f;
        // Proportionality constant used by WCSPH's own pressure solve and
        // DFSPH's Two-Way boundary coupling force to derive their pressureCoe
        // via WCSPHFluid::estimatePressureCoe()/setPressureCoeFromScale()
        // (pressureCoe = pressureCoeScale * effectLength, see docs/todo/
        // PLAN_sph_scale_invariance.md section 4/Phase 1). A raw pressureCoe
        // means something different at every scene scale (fixed stiffness ->
        // pressure accel scales as 1/effectLength), whereas this scale stays
        // meaningful regardless of radius/effectLength. Defaults to 1960.0f,
        // matching the historical default (target density error 1% at
        // gravity=9.8) under the retired physics-based derivation.
        float pressureCoeScale = 1960.0f;
        // How much of a particle's wall-normal velocity the domain walls
        // (the box/plane boundary and any SphereBoundary) absorb on contact,
        // via ISPHSolver::setBoundaryDampingRatio(). 0 (the default) is the
        // historical undamped penalty spring, whose restitution is ~1 -- fine
        // under a settled pool, but it turns a jet landing in an *empty*
        // container into upward spray. Clamped to [0, 0.5] by the solver;
        // ~0.35 is where the measured restitution bottoms out (see
        // SphereBoundaryTest's DampedBoundaryForceAbsorbsMostOfTheRebound and
        // docs/issue/water_sphere_showcase_emitter_instability.md).
        // Applied by reset(); WCSPH and DFSPH honor it, PBSPH does not need
        // it (it hard-clamps predicted positions instead).
        float boundaryDampingRatio = 0.0f;
        float viscosity = 5.0f;
        // Surface tension coefficient (WCSPH only, see WCSPHFluid::setTensionCoe()
        // -- docs/todo/PLAN_sph_surface_tension.md). Defaults to 0.f (disabled),
        // matching WCSPHFluid's own zero-initialized default.
        float tension = 0.0f;
        Phantom::Math::Vector3df gravity = Phantom::Math::Vector3df(0.0f, -9.8f, 0.0f);
        Phantom::Math::Box3df boundary = Phantom::Math::Box3df(
            Phantom::Math::Vector3df(0.0f, 0.0f, 0.0f),
            Phantom::Math::Vector3df(100.0f, 100.0f, 100.0f));
        Phantom::Math::Box3df fluidBounds = Phantom::Math::Box3df(
            Phantom::Math::Vector3df(0.0f, 0.0f, 0.0f),
            Phantom::Math::Vector3df(20.0f, 20.0f, 20.0f));
        // Hard cap on live particle count, applied to whichever *Fluid
        // reset() builds (WCSPHFluid/DFSPHFluid/PBSPHFluid all default this
        // to 50000 themselves -- see their own maxParticles_ -- so leaving
        // this field at its matching default is a no-op for every scene that
        // never touches it). Mainly exists for emitter-fed scenes that start
        // from zero particles and need a much larger budget (e.g. the SPH
        // showcase presets' several-hundred-thousand-particle targets).
        int maxParticles = 50000;
    };

    // Overrides the uniform-grid particle seeding for the next reset().
    // When positions is non-empty, these positions (and optional per-particle
    // velocities) are used instead of sampling from params().fluidBounds.
    // Pass empty vectors to revert to the default fluidBounds sampling.
    void setCustomInitialData(std::vector<Phantom::Math::Vector3df> positions,
                              std::vector<Phantom::Math::Vector3df> velocities = {});

    // Must be called before reset() when GPU_CSPH is used.
    // ctx and pool must outlive this object.
    void setVulkanContext(const Phantom::VKG::VulkanContext& ctx,
                          const Phantom::VKG::VulkanCommandPool& pool)
    {
        vkCtx_  = &ctx;
        vkPool_ = &pool;
    }

    void setSimulationType(SimulationType t) { type_ = t; }
    SimulationType getSimulationType() const { return type_; }

    // Rigid-Fluid coupling (docs/todo/PLAN_rigid_fluid_coupling.md): dispatches
    // to whichever solver is currently active. Track A (SDF penalty, one-way)
    // is supported by every CPU solver (DFSPH/PBSPH/CSPH); Track B
    // (boundary particles, two-way) only by DFSPH/PBSPH. GPU_CSPH supports
    // neither (no CPU solver instance to register with) -- these calls are a
    // silent no-op for it, so callers should check supportsOneWayCoupling()/
    // supportsTwoWayCoupling() before relying on registration having taken effect.
    void addRigidBoundary(Phantom::Physics::RigidBoundary* b);
    void clearRigidBoundaries();
    void addRigidBoundaryParticles(Phantom::Physics::RigidBoundaryParticles* p);
    void clearRigidBoundaryParticles();
    bool supportsOneWayCoupling() const { return type_ != SimulationType::GPU_CSPH; }
    // Delegates to the active ISPHSolver instead of a type_ switch (WCSPH
    // joined DFSPH/PBSPH in Phase 4 of
    // docs/todo/PLAN_physics_ownership_and_coupling_unification.md, and this
    // hardcoded switch predated ISPHSolver::supportsTwoWayCoupling() -- it
    // would otherwise have needed a 3rd case added here to stay correct).
    bool supportsTwoWayCoupling() const { return fluidSolver_ && fluidSolver_->supportsTwoWayCoupling(); }

    /**
     * @brief Loads a triangle mesh from an STL file and registers it as a
     * static One-Way (SDF penalty) fluid boundary (Physics::MeshBoundaryShape
     * + Physics::RigidBoundary), independent of rigid()/RigidFluidSolver --
     * this is not a RigidBody, just a second boundary registered directly on
     * the active solver via addRigidBoundary(), same mechanism refreshCoupling()
     * uses for rigid bodies. Replaces any previously loaded mesh boundary.
     * Re-registers itself automatically after reset() (which rebuilds
     * fluidSolver_), mirroring refreshCoupling()'s post-reset rebind.
     * @return False if the file could not be read as an STL mesh.
     */
    bool loadMeshBoundary(const std::filesystem::path& stlPath, float voxelSize = 0.1f);

    /** @brief Unregisters and discards the currently loaded mesh boundary, if any. */
    void clearMeshBoundary();

    /** @brief True once loadMeshBoundary() has succeeded and hasn't been cleared since. */
    bool hasMeshBoundary() const { return meshBoundaryShape_ != nullptr; }

    /** @brief Triangle count of the currently loaded mesh boundary (0 if none). */
    size_t getMeshBoundaryTriangleCount() const {
        return meshBoundaryShape_ ? meshBoundaryShape_->getTriangleCount() : 0;
    }

    // ---- Sphere boundary (docs/todo/PLAN_sph_showcase_water_sphere.md) ----
    // WCSPH-only (ISPHSolver::setBoundarySpheres() defaults to a no-op; only
    // WCSPHSolver overrides it -- see that header's doc comment for why DFSPH/
    // PBSPH intentionally don't). Unlike addRigidBoundary()/loadMeshBoundary(),
    // which register non-owning pointers directly on the solver,
    // setBoundarySpheres() takes its whole list by value each call, so this
    // list is the only copy that needs to survive reset() -- reregistered
    // wholesale (replacing whatever the fresh solver started with) the same
    // way reregisterMeshBoundary() re-adds the mesh boundary pointer.

    /** @brief Adds a solid-sphere boundary (persists across reset()). */
    void addBoundarySphere(const Phantom::Physics::SphereBoundary& s);

    /** @brief Removes every registered sphere boundary. */
    void clearBoundarySpheres();

    /** @brief Currently registered sphere boundaries (independent of solver type/reset() state). */
    const std::vector<Phantom::Physics::SphereBoundary>& getBoundarySpheres() const { return boundarySpheres_; }

    // ---- Multi-region initial fluid seeding (SPH showcase presets need more
    // than the single axis-aligned box params().fluidBounds supports -- e.g.
    // a thin sheet plus several falling drops seeded at once). Accumulates
    // box/sphere regions; reset() -- if any are registered -- fills
    // customPositions_ from their union (each region sampled on the same
    // uniform lattice params().radius*2 uses elsewhere) instead of the usual
    // single-box params().fluidBounds sampling. A scene that never calls
    // these behaves exactly as before (sourceRegions_ stays empty).

    /** @brief Adds a box region to the next reset()'s initial particle seeding. */
    void addFluidSourceBox(const Phantom::Math::Box3df& bounds);

    /** @brief Adds a spherical region to the next reset()'s initial particle seeding. */
    void addFluidSourceSphere(const Phantom::Math::Vector3df& center, float radius);

    /** @brief Removes every registered source region (reverts to plain params().fluidBounds seeding). */
    void clearFluidSources();

    /** @brief Number of registered source regions. */
    size_t getFluidSourceRegionCount() const { return sourceRegions_.size(); }

    // Kernel/rest-density of the active fluid, needed to compute Track B's
    // psi (RigidBoundaryParticles::computePsi()). Only meaningful (non-null/
    // valid) when supportsTwoWayCoupling() is true.
    Phantom::Physics::SPHKernel* getActiveKernel() const;
    float getActiveRestDensity() const;

    // ---- Emitter (continuous particle generation, docs/todo/PLAN_physics_fluid_emitter.md) ----
    // Dispatches to whichever concrete *Fluid is currently active (same
    // pattern as loadMeshBoundary()/addRigidBoundary() above). A no-op for
    // GPU_CSPH, which has no CPU *Fluid instance to register with (mirrors
    // supportsOneWayCoupling()'s GPU_CSPH exclusion). stepFluidOnly() calls
    // the active fluid's updateEmitters(dt) before fluidSolver_->simulate()
    // every step, so emitters keep spawning particles while running.

    /** @brief Registers a new emission region on the currently active fluid. */
    void addEmitter(const Phantom::Physics::Emitter& e);

    /** @brief Removes all registered emission regions from the currently active fluid. */
    void clearEmitters();

    /** @brief Returns the emission regions registered on the currently active fluid (empty for GPU_CSPH or before reset()). */
    const std::vector<Phantom::Physics::Emitter>& getEmitters() const;

    // ---- Outflow region (optional particle removal) ----
    // Dispatches to whichever concrete *Fluid is currently active (same
    // pattern as addEmitter() above). A no-op for GPU_CSPH, which has no CPU
    // *Fluid instance to register with. stepFluidOnly() calls the active
    // fluid's removeOutflowParticles() after fluidSolver_->simulate() every
    // step, so particles that have moved into a registered region are culled
    // before the next step. Purely opt-in -- a scene that never calls
    // addOutflowRegion() behaves exactly as before.

    /** @brief Registers a new outflow (deletion) region on the currently active fluid. */
    void addOutflowRegion(const Phantom::Physics::OutflowRegion& r);

    /** @brief Removes all registered outflow regions from the currently active fluid. */
    void clearOutflowRegions();

    /** @brief Returns the outflow regions registered on the currently active fluid (empty for GPU_CSPH or before reset()). */
    const std::vector<Phantom::Physics::OutflowRegion>& getOutflowRegions() const;

    Params&       params()       { return params_; }
    const Params& params() const { return params_; }

    void setRunning(bool running) { running_ = running; }
    bool isRunning() const { return running_; }

    void reset();

    /**
     * @brief Advances whichever of this fluid / rigid() are currently
     * running, independently -- unless coupling is enabled and both are
     * running, in which case they step together in RigidFluidSolver's
     * documented order (sync boundaries -> fluid step -> apply Two-Way
     * reaction -> rigid step). Used by the automatic per-frame update loop.
     */
    void step();

    /**
     * @brief Unconditionally advances both this fluid and rigid() by one
     * step (coupled if enabled), ignoring their individual isRunning()
     * flags -- for a manual/scenario-driven "Step" command. Uses
     * RigidBodyWorld::stepForced() so the rigid body actually
     * integrates without needing setRunning(true) (which would also make
     * the interactive per-frame loop advance the sim on its own,
     * contaminating scenario step counts with wall-clock-timed extra steps).
     */
    void stepOnce();

    /**
     * @brief Time step both sides of a *coupled* frame advance by (the fluid's
     * own params().timeStep). The uncoupled paths keep using each world's own
     * timeStep; see the .cpp for why a coupled frame cannot.
     */
    float coupledTimeStep() const;

    RigidBodyWorld&       rigid()       { return rigid_; }
    const RigidBodyWorld& rigid() const { return rigid_; }

    // The single Phantom::Physics::PhysicsSolver shared by rigid() and the sibling
    // SoftBodyWorld set via setSoftBodyWorld() (mirrors CGApp/Universe/Entity/UniverseScene's
    // single PhysicsSolver ownership). FluidApp constructs its SoftBodyWorld with this.
    Phantom::Physics::PhysicsSolver& physicsSolver() { return physicsSolver_; }

    /**
     * @brief Enables/disables Rigid-Fluid coupling. Rebuilds bindings
     * immediately (see refreshCoupling()); disabling clears them and
     * restores this fluid/rigid() to their independent, uncoupled behavior.
     */
    void setCouplingEnabled(bool v);
    bool isCouplingEnabled() const { return couplingEnabled_; }

    /** @brief One-Way (no reaction) or Two-Way (reaction) for newly (re)established bindings. */
    void setCouplingMode(Phantom::Physics::CouplingMode mode);
    Phantom::Physics::CouplingMode couplingMode() const { return couplingMode_; }

    /**
     * @brief The mode refreshCoupling() actually binds a generic dynamic
     * (non-plane) rigid body with, as opposed to couplingMode()'s requested
     * mode -- these differ exactly when TwoWay was requested but the active
     * solver doesn't supportTwoWayCoupling() (e.g. GPU_CSPH), in which
     * case refreshCoupling() silently falls back to OneWay per body. Exists
     * so scenario tests can detect that fallback (see
     * docs/todo/PLAN_physics_scenario_test_rebuild.md 1.1) instead of
     * asserting on the requested mode and never noticing it wasn't honored.
     * Plane-shaped bodies (floors) are always bound OneWay regardless of
     * this value (refreshCoupling() excludes static bodies, but a dynamic
     * plane would still hit that per-body fallback) -- this reports the
     * mode for the common case, not a per-body guarantee.
     */
    Phantom::Physics::CouplingMode activeCouplingMode() const {
        return (couplingMode_ == Phantom::Physics::CouplingMode::TwoWay && !supportsTwoWayCoupling())
            ? Phantom::Physics::CouplingMode::OneWay
            : couplingMode_;
    }

    /**
     * @brief Re-binds every non-static rigid body with a shape to the active
     * fluid solver, using the current coupling mode. Call whenever the set
     * of rigid bodies changes while coupling is enabled (preset switch,
     * AddSphere/AddBox/AddFloor) -- e.g. from RigidBodyControlPanel's/
     * RigidBodyCommandDispatcher's onWorldChanged callback. A no-op if
     * coupling is disabled.
     */
    void refreshCoupling();

    // SoftBody-Fluid coupling (mirrors the Rigid-Fluid coupling above, but
    // SoftBody has no analytic collision shape so binding is always Two-Way
    // -- there is no One-Way/mode toggle equivalent). SoftBodyWorld is a
    // sibling owned by FluidApp (unlike rigid(), which this class owns
    // directly), so it is referenced by a non-owning pointer set once via
    // setSoftBodyWorld(), the same way setVulkanContext()'s ctx/pool are.
    void setSoftBodyWorld(SoftBodyWorld* soft) { softWorld_ = soft; }

    void addSoftBoundaryParticles(Phantom::Physics::SoftBoundaryParticles* p);
    void clearSoftBoundaryParticles();

    /**
     * @brief Enables/disables SoftBody-Fluid coupling. Rebuilds bindings
     * immediately (see refreshSoftCoupling()); disabling clears them and
     * restores softWorld_ to its independent, uncoupled behavior. A no-op
     * (stays disabled) if setSoftBodyWorld() was never called or the active
     * solver doesn't supportTwoWayCoupling().
     */
    void setSoftCouplingEnabled(bool v);
    bool isSoftCouplingEnabled() const { return softCouplingEnabled_; }

    /**
     * @brief Re-binds every body currently in softWorld_ to the active fluid
     * solver. Call whenever the set of soft bodies changes while soft
     * coupling is enabled (preset switch, Reset) -- mirrors refreshCoupling().
     * A no-op if soft coupling is disabled.
     */
    void refreshSoftCoupling();

    size_t getParticleCount() const;
    std::vector<glm::vec3> getParticlePositions() const;
    // Empty for GPU_CSPH (no CPU *Fluid instance to read velocities back
    // from -- mirrors getParticlePositions()'s own GPU_CSPH gap; scenario
    // callers see an empty read the same way GetMax/MinParticlePositionY
    // already does for that type).
    std::vector<glm::vec3> getParticleVelocities() const;
    std::vector<glm::vec3> getSprayPositions() const;
    std::vector<float> getSprayLives() const;
    std::vector<glm::vec3> getFoamPositions() const;
    std::vector<float> getFoamLives() const;

    Phantom::Physics::CSPHSolverVk* getGpuSolver() const { return gpuSolver_.get(); }

    // Frees gpuSolver_'s VMA-backed buffers (CSPHParticleBufferVk/
    // CSPHGridBufferVk) while the owning VulkanContext is still alive. Must
    // be called from the app's onCleanup() hook, before VkAppBase::cleanup()
    // destroys the context's VmaAllocator -- FluidWorld's own destructor
    // runs too late for this (after the context is already gone), which is
    // what caused the VMA "allocations were not freed" abort on shutdown
    // whenever GPU_CSPH was ever selected (docs/issue/CODEBASE_ISSUES.md 1.3).
    void releaseGpuResources() { gpuSolver_.reset(); }

    Phantom::Physics::WhiteWaterSystem::Params&       whiteWaterParams()       { return whiteWater_.params; }
    const Phantom::Physics::WhiteWaterSystem::Params& whiteWaterParams() const { return whiteWater_.params; }

    // Returns the wall-clock duration of the last step() call in milliseconds.
    float getLastStepTimeMs() const { return lastStepTimeMs_; }

private:
    SimulationType type_ = SimulationType::DFSPH;
    Params params_;
    bool running_ = false;

    // Declared before rigid_ (member init order follows declaration order, not the
    // constructor's init-list order) so RigidBodyWorld's constructor can take a reference to
    // this. See Physics/CLAUDE.md's ownership note: neither RigidBodySolver nor SoftBodySolver
    // owns bodies -- this single PhysicsSolver (shared with the sibling SoftBodyWorld via
    // physicsSolver()) replaces what used to be two separate, independently-owned
    // RigidFluidSolver/SoftFluidSolver instances inside RigidBodyWorld/SoftBodyWorld.
    Phantom::Physics::PhysicsSolver physicsSolver_;

    RigidBodyWorld rigid_;
    bool                            couplingEnabled_ = false;
    Phantom::Physics::CouplingMode  couplingMode_    = Phantom::Physics::CouplingMode::OneWay;
    float                           couplingSpacing_ = 0.2f;  // Two-Way boundary-particle sampling spacing

    // SoftBody-Fluid coupling. Non-owning: FluidApp owns the actual SoftBodyWorld.
    SoftBodyWorld* softWorld_ = nullptr;
    bool           softCouplingEnabled_ = false;

    void teardownSoftCoupling();

    // Static mesh fluid boundary (loadMeshBoundary()/clearMeshBoundary()), independent
    // of rigid()/RigidFluidSolver -- see loadMeshBoundary()'s doc comment.
    std::unique_ptr<Phantom::Physics::MeshBoundaryShape> meshBoundaryShape_;
    Phantom::Physics::RigidBoundary                      meshBoundary_;

    // Re-registers meshBoundary_ on the (possibly just-rebuilt) active solver.
    // A no-op if no mesh boundary is loaded. Called by loadMeshBoundary() and reset().
    void reregisterMeshBoundary();

    // Sphere boundaries (addBoundarySphere()/clearBoundarySpheres() above).
    std::vector<Phantom::Physics::SphereBoundary> boundarySpheres_;

    // Re-registers boundarySpheres_ (whole list, by value) on the
    // (possibly just-rebuilt) active solver. A no-op (ISPHSolver's default)
    // on solvers that don't override setBoundarySpheres(). Called by
    // addBoundarySphere()/clearBoundarySpheres() and reset().
    void reregisterBoundarySpheres();

    // Multi-region initial fluid seeding (addFluidSourceBox()/
    // addFluidSourceSphere() above).
    struct SourceRegion {
        bool isSphere = false;
        Phantom::Math::Box3df    box;      // isSphere == false
        Phantom::Math::Vector3df center;   // isSphere == true
        float                    radius = 0.0f;
    };
    std::vector<SourceRegion> sourceRegions_;

    // Samples every region in sourceRegions_ on a params().radius*2 lattice
    // and returns the concatenated points (regions may overlap -- callers are
    // expected to lay them out so they don't, same as Blender preset authors
    // are expected to).
    std::vector<Phantom::Math::Vector3df> buildSourceRegionPositions() const;

    const Phantom::VKG::VulkanContext*     vkCtx_  = nullptr;
    const Phantom::VKG::VulkanCommandPool* vkPool_ = nullptr;

    float lastStepTimeMs_ = 0.0f;

    std::unique_ptr<Phantom::Physics::DFSPHFluid>  dfsphFluid_;
    std::unique_ptr<Phantom::Physics::PBSPHFluid>  pbsphFluid_;

    std::unique_ptr<Phantom::Physics::WCSPHFluid>  csphFluid_;

    // The active fluid type's CPU solver (see ISPHSolver's class doc). Only
    // one of dfsphFluid_/pbsphFluid_/csphFluid_ is populated at a
    // time, matching whichever concrete type fluidSolver_ currently holds.
    // GPU_CSPH uses gpuSolver_ instead -- CSPHSolverVk is a GPU-only class
    // with its own simulate() and does not implement ISPHSolver.
    std::unique_ptr<Phantom::Physics::ISPHSolver> fluidSolver_;

    std::unique_ptr<Phantom::Physics::CSPHSolverVk> gpuSolver_;
    Phantom::Physics::WhiteWaterSystem              whiteWater_;

    std::vector<Phantom::Math::Vector3df> customPositions_;
    std::vector<Phantom::Math::Vector3df> customVelocities_;

    void clear();
    void createDFSPH();
    void createPBSPH();
    void createWCSPH();
    void createGPUCSPH();

    void updateWhiteWater(float dt);

    // Dispatches to the active fluid's updateEmitters(dt) (see addEmitter()'s
    // doc comment above). No-op for GPU_CSPH.
    void updateEmitters(float dt);

    // Dispatches to the active fluid's removeOutflowParticles() (see
    // addOutflowRegion()'s doc comment above). No-op for GPU_CSPH.
    void updateOutflow();

    // The raw single-fluid step (previously step()'s whole body, before
    // step() also had to orchestrate rigid()/coupling).
    void stepFluidOnly();
    void teardownCoupling();
};

} // namespace Phantom
