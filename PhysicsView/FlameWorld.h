#pragma once

#include "../Physics/FlameFluid.h"
#include "../Physics/FlameSolver.h"

#include <glm/glm.hpp>

#include <memory>

namespace Phantom {

/**
 * @brief Standalone Flame (reacting hot-gas SPH) scene for FluidApp, folded in
 * from the former standalone FlameView/FlameApp.
 *
 * Owns a FlameFluid + FlameSolver and drives them exactly as FlameView did:
 * the same initial scene (buildScene()), the same fixed 1/60 s step, the same
 * deterministic RNG seed. There is deliberately no Rigid/Soft/Fluid coupling --
 * FlameSolver does not implement ISPHSolver and is not registered with
 * PhysicsSolver (see Physics/CLAUDE.md's Flame section). It sits alongside the
 * fluid/rigid/soft domains in FluidApp with its own Play/Pause/Step, its own
 * control page (ControlPage::Flame) and its own sub-renderer (FlameRenderer),
 * touching none of the coupling machinery.
 *
 * The simulation runs at its native ~3-unit scale (identical numbers to
 * FlameView, so it stays deterministic and stable). FluidApp::syncFlameRenderer()
 * multiplies each particle position by render().renderScale and adds
 * render().renderOffset before handing it to FlameRenderer, so the plume
 * composes with the shared FluidRenderer camera (centred on (20,20,20))
 * instead of being a sub-unit speck. That is a pure display transform -- the
 * SPH state itself is byte-identical to FlameView's.
 */
class FlameWorld {
public:
    FlameWorld();

    /** @brief Rebuilds the fluid/solver and re-seeds the initial scene. */
    void reset();

    void setRunning(bool r) { running_ = r; }
    bool isRunning() const { return running_; }

    /** @brief Advances the flame sim by one fixed step (getTimeStep()) if running. */
    void step();

    /** @brief One fixed-size step regardless of isRunning() (manual "Step"). */
    void stepOnce();

    /**
     * @brief Fixed simulation step (seconds, default 1/60). Survives reset() --
     * it belongs to the world, not to the rebuilt fluid -- so a scenario can
     * compare dt=1/60 against dt=1/120 on otherwise identical scenes.
     */
    void  setTimeStep(float dt) { timeStep_ = dt; }
    float getTimeStep() const   { return timeStep_; }

    /** @brief Simulated seconds since the last reset(). */
    float getSimTime() const { return simTime_; }

    Phantom::Physics::FlameFluid&       fluid()        { return *fluid_; }
    const Phantom::Physics::FlameFluid& fluid() const  { return *fluid_; }
    Phantom::Physics::FlameSolver&      solver()       { return *solver_; }

    /** @brief Display-only parameters (were FlameApp members); never affect the sim. */
    struct RenderParams {
        // Sprite diameters in *simulation* units (multiplied by renderScale
        // for display). Projected with each particle's own depth (plan A7);
        // the defaults reproduce the former fixed 14 px / 20 px sprites at
        // the default camera distance (120) and 720 px viewport.
        float particleSize      = 0.16f;
        float smokeParticleSize = 0.23f;
        bool  pbvrMode          = false;

        // Emission (plan Phase 3): blackbody radiance, 1 at the reference
        // temperature and HDR above it; ACES downstream rolls the core to white.
        float exposure               = 4.0f;
        bool  autoReferenceTemperature = true;   ///< track the (smoothed) hottest particle
        float referenceTemperature   = 2000.0f;  ///< used when auto is off
        // White balance (chromatic adaptation, Bradford): 0 = off (colorimetric
        // D65 -- a ~1500 K flame is nearly pure red), 1 = auto (adapt to the
        // radiance reference, i.e. the smoothed hottest temperature: the core
        // renders near-white, cooler gas orange/red), 2 = manual.
        int   whiteBalance            = 1;
        float whiteBalanceTemperature = 2000.0f; ///< manual white (K)
        float whiteBalanceDegree      = 0.8f;    ///< adaptation degree D (1 = full, 0 = none)
        // Absorption: smoke optical density = envelope * soot * smokeOpacityScale,
        // optical depth = smokeExtinction * density.
        float smokeOpacityScale = 1.0f;
        float smokeExtinction   = 2.0f;
        // Hot soot re-emits in proportion to what it absorbs (Kirchhoff).
        // Relative to one optically-thin gas sprite -- which sums over
        // overlapping neighbours -- an opaque soot sample needs ~the typical
        // overlap count to match, hence > 1.
        float smokeGlow         = 4.0f;
        float smokeAlbedo       = 0.03f;         ///< grey albedo x ambient light

        // PBVR (plan Phase 4, FlamePBVRPass).
        bool  pbvrAdaptive       = true;  ///< EnsembleLodController picks R from GPU time
        int   pbvrEnsembles      = 4;     ///< R when not adaptive (1..8)
        int   pbvrTargetEnsembles = 64;   ///< static-frame convergence target
        float pbvrTemporalFrames = 4.0f;  ///< EMA length while running (0 = off)
        float pbvrBudgetMs       = 8.0f;
        float pbvrSubdivision    = 2.0f;  ///< sub-particle diameter = puff / subdivision
        float pbvrMinSubPixels   = 1.5f;  ///< never smaller than this on screen
        float pbvrDensityScale   = 1.0f;

        // Display transform onto FluidApp's shared FluidRenderer camera space.
        float     renderScale  = 12.0f;
        glm::vec3 renderOffset { 20.0f, 4.0f, 20.0f };
    };

    RenderParams&       render()       { return render_; }
    const RenderParams& render() const { return render_; }

private:
    std::unique_ptr<Phantom::Physics::FlameFluid>  fluid_;
    std::unique_ptr<Phantom::Physics::FlameSolver> solver_;
    bool         running_ = false;
    float        timeStep_ = 1.0f / 60.0f;
    float        simTime_  = 0.0f;
    RenderParams render_;

    void buildScene();
};

} // namespace Phantom
