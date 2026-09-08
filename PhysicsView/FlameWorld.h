#pragma once

#include "../Physics/FlameFluid.h"
#include "../Physics/FlameSolver.h"

#include <glm/glm.hpp>

#include <memory>
#include <random>

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

    /** @brief Advances the flame sim by one fixed 1/60 s step if running. */
    void step();

    /** @brief One fixed 1/60 s step regardless of isRunning() (manual "Step"). */
    void stepOnce();

    Phantom::Physics::FlameFluid&       fluid()        { return *fluid_; }
    const Phantom::Physics::FlameFluid& fluid() const  { return *fluid_; }
    Phantom::Physics::FlameSolver&      solver()       { return *solver_; }

    /** @brief Display-only parameters (were FlameApp members); never affect the sim. */
    struct RenderParams {
        float pointSize        = 14.0f;
        float smokePointSize   = 20.0f;
        bool  pbvrMode         = false;
        float flameOpacityScale = 1.0f;
        float smokeOpacityScale = 1.0f;
        int   pbvrRepeatCount   = 1;

        // Display transform onto FluidApp's shared FluidRenderer camera space.
        float     renderScale  = 12.0f;
        glm::vec3 renderOffset { 20.0f, 4.0f, 20.0f };
    };

    RenderParams&       render()       { return render_; }
    const RenderParams& render() const { return render_; }

    // Fresh independent draw every frame for PBVR stochastic thinning (not
    // re-seeded), same idea as FlameView's pbvrRng_.
    std::mt19937& pbvrRng() { return pbvrRng_; }

private:
    std::unique_ptr<Phantom::Physics::FlameFluid>  fluid_;
    std::unique_ptr<Phantom::Physics::FlameSolver> solver_;
    bool         running_ = false;
    RenderParams render_;
    std::mt19937 pbvrRng_{ 1337 };

    void buildScene();
};

} // namespace Phantom
