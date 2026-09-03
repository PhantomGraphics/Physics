// 03_solver_interface.cpp
// ============================================================================
// One stepping loop, two solvers, through the ISPHSolver interface.
//
// WCSPHSolver and PBSPHSolver are different fluid models (weakly compressible
// vs. position-based), but both implement ISPHSolver. Once a scene is built,
// the stepping and query code holds a std::unique_ptr<ISPHSolver> and never
// mentions the concrete type again.
//
// NOTE: the material parameters below are NOT shared -- each SPH model has its
// own stable regime, and there is no set of numbers that means the same thing
// to both. What is shared is the geometry, the interface, and the loop.
//
//     ./sph_example_03_solver_interface
// ============================================================================

#include "ISPHSolver.h"
#include "WCSPHFluid.h"
#include "WCSPHSolver.h"
#include "PBSPHFluid.h"
#include "PBSPHSolver.h"

#include "sph_examples_common.h"

#include <cstdio>
#include <memory>
#include <string>

using namespace sph_examples;
namespace P = Phantom::Physics;

// Shared geometry (mirrors scenarios 11 & 12: a resting 3x3x3 block that
// settles under gravity in a roomy box).
namespace scene {
    constexpr float radius       = 0.15f;
    constexpr float spacing      = radius * 2.0f;
    constexpr float effectLength = 1.0f;
    constexpr float restDensity  = 1.0f;
    constexpr float dt           = 0.005f;
    constexpr int   steps        = 600;
    const Box3df container(Vector3df(-3.f, -3.f, -3.f), Vector3df(3.f, 3.f, 3.f));
    const Box3df block    (Vector3df(-0.3f, 0.6f, -0.3f), Vector3df(0.3f, 1.2f, 0.3f));
    const Vector3df gravity(0.f, -9.8f, 0.f);
}

// A solver plus the fluid it steps. The solver holds only a non-owning pointer
// to the fluid, so this struct keeps both alive together and exposes the base
// interface through operator->.
struct Sim {
    std::unique_ptr<P::WCSPHFluid> wcsph;
    std::unique_ptr<P::PBSPHFluid> pbsph;
    std::unique_ptr<P::ISPHSolver> solver;
    P::ISPHSolver* operator->() const { return solver.get(); }
};

static Sim makeWCSPH()
{
    Sim s;
    s.wcsph = std::make_unique<P::WCSPHFluid>();
    s.wcsph->setDensity(scene::restDensity);
    s.wcsph->setEffectLength(scene::effectLength);
    s.wcsph->setPressureCoeFromScale(1960.0f);
    s.wcsph->setVicosityCoe(0.5f);
    s.wcsph->setTensionCoe(0.0f);
    for (const auto& p : seedBox(scene::block, scene::spacing))
        s.wcsph->createParticle(p, scene::radius);

    auto solver = std::make_unique<P::WCSPHSolver>();
    solver->add(s.wcsph.get());
    s.solver = std::move(solver);
    return s;
}

static Sim makePBSPH()
{
    Sim s;
    s.pbsph = std::make_unique<P::PBSPHFluid>();
    s.pbsph->setEffectLength(scene::effectLength);
    s.pbsph->setRestDensity(scene::restDensity);
    s.pbsph->setStiffness(0.001f);        // PBSPH's own stable regime
    s.pbsph->setVicsosity(0.0f);
    for (const auto& p : seedBox(scene::block, scene::spacing))
        s.pbsph->createParticle(p, scene::radius);

    auto solver = std::make_unique<P::PBSPHSolver>();
    solver->add(s.pbsph.get());
    s.solver = std::move(solver);
    return s;
}

// From here down, only ISPHSolver is used -- no solver-specific code.
static void run(const std::string& name, Sim sim)
{
    sim->setEffectLength(scene::effectLength);
    sim->setTimeStep(scene::dt);
    sim->setExternalForce(scene::gravity);
    sim->setBoundary(scene::container, scene::dt);
    // A little wall-normal damping is part of the scene, not a material
    // parameter.  Without it PBSPH's hard positional wall constraint and
    // WCSPH's penalty wall retain very different amounts of impact energy,
    // so this example compared boundary transients rather than the solvers.
    sim->setBoundaryDampingRatio(0.2f);

    const int   n0 = sim->getParticleCount();
    const float y0 = minY(sim->getParticlePositions());

    Stopwatch clock;
    float worstSpeed = 0.f;
    for (int step = 0; step < scene::steps; ++step) {
        sim->simulate(scene::dt, 4);
        worstSpeed = std::max(worstSpeed, maxSpeed(sim->getParticleVelocities()));
    }

    const auto pos = sim->getParticlePositions();
    const float finalSpeed = maxSpeed(sim->getParticleVelocities());
    std::printf("  %-6s  n=%d  minY %.2f -> %.2f  restDensity %.3f  "
                "peakSpeed %.2f  finalSpeed %.2f  %.2fs\n",
                name.c_str(), n0, y0, minY(pos),
                sim->getRestDensity(), worstSpeed, finalSpeed, clock.seconds());
}

int main()
{
    const int n = static_cast<int>(seedBox(scene::block, scene::spacing).size());
    std::printf("%d-particle block settling in a box, %d steps, one ISPHSolver* loop:\n",
                n, scene::steps);
    run("WCSPH", makeWCSPH());
    run("PBSPH", makePBSPH());
    std::printf("\nBoth were driven through the identical ISPHSolver* code path in run().\n");
    return 0;
}
