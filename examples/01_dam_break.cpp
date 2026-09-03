// 01_dam_break.cpp
// ============================================================================
// The smallest useful Phantom::Physics program: a WCSPH "dam break".
//
// A tall, narrow column of fluid stands against one wall of a box and is
// released. It collapses under gravity, sweeps across the floor and sloshes
// up the far wall. Every few steps the particle positions are written to a
// numbered PLY file so the run can be inspected in any point-cloud viewer
// (Blender, MeshLab, CloudCompare, Houdini, ...). No window, no GPU.
//
//     ./sph_example_01_dam_break  [output_dir]
// ============================================================================

#include "WCSPHFluid.h"
#include "WCSPHSolver.h"

#include "sph_examples_common.h"

#include <cstdio>
#include <string>

using namespace sph_examples;
using Phantom::Physics::WCSPHFluid;
using Phantom::Physics::WCSPHSolver;

int main(int argc, char** argv)
{
    const std::string outDir = (argc > 1) ? argv[1] : "out_dam_break";

    // ---- Scene scale ----------------------------------------------------
    // Phantom's SPH types carry no length unit. These numbers are the
    // "known-stable" small-scale regime the scenario test-suite uses for
    // WCSPH (Physics/PhysicsView/scenarios/12_fluid_wcsph_pool_settle.json):
    // particle radius 0.15, kernel support 1.0, time step 0.005 s.
    const float radius       = 0.15f;
    const float spacing      = radius * 2.0f;   // rest-density lattice spacing
    const float effectLength = 1.0f;            // kernel support radius
    const float restDensity  = 1.0f;
    const float viscosityCoe = 0.5f;
    const float dt           = 0.005f;          // seconds per step

    const Box3df container(Vector3df(-3.f, -3.f, -3.f), Vector3df(3.f, 3.f, 3.f));
    // A tall column flush against the left (-x) wall -- the "dam".
    const Box3df column   (Vector3df(-2.9f, -2.9f, -0.9f), Vector3df(-1.7f, 2.4f, 0.9f));

    // ---- Build the fluid ----------------------------------------------
    // The fluid OWNS its particle storage and material parameters. The solver
    // is a separate object that steps one or more fluids and holds only
    // non-owning pointers, so the fluid must outlive the solver (here they
    // simply share this scope).
    WCSPHFluid fluid;
    fluid.setDensity(restDensity);
    fluid.setEffectLength(effectLength);        // also (re)builds the SPH kernel
    // WCSPH's pressure stiffness is scale-dependent; derive it from the kernel
    // support rather than hand-tuning a raw number (see WCSPHFluid docs).
    fluid.setPressureCoeFromScale(1960.0f);
    fluid.setVicosityCoe(viscosityCoe);
    fluid.setTensionCoe(0.0f);

    for (const auto& p : seedBox(column, spacing))
        fluid.createParticle(p, radius);        // 2nd arg is the particle radius

    // ---- Build the solver -------------------------------------------
    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setEffectLength(effectLength);       // mirror onto the solver
    solver.setTimeStep(dt);
    solver.setExternalForce(Vector3df(0.f, -9.8f, 0.f));      // gravity
    // The domain container: internally 6 inward-facing planes that push
    // particles back in with a penalty force.
    solver.setBoundary(container, dt);
    solver.setBoundaryDampingRatio(0.2f);       // dissipate wall-impact energy

    std::printf("dam break: %d particles -> frames in '%s/'\n",
                fluid.getNumParticles(), outDir.c_str());

    // ---- Simulate ----------------------------------------------------
    const int totalSteps    = 600;             // 3.0 s of simulated time
    const int stepsPerFrame = 4;               // one PLY every 0.02 s
    int frame = 0;
    Stopwatch clock;

    writePLYFrame(outDir, "dam", frame++, solver.getParticlePositions());   // t = 0

    for (int step = 1; step <= totalSteps; ++step) {
        // Advance one frame. The 2nd argument is the constraint-iteration
        // budget; WCSPH is single-pass and ignores it (DFSPH / PBSPH use it).
        solver.simulate(dt, 4);

        if (step % stepsPerFrame == 0) {
            const auto pos = solver.getParticlePositions();
            const auto vel = solver.getParticleVelocities();
            writePLYFrame(outDir, "dam", frame++, pos);

            if (step % 40 == 0)
                std::printf("  step %4d  t=%4.2fs  minY=%6.2f  maxSpeed=%5.2f\n",
                            step, step * dt, minY(pos), maxSpeed(vel));
        }
    }

    std::printf("done: %d frames in %.2fs wall time\n", frame, clock.seconds());
    return 0;
}
