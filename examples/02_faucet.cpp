// 02_faucet.cpp
// ============================================================================
// A running faucet, simulated with WCSPH.
//
// Shows the two optional per-step hooks that turn a closed simulation into an
// open one:
//   * emitters        -- spawn new particles over time   (fluid.updateEmitters)
//   * outflow regions -- delete particles that enter a box (fluid.removeOutflowParticles)
//
// A jet pours down from an emitter, pools in the basin, and drains through an
// outflow region just below the floor, so the particle count climbs and then
// levels off as inflow and outflow balance.
//
//     ./sph_example_02_faucet  [output_dir]
// ============================================================================

#include "WCSPHFluid.h"
#include "WCSPHSolver.h"
#include "Emitter.h"
#include "OutflowRegion.h"

#include "sph_examples_common.h"

#include <cstdio>
#include <string>

using namespace sph_examples;
using Phantom::Physics::WCSPHFluid;
using Phantom::Physics::WCSPHSolver;
using Phantom::Physics::Emitter;
using Phantom::Physics::OutflowRegion;

int main(int argc, char** argv)
{
    const std::string outDir = (argc > 1) ? argv[1] : "out_faucet";

    // Same known-stable small-scale regime as example 01.
    const float radius       = 0.15f;
    const float effectLength = 1.0f;
    const float restDensity  = 1.0f;
    const float dt           = 0.005f;

    const Box3df container(Vector3df(-3.f, -3.f, -3.f), Vector3df(3.f, 4.f, 3.f));

    // ---- Fluid (starts empty -- the emitter fills it) ------------------
    WCSPHFluid fluid;
    fluid.setDensity(restDensity);
    fluid.setEffectLength(effectLength);
    fluid.setPressureCoeFromScale(1960.0f);
    fluid.setVicosityCoe(0.5f);
    fluid.setTensionCoe(0.0f);
    fluid.setMaxParticles(4000);               // hard cap the emitter respects

    // ---- Emitter: a downward jet from above --------------------------
    Emitter faucet;
    faucet.center         = Vector3df(0.f, 2.5f, 0.f);
    faucet.radius         = 0.4f;              // radius of the circular nozzle
    faucet.particleRadius = radius;            // MUST match the scene's radius --
                                              // SPH mass is derived from it
    faucet.direction      = Vector3df(0.f, -1.f, 0.f);
    faucet.speed          = 1.5f;
    faucet.speedJitter    = 0.05f;
    faucet.rate           = 250.f;             // particles per second
    fluid.addEmitter(faucet);

    // ---- Outflow region: a drain slot at the bottom of the basin -----
    OutflowRegion drain;
    drain.bounds = Box3df(Vector3df(-0.6f, -3.0f, -0.6f), Vector3df(0.6f, -2.6f, 0.6f));
    fluid.addOutflowRegion(drain);

    // ---- Solver ---------------------------------------------------
    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setEffectLength(effectLength);
    solver.setTimeStep(dt);
    solver.setExternalForce(Vector3df(0.f, -9.8f, 0.f));
    solver.setBoundary(container, dt);

    std::printf("faucet: emitting into '%s/' (drain removes particles below y=-2.6)\n",
                outDir.c_str());

    const int totalSteps    = 600;
    const int stepsPerFrame = 4;
    int frame = 0;
    Stopwatch clock;

    for (int step = 1; step <= totalSteps; ++step) {
        // Order matters -- grow the system, step it, then prune it. This is
        // the same sequence PhysicsView's FluidWorld::stepFluidOnly() uses.
        fluid.updateEmitters(dt);
        solver.simulate(dt, 4);
        fluid.removeOutflowParticles();

        if (step % stepsPerFrame == 0)
            writePLYFrame(outDir, "faucet", frame++, solver.getParticlePositions());

        if (step % 100 == 0)
            std::printf("  step %4d  t=%4.2fs  particles=%5d  maxSpeed=%5.2f\n",
                        step, step * dt, solver.getParticleCount(),
                        maxSpeed(solver.getParticleVelocities()));
    }

    std::printf("done: %d frames, %d particles resident, %.2fs wall time\n",
                frame, solver.getParticleCount(), clock.seconds());
    return 0;
}
