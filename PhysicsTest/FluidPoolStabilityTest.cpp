#include "pch.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/SphereBoundary.h"
#include <cmath>
#include <cstdio>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
bool isFinite3(const Vector3df& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
}

// A small 3x3x3 block of CSPH particles, dropped under gravity inside a box
// boundary, should settle near the floor and stay bounded/finite -- this is
// the pure fluid-fluid + plane-boundary baseline, with no RigidBoundary/
// RigidBoundaryParticles involved at all.
//
// NOTE: this is the only case from the former FluidPoolStabilityTest kept
// here rather than moved to a PhysicsView scenario. It used to be that
// switching PhysicsView to SetSimulationType:CSPH (WCSPH) and calling Reset
// crashed with "[VKG] Cannot open SPIR-V file" (fixed in 5c44679: the
// FluidView->PhysicsView rename had left PhysicsView.vcxproj's
// PostBuildEvent copying shaders from the old FluidView path, so
// fluid_point/line shaders were never copied to $(OutDir)shaders, and
// SetSimulationType:CSPH didn't accept the "CSPH" string in the first
// place). A scenario replacement is possible now but hasn't been done; this
// GoogleTest stays as the fast regression check.
TEST(FluidPoolStabilityTest, WCSPH_PoolSettlesInBoxWithoutExploding) {
    WCSPHFluid fluid;
    fluid.setDensity(1000.0f);
    fluid.setPressureCoe(200.0f);
    fluid.setVicosityCoe(0.05f);
    fluid.setEffectLength(0.1f);
    fluid.setStatic(false);

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                fluid.createParticle(Vector3df(-0.05f + 0.05f * i, 0.3f + 0.05f * j, -0.05f + 0.05f * k), 0.001f);

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver.setTimeStep(0.005f);
    solver.setEffectLength(0.1f);
    solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.01f);

    for (int step = 0; step < 200; ++step) {
        solver.simulate(0.005f, 1);
        for (size_t i = 0; i < fluid.getParticles().size(); ++i) {
            ASSERT_TRUE(isFinite3(fluid.getParticles().positions[i])) << "step " << step;
            ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[i])) << "step " << step;
        }
    }

    for (const auto& pos : fluid.getParticles().positions) {
        EXPECT_GT(pos.y, -0.6f);
        EXPECT_LT(pos.y, 0.6f);
    }
}

// The water-sphere showcase's core failure mode: a pool resting against a
// *curved* wall for the whole shot (docs/todo/PLAN_sph_showcase_water_sphere.md
// sections 2.2, 2.4 and 11-2). Two independent things have to hold at once:
//
//   1. The sphere wall contributes density, not just a penalty force. A wall
//      that only pushes leaves its adjacent particles at rho < rho0 forever,
//      so their pressure is pinned at zero (p = k*max(0, rho-rho0)), nothing
//      resists the column above them, and they get crushed into each other
//      until the solve goes non-finite. That is exactly why the existing
//      MeshBoundaryShape/RigidBoundary path could not be reused here.
//   2. pressure_coe_scale = 20 (rather than the 1.96 default) keeps the
//      hydrostatic compression of a deep column small.
//
// Geometry is the design's R >= 10h floor at 1/25 of the production particle
// count, which keeps this a seconds-long unit test. The production-depth
// version of check 2 (D = 0.31 m = 31h, where the 1.96 default would sink the
// pool by 15.5%) is the preview-tier Blender run in section 11-2, not
// something worth spending a hundred thousand particles on here.
TEST(FluidPoolStabilityTest, WCSPH_DeepPoolRestsInsideSphereContainer) {
    const float particleRadius = 0.005f;
    const float spacing = particleRadius * 2.0f;
    const float effectLength = particleRadius * 4.0f;   // 0.02
    const float sphereRadius = effectLength * 10.0f;    // 0.20 == the design's R >= 10h floor
    const float restDensity = 1000.0f;
    const float fillZ = -sphereRadius * 0.5f;           // pool depth 0.10 == 5h
    const float timeStep = 2.0e-4f;                     // CFL: c = sqrt(20*h*rho0) = 20 m/s
    const Vector3df center(0.0f, 0.0f, 0.0f);

    WCSPHFluid fluid;
    fluid.setDensity(restDensity);
    fluid.setPressureCoeFromScale(20.0f);
    fluid.setVicosityCoe(0.5f * effectLength);
    fluid.setTensionCoe(0.0f);
    fluid.setEffectLength(effectLength);
    fluid.setStatic(false);

    // Fill the bottom of the sphere on a regular grid, stopping half a spacing
    // short of the wall so no particle starts already penetrating it.
    const float innerRadius = sphereRadius - spacing * 0.5f;
    const int n = static_cast<int>(sphereRadius / spacing);
    for (int i = -n; i <= n; ++i) {
        for (int j = -n; j <= n; ++j) {
            for (int k = -n; k <= n; ++k) {
                const Vector3df pos(i * spacing, j * spacing, k * spacing);
                if (pos.z > fillZ) continue;
                if (getDistance(pos, center) > innerRadius) continue;
                fluid.createParticle(pos, particleRadius);
            }
        }
    }
    ASSERT_GT(fluid.getNumParticles(), 3000);

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, 0.0f, -9.8f));
    solver.setTimeStep(timeStep);
    solver.setEffectLength(effectLength);
    // The box is deliberately far larger than the sphere, so it never engages:
    // the sphere alone holds the pool. This mirrors the showcase scene, where
    // the domain box exists only because build_simulation() requires one.
    solver.setBoundary(Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f)), timeStep);
    solver.setBoundarySpheres(
        { SphereBoundary(center, sphereRadius, effectLength * 2.0f) }, timeStep);

    for (int step = 0; step < 200; ++step) {
        solver.simulate(timeStep, 1);
        for (size_t i = 0; i < fluid.getParticles().size(); ++i) {
            ASSERT_TRUE(isFinite3(fluid.getParticles().positions[i])) << "step " << step;
            ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[i])) << "step " << step;
        }
    }

    // Nothing leaks through the curved wall. The penalty force decelerates a
    // penetrating particle rather than hard-clamping it, so allow it to
    // overshoot by a fraction of a support radius.
    for (const auto& pos : fluid.getParticles().positions) {
        EXPECT_LE(getDistance(pos, center), sphereRadius + effectLength * 0.5f);
    }

    // Mean density is measured over interior particles only -- one support
    // radius clear of both the free surface and the wall. SPH density at a
    // free surface is deficient by construction (there are no neighbors above
    // it), so averaging over every particle would measure the surface, not the
    // compression this test is about.
    double sum = 0.0;
    int counted = 0;
    for (size_t i = 0; i < fluid.getParticles().size(); ++i) {
        const Vector3df& pos = fluid.getParticles().positions[i];
        if (pos.z > fillZ - effectLength) continue;
        if (getDistance(pos, center) > sphereRadius - effectLength) continue;
        sum += WCSPHParticle(fluid.getParticles(), i, &fluid).getDensity();
        ++counted;
    }
    ASSERT_GT(counted, 500);
    const double meanDensity = sum / counted;
    EXPECT_NEAR(meanDensity, restDensity, 0.03 * restDensity);
}
