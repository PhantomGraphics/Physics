#include "pch.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/SphereBoundary.h"
#include "../Physics/Emitter.h"
#include <cmath>
#include <cstdio>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

// The two resolutions the showcase ships at, as
// Physics/PhysicsView/scenarios/showcase/showcase_water_sphere{,_preview}.json
// configure them. viscosityCoe is part of the tier because getting it wrong per
// tier is exactly what kept the production tier unstable after the 2026-08-26
// fixes -- see docs/issue/water_sphere_showcase_emitter_instability.md
// section 11.
struct ShowcaseTier {
    float particleRadius;
    float effectLength;
    float timeStep;
    float viscosityCoe;
};

constexpr ShowcaseTier kPreviewTier{ 0.005f,  0.02f, 2.0e-4f, 0.010f };
constexpr ShowcaseTier kProductionTier{ 0.0025f, 0.01f, 1.5e-4f, 0.010f };

// A scaled-down stand-in for the water-sphere showcase's opening shot: a jet
// pouring from an emitter into a *dry* spherical container
// (docs/issue/water_sphere_showcase_emitter_instability.md). Same particle
// spacing, effect length, time step, stiffness, viscosity and jet speed as the
// scenario for the given tier -- only the container (R = 0.20 = the design's
// 10h floor), the drop height and the jet's cross-section are shrunk, which
// keeps the particle count manageable without changing any of the physics
// under test.
struct JetIntoDrySphere {
    static constexpr float sphereRadius = 0.20f;
    static constexpr float emitterY = 0.10f;
    static constexpr float jetRadius = 0.03f;
    static constexpr float jetSpeed = 2.45f;
    static constexpr float gravity = 9.8f;

    ShowcaseTier tier = kPreviewTier;
    WCSPHFluid fluid;
    WCSPHSolver solver;

    // Fastest a particle can possibly be moving: it leaves the emitter at
    // jetSpeed and falls to the bottom of the sphere. Anything above this is
    // kinetic energy the solver invented.
    static float ballisticLimit() {
        return std::sqrt(jetSpeed * jetSpeed + 2.0f * gravity * (emitterY + sphereRadius));
    }

    void build(const float boundaryDampingRatio, const ShowcaseTier& t = kPreviewTier) {
        tier = t;
        const float spacing = t.particleRadius * 2.0f;
        fluid.setDensity(1000.0f);
        fluid.setEffectLength(t.effectLength);
        fluid.setPressureCoeFromScale(20.0f);
        fluid.setVicosityCoe(t.viscosityCoe);
        fluid.setTensionCoe(0.0f);
        fluid.setStatic(false);
        fluid.setMaxParticles(40000);

        Emitter e;
        e.center = Vector3df(0.0f, emitterY, 0.0f);
        e.radius = jetRadius;
        e.particleRadius = t.particleRadius;
        e.direction = Vector3df(0.0f, -1.0f, 0.0f);
        e.speed = jetSpeed;
        // Deliberately 0, unlike the showcase's own 0.02: WCSPHFluid seeds its
        // jitter RNG from std::random_device, so any non-zero jitter makes this
        // scene unrepeatable run to run (measured spread on the metrics below:
        // ~10% on peak speed, ~5% on rho_max). With jitter off the run is
        // bit-identical, which is what lets these thresholds be tight.
        e.speedJitter = 0.0f;
        // The rate a correctly configured emitter needs: one particle per
        // spacing^3 of jet volume per second (see Emitter.h).
        e.rate = 3.14159265f * jetRadius * jetRadius * jetSpeed / (spacing * spacing * spacing);
        fluid.addEmitter(e);

        solver.add(&fluid);
        solver.setExternalForce(Vector3df(0.0f, -gravity, 0.0f));
        solver.setTimeStep(t.timeStep);
        solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), t.timeStep);
        solver.setBoundarySpheres(
            { SphereBoundary(Vector3df(0.0f, 0.0f, 0.0f), sphereRadius, t.effectLength * 2.0f) },
            t.timeStep);
        solver.setBoundaryDampingRatio(boundaryDampingRatio);
    }

    void step() {
        fluid.updateEmitters(tier.timeStep);
        solver.simulate(tier.timeStep, 1);
    }

    float maxSpeed() const {
        float m = 0.0f;
        for (const auto& v : fluid.getParticles().velocities) m = std::max(m, glm::length(v));
        return m;
    }

    // Highest point reached by anything that has left the falling column
    // (moving sideways, or no longer falling) -- i.e. how far the splash
    // climbs back toward the emitter.
    float splashTopY() const {
        float top = -sphereRadius;
        const auto& soa = fluid.getParticles();
        for (size_t i = 0; i < soa.size(); ++i) {
            const auto& v = soa.velocities[i];
            const float horizontal = std::sqrt(v.x * v.x + v.z * v.z);
            if (horizontal > 0.5f || v.y > -1.0f) top = std::max(top, soa.positions[i].y);
        }
        return top;
    }
};

} // namespace

// The showcase's actual failure mode, as a bounded assertion: a jet landing in
// a dry sphere must not come back out faster than it went in.
//
// Before the two fixes this test covers -- WCSPHSolver::addBoundaryDensity()
// clamping the wall's density contribution to the particle's remaining
// headroom, and setBoundaryDampingRatio() taking the restitution off the wall
// penalty -- the first layer to reach the floor was handed rho ~ 1.5*rho0
// (a full fluid neighborhood from the column ramming into it, *plus* the
// wall's unconditional half-space share) and was blown back up at 2.4x its own
// ballistic limit. In the full-size scene that spray reached the emitter
// around frame 30 and wrecked the shot from there on.
TEST(WaterSphereEmitterStabilityTest, JetIntoDrySphereDoesNotOutrunItsOwnFall) {
    JetIntoDrySphere scene;
    scene.build(0.35f);

    const float limit = JetIntoDrySphere::ballisticLimit();
    float peakSpeed = 0.0f;
    bool landed = false;
    for (int step = 0; step < 700; ++step) {
        scene.step();
        // Only look after the jet has actually reached the floor; the fall
        // itself is uneventful by construction.
        if (!landed) {
            landed = scene.splashTopY() > -JetIntoDrySphere::sphereRadius;
            continue;
        }
        peakSpeed = std::max(peakSpeed, scene.maxSpeed());
    }
    ASSERT_TRUE(landed) << "the jet never reached the sphere floor -- scene is misconfigured";
    ASSERT_GT(scene.fluid.getNumParticles(), 500);

    // The discriminating assertion. Some overshoot is expected -- the impact
    // is a genuine shock and WCSPH resolves it with a compressible spike -- so
    // this is not asserting 1.0. Measured over the window this test runs:
    // 1.03x as it stands, 1.39x with addBoundaryDensity()'s headroom clamp
    // reverted to the old unconditional restDensity cap.
    EXPECT_LT(peakSpeed, 1.20f * limit) << "ballistic limit " << limit;

    // The symptom the showcase itself suffers from, as a coarse invariant: the
    // splash stays in the lower half of the drop. It takes a few hundred more
    // steps than this test runs for the two cases to separate on *this* metric
    // (the spray has to climb), so it is a guard rather than the
    // discriminator -- but it is the quantity that matters in the real shot,
    // which is why it is asserted here rather than left to the doc.
    const float midDrop = 0.5f * (JetIntoDrySphere::emitterY - JetIntoDrySphere::sphereRadius);
    EXPECT_LT(scene.splashTopY(), midDrop);
}

// The same assertion at the *production* resolution (radius 0.0025, h 0.01,
// dt 1.5e-4), which is the tier the showcase actually bakes at.
//
// This is not redundant with the test above. The 2026-08-26 fixes were only
// ever measured at the preview tier, and they do not carry over on their own:
// with the viscosityCoe the preset used to halve along with the radius
// (0.010 -> 0.005), this scene peaks at 1.53x its ballistic limit and the
// spray is past the emitter plane by frame 14, even with the headroom clamp
// and boundary damping in place. viscosityCoe behaves as a kinematic viscosity
// (m^2/s) -- the same value produces the same damping at either resolution
// (measured: velocity decay agrees within ~5% across a 2x resolution change on
// an identical physical scene) -- so it must NOT be rescaled with the radius.
// See docs/issue/water_sphere_showcase_emitter_instability.md section 11.
TEST(WaterSphereEmitterStabilityTest, ProductionTierJetDoesNotOutrunItsOwnFall) {
    JetIntoDrySphere scene;
    scene.build(0.35f, kProductionTier);

    const float limit = JetIntoDrySphere::ballisticLimit();
    float peakSpeed = 0.0f;
    bool landed = false;
    // 30 frames x 33 substeps, matching showcase_water_sphere.json's frame
    // length. The jet needs ~18 of those frames just to fall; the
    // misconfigured case is past the emitter plane by frame 14 (it blows up
    // before the column has even landed), and the correct one is still
    // climbing out of the impact at frame 30.
    for (int step = 0; step < 30 * 33; ++step) {
        scene.step();
        if (!landed) {
            landed = scene.splashTopY() > -JetIntoDrySphere::sphereRadius;
            continue;
        }
        peakSpeed = std::max(peakSpeed, scene.maxSpeed());
    }
    ASSERT_TRUE(landed) << "the jet never reached the sphere floor -- scene is misconfigured";
    ASSERT_GT(scene.fluid.getNumParticles(), 4000);

    // Measured: 1.00x as it stands, 1.53x with viscosityCoe put back to 0.005.
    EXPECT_LT(peakSpeed, 1.20f * limit) << "ballistic limit " << limit;

    // The symptom itself: nothing thrown back up as far as the emitter.
    // Measured: -0.08 as it stands, +0.15 (i.e. above the emitter) at 0.005.
    EXPECT_LT(scene.splashTopY(), JetIntoDrySphere::emitterY);
}
