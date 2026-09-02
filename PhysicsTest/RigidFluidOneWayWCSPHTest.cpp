#include "pch.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/RigidFluidSolver.h"
#include "../Physics/ICollisionShape.h"
#include <cmath>
#include <vector>

using namespace Phantom::Physics;
using namespace Phantom::Math;

// Headless equivalent of PhysicsView's scenarios/50_couple_rigid_fluid_oneway.json
// and scenarios/54_couple_disabled_baseline.json: a WCSPH pool with a 1x1x1 box
// released from y = 3 straight through it, One-Way (RigidBoundary SDF penalty)
// coupling on versus off.
//
// Mirrors FluidWorld::createWCSPH()/refreshCoupling()/stepOnce() (same
// setPressureCoeFromScale() derivation, same uniform-grid seeding at spacing
// 2*radius, same per-frame syncBoundaries -> simulate -> stepForced order), so a
// regression here shows up in those scenarios too, but runs in a second instead
// of needing the Vulkan app.
//
// This is the regression guard for internal design notes 1.7: the
// coupled and uncoupled runs came out bit-for-bit identical, and the cause was
// the *scene*, not the coupling -- see OriginalPoolNeverOverlapsTheBox below.
namespace {

const float kFrameDt  = 0.01f;
const float kGravityY = -9.8f;

struct BoxDropScene {
    // Defaults reproduce the original scenarios/54 configuration exactly:
    // FluidWorld::Params radius 1.0 (seed spacing 2.0), the scenario's own
    // SetFluidEffectLength:1.0 / SetFluidPressureCoeScale:0.5 /
    // SetFluidDensity:1.0, and SetFluidBounds:-3:0:-3:3:2:3.
    float    radius           = 1.0f;
    float    effectLength     = 1.0f;
    float    pressureCoeScale = 0.5f;
    Vector3df boundsMin       = { -3.f, 0.f, -3.f };
    Vector3df boundsMax       = {  3.f, 2.f,  3.f };

    WCSPHFluid       fluid;
    WCSPHSolver      solver;
    RigidFluidSolver world;
    BoxShape         box;         // ScenePreset::BoxDrop's half extents
    RigidBody        body;

    void build(bool coupled) {
        // FluidWorld::createWCSPH(): effectLength must precede
        // setPressureCoeFromScale(), which reads it.
        fluid.setEffectLength(effectLength);
        fluid.setDensity(1.0f);
        fluid.setVicosityCoe(5.0f);   // FluidWorld::Params::viscosity
        fluid.setTensionCoe(0.0f);
        fluid.setPressureCoeFromScale(pressureCoeScale);

        const float diameter = radius * 2.0f;
        for (float z = boundsMin.z; z <= boundsMax.z + 1e-4f; z += diameter)
            for (float y = boundsMin.y; y <= boundsMax.y + 1e-4f; y += diameter)
                for (float x = boundsMin.x; x <= boundsMax.x + 1e-4f; x += diameter)
                    fluid.createParticle(Vector3df(x, y, z), radius);

        solver.setEffectLength(effectLength);
        solver.setTimeStep(kFrameDt);
        // The scenario's SetFluidBoundary:-1000:0:-1000:1000:1000:1000.
        solver.setBoundary(Box3df(Vector3df(-1000.f, 0.f, -1000.f),
                                  Vector3df(1000.f, 1000.f, 1000.f)), kFrameDt);
        solver.setExternalForce(Vector3df(0.f, kGravityY, 0.f));
        solver.add(&fluid);

        // ScenePreset::BoxDrop's dynamic body (the preset's static floor plane
        // is excluded from coupling by FluidWorld::refreshCoupling() anyway, and
        // the fluid's own domain floor above already stops the pool at y = 0).
        box.halfExtents = { 0.5f, 0.5f, 0.5f };
        body.position = { 0.f, 3.f, 0.f };
        body.setShape(&box);
        body.setMass(1.0f);
        world.rigidWorld().addBody(&body);
        world.rigidWorld().params().gravity = { 0.f, kGravityY, 0.f };
        world.rigidWorld().saveSnapshot();
        world.rigidWorld().setRunning(true);

        if (!coupled) return;

        // FluidWorld::refreshCoupling()'s One-Way branch, verbatim.
        auto& binding = world.bind(&body, &box, CouplingMode::OneWay);
        solver.addRigidBoundary(&binding.boundary);
    }

    // One FluidWorld::stepOnce() frame.
    void step() {
        world.syncBoundaries();
        solver.simulate(kFrameDt, 3);
        world.stepForced(kFrameDt);
    }

    void run(int frames) { for (int i = 0; i < frames; ++i) step(); }

    float maxParticleY() const {
        float m = -1.0e30f;
        for (const auto& p : fluid.getParticles().positions) m = std::max(m, p.y);
        return m;
    }

    // Smallest signed distance from the box over every particle: < 0 means at
    // least one particle is inside the shape and RigidBoundary::getBoundaryForce()
    // returns something other than the zero vector.
    float minSignedDistanceToBox() const {
        float d = 1.0e30f;
        for (const auto& p : fluid.getParticles().positions)
            d = std::min(d, box.getSignedDistance(p, body.position, body.orientation));
        return d;
    }

    bool allParticlesFinite() const {
        for (const auto& p : fluid.getParticles().positions)
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) return false;
        return true;
    }
};

// The corrected pool: particle radius 0.3 (seed spacing 0.6) over
// x/z in [-1.2, 1.2], which puts particles on the x = 0 / z = 0 axes and
// therefore inside the box's 1x1x1 drop column. (BoxDropScene holds a
// WCSPHFluid, which is UnCopyable, so this configures in place rather than
// returning a scene by value.)
void makeOverlapping(BoxDropScene& scene) {
    scene.radius           = 0.3f;
    scene.effectLength     = 0.6f;
    scene.pressureCoeScale = 98.0f;
    scene.boundsMin        = { -1.2f, 0.f, -1.2f };
    scene.boundsMax        = {  1.2f, 1.2f,  1.2f };
}

} // namespace

// Root cause of internal design notes 1.7. The original scene seeds the
// pool on a 2.0 grid from x = -3 (so x, z in {-3, -1, 1, 3}), while the box is
// 1x1x1 centred on x = z = 0: the nearest particle column misses the box by
// 0.5 in each of x and z, at every height. RigidBoundary::getBoundaryForce()
// only acts on particles *inside* the shape, so the penalty was identically zero
// on every particle of every frame -- which is why coupled and uncoupled runs
// agreed to the last digit. The coupling was never actually exercised.
TEST(RigidFluidOneWayWCSPHTest, OriginalPoolNeverOverlapsTheBox) {
    BoxDropScene scene;
    scene.build(/*coupled=*/true);

    float minDist = scene.minSignedDistanceToBox();
    for (int i = 0; i < 150; ++i) {
        scene.step();
        minDist = std::min(minDist, scene.minSignedDistanceToBox());
    }

    ASSERT_TRUE(scene.allParticlesFinite());
    EXPECT_GT(minDist, 0.f);
}

// ... and with the pool seeded so that it does populate the box's drop column,
// the box passes right through it, so the penalty boundary is genuinely active.
TEST(RigidFluidOneWayWCSPHTest, OverlappingPoolIsPenetratedByTheBox) {
    BoxDropScene scene;
    makeOverlapping(scene);
    scene.build(/*coupled=*/true);

    float minDist = 1.0e30f;
    for (int i = 0; i < 150; ++i) {
        scene.step();
        minDist = std::min(minDist, scene.minSignedDistanceToBox());
    }

    ASSERT_TRUE(scene.allParticlesFinite());
    EXPECT_LT(minDist, 0.f);
}

// The actual contract 54_couple_disabled_baseline.json was written to prove:
// on a scene the box really passes through, One-Way coupling has to lift the
// fluid measurably above the uncoupled pass-through baseline.
TEST(RigidFluidOneWayWCSPHTest, CoupledPoolIsPushedHigherThanUncoupled) {
    BoxDropScene coupled;
    makeOverlapping(coupled);
    coupled.build(/*coupled=*/true);
    coupled.run(150);

    BoxDropScene control;
    makeOverlapping(control);
    control.build(/*coupled=*/false);
    control.run(150);

    ASSERT_TRUE(coupled.allParticlesFinite());
    ASSERT_TRUE(control.allParticlesFinite());
    EXPECT_GT(coupled.maxParticleY(), control.maxParticleY());
}

// One-Way means exactly that: the fluid never pushes back, so the box's
// trajectory must be bit-for-bit the free-fall one either way. (This is the
// half of the original observation that *was* correct behaviour.)
TEST(RigidFluidOneWayWCSPHTest, BoxTrajectoryIsUnaffectedByTheFluid) {
    BoxDropScene coupled;
    makeOverlapping(coupled);
    coupled.build(/*coupled=*/true);
    coupled.run(150);

    BoxDropScene control;
    makeOverlapping(control);
    control.build(/*coupled=*/false);
    control.run(150);

    EXPECT_FLOAT_EQ(coupled.body.position.y, control.body.position.y);
}
