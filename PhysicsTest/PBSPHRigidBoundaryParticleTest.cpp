#include "pch.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/RigidBoundaryParticles.h"
#include "../Physics/SoftBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>
#include <memory>

using namespace Phantom::Physics;
using namespace Phantom::Math;

// PBSPH's Two-Way (Track B) boundary-particle contract. Structured to mirror
// WCSPH/DFSPHRigidBoundaryParticleTest.cpp, with one deliberate difference:
// PBSPH's fluid-side quantity is a *position correction* (getDx()), not a
// force, so the "pushed away" assertions read getDx(); only the boundary-side
// reaction (accumForce) is checked as a force, exactly as in WCSPH/DFSPH.

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);

// A single-particle fluid ready for a boundary-particle coupling pass.
std::unique_ptr<PBSPHFluid> makeFluid(const Vector3df& at) {
    auto fluid = std::make_unique<PBSPHFluid>();
    fluid->setRestDensity(1000.f);
    fluid->setStiffness(1.f);
    fluid->setVicsosity(0.f);
    fluid->setEffectLength(1.0f);
    fluid->setIsBoundary(false);
    fluid->createParticle(at, 0.05f);
    return fluid;
}

// A 5x5 puddle in the stable regime used by TwoWayCouplingTest -- dense enough
// that a submerged boundary actually produces a measurable reaction under a
// full simulate() pass (the single-particle makeFluid() fixture above is only
// usable with a hand-set density + direct addBoundaryParticlePressure() call).
std::unique_ptr<PBSPHFluid> makePuddle() {
    auto fluid = std::make_unique<PBSPHFluid>();
    fluid->setRestDensity(1.0f);
    fluid->setStiffness(0.001f);
    fluid->setVicsosity(0.0f);
    fluid->setEffectLength(1.0f);
    fluid->setIsBoundary(false);
    for (int i = 0; i < 5; ++i)
        for (int j = 0; j < 5; ++j)
            fluid->createParticle({ -0.6f + 0.3f * i, -0.3f, -0.6f + 0.3f * j }, 0.15f);
    return fluid;
}
}

TEST(PBSPHRigidBoundaryParticleTest, PenetratingParticle_CorrectionPushesAway_AndReactionAccumulated) {
    auto fluid = makeFluid({ 0.6f, 0.f, 0.f });
    PBSPHParticle particle(fluid->getParticles(), 0, fluid.get());
    particle.setDensity(1500.f);  // density (1500) > rest density (1000) -> positive constraint
    particle.setLambda(0.5f);     // stand-in for calculateLambda() in a full solver pass

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    ASSERT_FALSE(boundary.particles().empty());
    boundary.computePsi(*fluid->getKernel(), fluid->getRestDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    // Particle sits just outside the sphere but within kernel range of nearby
    // boundary particles -> the position correction should push it further
    // away (+x).
    EXPECT_GT(particles[0].getDx().x, 0.f);

    // Newton's third law: the reaction accumulated on the boundary particles
    // points the opposite way.
    float totalBoundaryForceX = 0.f;
    for (auto& bp : boundary.particles()) totalBoundaryForceX += bp.accumForce.x;
    EXPECT_LT(totalBoundaryForceX, 0.f);
}

TEST(PBSPHRigidBoundaryParticleTest, ParticleFarAway_NoCorrection) {
    auto fluid = makeFluid({ 10.f, 0.f, 0.f });
    PBSPHParticle particle(fluid->getParticles(), 0, fluid.get());
    particle.setDensity(1500.f);

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    boundary.computePsi(*fluid->getKernel(), fluid->getRestDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getDx()), 0.f, 1.0e-6f);
}

TEST(PBSPHRigidBoundaryParticleTest, NoRigidBoundaries_IsNoOp) {
    auto fluid = makeFluid({ 0.6f, 0.f, 0.f });
    PBSPHParticle particle(fluid->getParticles(), 0, fluid.get());
    particle.setDensity(1500.f);
    particle.setLambda(0.5f);

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids;  // empty

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getDx()), 0.f, 1.0e-6f);
}

TEST(PBSPHRigidBoundaryParticleTest, SupportsTwoWayCoupling_IsTrue) {
    PBSPHSolver solver;
    EXPECT_TRUE(solver.supportsTwoWayCoupling());
}

// clearRigidBoundaryParticles() must drop only the rigid list -- an
// independently registered soft boundary particle set stays coupled.
TEST(PBSPHRigidBoundaryParticleTest, ClearRigidBoundaryParticles_LeavesSoftBoundariesCoupled) {
    auto fluid = makePuddle();

    RigidBoundaryParticles rigid;
    SphereShape sphere;
    sphere.radius = 0.3f;
    rigid.sample(sphere, 0.15f);
    rigid.computePsi(*fluid->getKernel(), fluid->getRestDensity());
    rigid.sync({ 0.f, -0.15f, 0.f }, kIdentity);   // overlaps the puddle from above

    // A small dense patch of soft boundary particles (a single point contributes
    // too little density to compress this low-contrast puddle).
    SoftMesh mesh;
    mesh.particles.resize(9);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            mesh.particles.positions[i * 3 + j] =
                { -0.15f + 0.15f * i, -0.15f, -0.15f + 0.15f * j };
    SoftBoundaryParticles soft;
    soft.bind(&mesh);
    soft.sync();
    soft.computePsi(*fluid->getKernel(), fluid->getRestDensity());

    PBSPHSolver solver;
    solver.add(fluid.get());
    solver.setExternalForce({ 0.f, 0.f, 0.f });
    solver.setTimeStep(0.005f);
    solver.addRigidBoundaryParticles(&rigid);
    solver.addSoftBoundaryParticles(&soft);

    solver.clearRigidBoundaryParticles();

    for (int step = 0; step < 5; ++step) {
        rigid.clearAccumForce();
        soft.clearAccumForce();
        solver.simulate(0.005f, 3);
    }

    float rigidF = 0.f;
    for (auto& bp : rigid.particles()) rigidF += getLength(bp.accumForce);
    float softF = 0.f;
    for (auto& bp : soft.particles()) softF += getLength(bp.accumForce);

    EXPECT_NEAR(rigidF, 0.f, 1.0e-6f);   // rigid list was cleared -> untouched
    EXPECT_GT(softF, 0.f);               // soft list still coupled
}

// PBSPH runs a fixed maxIter constraint iterations per simulate() call (it has
// no CFL substepping). The accumulated boundary reaction must not blow up
// super-linearly as maxIter grows -- later iterations correct less as the
// fluid is pushed off the wall and its lambda decays (PBSPH's analogue of
// DFSPH's frame-share weighting, internal design notes 1.6).
TEST(PBSPHRigidBoundaryParticleTest, BoundaryReactionDoesNotInflateWithIterationCount) {
    auto reactionForIters = [](int maxIter) {
        auto fluid = makePuddle();

        RigidBoundaryParticles rigid;
        SphereShape sphere;
        sphere.radius = 0.3f;
        rigid.sample(sphere, 0.15f);
        rigid.computePsi(*fluid->getKernel(), fluid->getRestDensity());
        rigid.sync({ 0.f, -0.15f, 0.f }, kIdentity);

        PBSPHSolver solver;
        solver.add(fluid.get());
        solver.setExternalForce({ 0.f, 0.f, 0.f });
        solver.setTimeStep(0.005f);
        solver.addRigidBoundaryParticles(&rigid);

        rigid.clearAccumForce();
        solver.simulate(0.005f, maxIter);

        Vector3df f(0.f, 0.f, 0.f);
        for (auto& bp : rigid.particles()) f += bp.accumForce;
        return f;
    };

    const Vector3df r2  = reactionForIters(2);
    const Vector3df r10 = reactionForIters(10);

    ASSERT_TRUE(std::isfinite(getLength(r2)) && std::isfinite(getLength(r10)));
    ASSERT_GT(getLength(r2), 0.f);   // a real reaction is being accumulated
    // 5x the iterations must not mean >=5x the reaction.
    EXPECT_LT(getLength(r10), 5.0f * getLength(r2));
}
