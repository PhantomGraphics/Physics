#include "pch.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/RigidBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>

// Mirrors DFSPHRigidBoundaryParticleTest.cpp: WCSPH joined DFSPH/PBSPH in
// Phase 4 of docs/todo/PLAN_physics_ownership_and_coupling_unification.md,
// so it must accumulate the same "pushed away, reaction on the boundary"
// behavior via addBoundaryParticlePressure() -- same regression this test
// suite provides for DFSPH.

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
}

TEST(WCSPHRigidBoundaryParticleTest, PenetratingParticle_PushedAway_AndReactionAccumulated) {
    WCSPHFluid fluid;
    fluid.setDensity(1000.f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(0.f);
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f);
    WCSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setKernel(fluid.getKernel());
    particle.addDensity(1500.f);  // density (1500) > rest density (1000) -> positive pressure

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    ASSERT_FALSE(boundary.particles().empty());
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<WCSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    WCSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids);

    // Particle sits just outside the sphere but within kernel range of nearby
    // boundary particles -> should be pushed further away (+x).
    EXPECT_GT(particles[0].getForce().x, 0.f);

    // Newton's third law: the reaction accumulated on the boundary particles
    // points the opposite way.
    float totalBoundaryForceX = 0.f;
    for (auto& bp : boundary.particles()) totalBoundaryForceX += bp.accumForce.x;
    EXPECT_LT(totalBoundaryForceX, 0.f);
}

TEST(WCSPHRigidBoundaryParticleTest, ParticleFarAway_NoForce) {
    WCSPHFluid fluid;
    fluid.setDensity(1000.f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(0.f);
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 10.f, 0.f, 0.f }, 0.05f);
    WCSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setKernel(fluid.getKernel());
    particle.addDensity(1500.f);

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<WCSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    WCSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}

TEST(WCSPHRigidBoundaryParticleTest, NoRigidBoundaries_IsNoOp) {
    WCSPHFluid fluid;
    fluid.setDensity(1000.f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(0.f);
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f);
    WCSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setKernel(fluid.getKernel());
    particle.addDensity(1500.f);

    std::vector<WCSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids;  // empty

    WCSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}

TEST(WCSPHRigidBoundaryParticleTest, SupportsTwoWayCoupling_IsTrue) {
    WCSPHSolver solver;
    EXPECT_TRUE(solver.supportsTwoWayCoupling());
}
