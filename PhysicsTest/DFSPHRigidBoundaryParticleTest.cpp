#include "pch.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/RigidBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
}

TEST(DFSPHRigidBoundaryParticleTest, PenetratingParticle_PushedAway_AndReactionAccumulated) {
    DFSPHFluid fluid;
    fluid.density      = 1000.f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f, 1.0f);
    DFSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.addDensity(1500.f);  // density (1500) > rest density (1000) -> positive pressure

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    ASSERT_FALSE(boundary.particles().empty());
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<DFSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    DFSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    // Particle sits just outside the sphere but within kernel range of nearby
    // boundary particles -> should be pushed further away (+x).
    EXPECT_GT(particles[0].getForce().x, 0.f);

    // Newton's third law: the reaction accumulated on the boundary particles
    // points the opposite way.
    float totalBoundaryForceX = 0.f;
    for (auto& bp : boundary.particles()) totalBoundaryForceX += bp.accumForce.x;
    EXPECT_LT(totalBoundaryForceX, 0.f);
}

TEST(DFSPHRigidBoundaryParticleTest, ParticleFarAway_NoForce) {
    DFSPHFluid fluid;
    fluid.density      = 1000.f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 10.f, 0.f, 0.f }, 0.05f, 1.0f);
    DFSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.addDensity(1500.f);

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<DFSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    DFSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}

TEST(DFSPHRigidBoundaryParticleTest, NoRigidBoundaries_IsNoOp) {
    DFSPHFluid fluid;
    fluid.density      = 1000.f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f, 1.0f);
    DFSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.addDensity(1500.f);

    std::vector<DFSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids;  // empty

    DFSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}
