#include "pch.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/SoftBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
SoftMesh makeSingleParticleMesh(const Vector3df& pos) {
    SoftMesh mesh;
    mesh.particles.resize(1);
    mesh.particles.positions[0] = pos;
    return mesh;
}
}

TEST(DFSPHSoftBoundaryParticleTest, PenetratingParticle_PushedAway_AndReactionAccumulated) {
    DFSPHFluid fluid;
    fluid.density      = 1000.f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f, 1.0f);
    DFSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.addDensity(1500.f);  // density (1500) > rest density (1000) -> positive pressure

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());
    ASSERT_FALSE(boundary.particles().empty());

    std::vector<DFSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    DFSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    // Particle sits within kernel range of the soft boundary particle at the
    // origin -> should be pushed further away (+x).
    EXPECT_GT(particles[0].getForce().x, 0.f);

    // Newton's third law: the reaction accumulated on the boundary particle
    // points the opposite way.
    EXPECT_LT(boundary.particles().front().accumForce.x, 0.f);
}

TEST(DFSPHSoftBoundaryParticleTest, ParticleFarAway_NoForce) {
    DFSPHFluid fluid;
    fluid.density      = 1000.f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 10.f, 0.f, 0.f }, 0.05f, 1.0f);
    DFSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.addDensity(1500.f);

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());

    std::vector<DFSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    DFSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}

TEST(DFSPHSoftBoundaryParticleTest, NoSoftBoundaries_IsNoOp) {
    DFSPHFluid fluid;
    fluid.density      = 1000.f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f, 1.0f);
    DFSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.addDensity(1500.f);

    std::vector<DFSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs;  // empty

    DFSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}
