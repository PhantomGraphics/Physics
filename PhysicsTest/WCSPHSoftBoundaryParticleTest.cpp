#include "pch.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/SoftBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>

// Mirrors DFSPHSoftBoundaryParticleTest.cpp -- see WCSPHRigidBoundaryParticleTest.cpp's
// top comment for why WCSPH needs the same coverage as DFSPH here.

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

TEST(WCSPHSoftBoundaryParticleTest, PenetratingParticle_PushedAway_AndReactionAccumulated) {
    WCSPHFluid fluid;
    fluid.setDensity(1000.f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(0.f);
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f);
    WCSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setKernel(fluid.getKernel());
    particle.addDensity(1500.f);  // density (1500) > rest density (1000) -> positive pressure

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());
    ASSERT_FALSE(boundary.particles().empty());

    std::vector<WCSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    WCSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs);

    // Particle sits within kernel range of the soft boundary particle at the
    // origin -> should be pushed further away (+x).
    EXPECT_GT(particles[0].getForce().x, 0.f);

    // Newton's third law: the reaction accumulated on the boundary particle
    // points the opposite way.
    EXPECT_LT(boundary.particles().front().accumForce.x, 0.f);
}

TEST(WCSPHSoftBoundaryParticleTest, ParticleFarAway_NoForce) {
    WCSPHFluid fluid;
    fluid.setDensity(1000.f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(0.f);
    fluid.setEffectLength(1.0f);

    fluid.createParticle({ 10.f, 0.f, 0.f }, 0.05f);
    WCSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setKernel(fluid.getKernel());
    particle.addDensity(1500.f);

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid.getKernel(), fluid.getDensity());

    std::vector<WCSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    WCSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}

TEST(WCSPHSoftBoundaryParticleTest, NoSoftBoundaries_IsNoOp) {
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
    std::vector<IBoundaryParticles*> softs;  // empty

    WCSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs);

    EXPECT_NEAR(getLength(particles[0].getForce()), 0.f, 1.0e-6f);
}
