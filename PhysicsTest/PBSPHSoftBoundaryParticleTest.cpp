#include "pch.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
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

TEST(PBSPHSoftBoundaryParticleTest, PenetratingParticle_CorrectionPushesAway_AndReactionAccumulated) {
    PBSPHFluid fluid;
    fluid.setRestDensity(1000.f);
    fluid.setStiffness(1.f);
    fluid.setVicsosity(0.f);
    fluid.setEffectLength(1.0f);
    fluid.setIsBoundary(false);

    fluid.createParticle({ 0.6f, 0.f, 0.f }, 0.05f);
    PBSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setDensity(1500.f);  // density (1500) > rest density (1000) -> positive constraint
    particle.setLambda(0.5f);     // stand-in for calculateLambda() in a full solver pass

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid.getKernel(), fluid.getRestDensity());
    ASSERT_FALSE(boundary.particles().empty());

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    // Particle sits within kernel range of the soft boundary particle at the
    // origin -> the position correction should push it further away (+x).
    EXPECT_GT(particles[0].getDx().x, 0.f);

    // Newton's third law: the reaction accumulated on the boundary particle
    // points the opposite way.
    EXPECT_LT(boundary.particles().front().accumForce.x, 0.f);
}

TEST(PBSPHSoftBoundaryParticleTest, ParticleFarAway_NoCorrection) {
    PBSPHFluid fluid;
    fluid.setRestDensity(1000.f);
    fluid.setStiffness(1.f);
    fluid.setVicsosity(0.f);
    fluid.setEffectLength(1.0f);
    fluid.setIsBoundary(false);

    fluid.createParticle({ 10.f, 0.f, 0.f }, 0.05f);
    PBSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setDensity(1500.f);

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid.getKernel(), fluid.getRestDensity());

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getDx()), 0.f, 1.0e-6f);
}
