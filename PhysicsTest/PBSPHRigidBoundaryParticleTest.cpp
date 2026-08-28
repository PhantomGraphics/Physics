#include "pch.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/RigidBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);
}

TEST(PBSPHRigidBoundaryParticleTest, PenetratingParticle_CorrectionPushesAway_AndReactionAccumulated) {
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

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    ASSERT_FALSE(boundary.particles().empty());
    boundary.computePsi(*fluid.getKernel(), fluid.getRestDensity());
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
    PBSPHFluid fluid;
    fluid.setRestDensity(1000.f);
    fluid.setStiffness(1.f);
    fluid.setVicsosity(0.f);
    fluid.setEffectLength(1.0f);
    fluid.setIsBoundary(false);

    fluid.createParticle({ 10.f, 0.f, 0.f }, 0.05f);
    PBSPHParticle particle(fluid.getParticles(), 0, &fluid);
    particle.setDensity(1500.f);

    RigidBoundaryParticles boundary;
    SphereShape sphere;
    sphere.radius = 0.5f;
    boundary.sample(sphere, 0.2f);
    boundary.computePsi(*fluid.getKernel(), fluid.getRestDensity());
    boundary.sync({ 0.f, 0.f, 0.f }, kIdentity);

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> rigids{ &boundary };

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, rigids, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getDx()), 0.f, 1.0e-6f);
}
