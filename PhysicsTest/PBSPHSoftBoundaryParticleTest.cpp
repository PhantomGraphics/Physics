#include "pch.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/SoftBoundaryParticles.h"
#include "../Physics/RigidBoundaryParticles.h"
#include "../Physics/IBoundaryParticles.h"
#include <cmath>
#include <memory>

using namespace Phantom::Physics;
using namespace Phantom::Math;

// PBSPH's Two-Way (Track B) SoftBody boundary-particle contract. Mirrors
// WCSPH/DFSPHSoftBoundaryParticleTest.cpp, with the same deliberate
// difference as the rigid counterpart: PBSPH's fluid-side quantity is a
// *position correction* (getDx()), not a force; only the boundary-side
// reaction (accumForce) is checked as a force.

namespace {
const Quaternion kIdentity(1.f, 0.f, 0.f, 0.f);

SoftMesh makeSingleParticleMesh(const Vector3df& pos) {
    SoftMesh mesh;
    mesh.particles.resize(1);
    mesh.particles.positions[0] = pos;
    return mesh;
}

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

// See the rigid counterpart's makePuddle() -- dense enough for a full
// simulate() pass to produce a measurable boundary reaction.
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

TEST(PBSPHSoftBoundaryParticleTest, PenetratingParticle_CorrectionPushesAway_AndReactionAccumulated) {
    auto fluid = makeFluid({ 0.6f, 0.f, 0.f });
    PBSPHParticle particle(fluid->getParticles(), 0, fluid.get());
    particle.setDensity(1500.f);  // density (1500) > rest density (1000) -> positive constraint
    particle.setLambda(0.5f);     // stand-in for calculateLambda() in a full solver pass

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid->getKernel(), fluid->getRestDensity());
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
    auto fluid = makeFluid({ 10.f, 0.f, 0.f });
    PBSPHParticle particle(fluid->getParticles(), 0, fluid.get());
    particle.setDensity(1500.f);

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, 0.f, 0.f });
    SoftBoundaryParticles boundary;
    boundary.bind(&mesh);
    boundary.sync();
    boundary.computePsi(*fluid->getKernel(), fluid->getRestDensity());

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs{ &boundary };

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getDx()), 0.f, 1.0e-6f);
}

TEST(PBSPHSoftBoundaryParticleTest, NoSoftBoundaries_IsNoOp) {
    auto fluid = makeFluid({ 0.6f, 0.f, 0.f });
    PBSPHParticle particle(fluid->getParticles(), 0, fluid.get());
    particle.setDensity(1500.f);
    particle.setLambda(0.5f);

    std::vector<PBSPHParticle> particles{ particle };
    std::vector<IBoundaryParticles*> softs;  // empty

    PBSPHSolver solver;
    solver.addBoundaryParticlePressure(particles, softs, 0.016f);

    EXPECT_NEAR(getLength(particles[0].getDx()), 0.f, 1.0e-6f);
}

// clearSoftBoundaryParticles() must drop only the soft list -- an
// independently registered rigid boundary particle set stays coupled.
TEST(PBSPHSoftBoundaryParticleTest, ClearSoftBoundaryParticles_LeavesRigidBoundariesCoupled) {
    auto fluid = makePuddle();

    SoftMesh mesh = makeSingleParticleMesh({ 0.f, -0.15f, 0.f });
    SoftBoundaryParticles soft;
    soft.bind(&mesh);
    soft.sync();
    soft.computePsi(*fluid->getKernel(), fluid->getRestDensity());

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
    solver.addSoftBoundaryParticles(&soft);
    solver.addRigidBoundaryParticles(&rigid);

    solver.clearSoftBoundaryParticles();

    for (int step = 0; step < 5; ++step) {
        soft.clearAccumForce();
        rigid.clearAccumForce();
        solver.simulate(0.005f, 3);
    }

    float softF = 0.f;
    for (auto& bp : soft.particles()) softF += getLength(bp.accumForce);
    float rigidF = 0.f;
    for (auto& bp : rigid.particles()) rigidF += getLength(bp.accumForce);

    EXPECT_NEAR(softF, 0.f, 1.0e-6f);   // soft list was cleared -> untouched
    EXPECT_GT(rigidF, 0.f);             // rigid list still coupled
}
