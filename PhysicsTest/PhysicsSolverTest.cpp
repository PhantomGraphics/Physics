#include "pch.h"
#include "../Physics/PhysicsSolver.h"
#include "../Physics/ClothBody.h"
#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/DFSPHSolver.h"
#include "PhysicsFluidFactory.h"
#include <cmath>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {
constexpr float kTol = 1.0e-4f;

// Keeps build()'s uniform-grid seeding to a handful of particles so tests stay fast.
void useTinyFluidBounds(PhysicsFluidFactory& factory) {
    auto& p = factory.params();
    p.radius = 0.5f;
    p.boundaryTimeStep = 0.01f;
    p.fluidBounds = Box3df(Vector3df(0.f, 0.f, 0.f), Vector3df(1.f, 1.f, 1.f));
}
}

TEST(PhysicsSolverTest, Build_DFSPH_CreatesParticles) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);
    factory.build();
    EXPECT_GT(factory.getParticleCount(), 0u);
}

TEST(PhysicsSolverTest, Build_PBSPH_CreatesParticles) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::PBSPH);
    factory.build();
    EXPECT_GT(factory.getParticleCount(), 0u);
}

TEST(PhysicsSolverTest, Build_CSPH_CreatesParticles) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);
    factory.build();
    EXPECT_GT(factory.getParticleCount(), 0u);
}

TEST(PhysicsSolverTest, Build_SwitchingTypeDiscardsPreviousFluid) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);
    factory.build();
    ASSERT_GT(factory.getParticleCount(), 0u);

    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);
    factory.build();
    EXPECT_GT(factory.getParticleCount(), 0u);
    // CSPH exposes no SPHKernel accessor (only DFSPH/PBSPH do).
    EXPECT_EQ(factory.getActiveKernel(), nullptr);
}

TEST(PhysicsSolverTest, SetCustomInitialData_OverridesGridSeeding) {
    PhysicsFluidFactory factory;
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);
    factory.setCustomInitialData({ { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f }, { 2.f, 0.f, 0.f } });
    factory.build();
    EXPECT_EQ(factory.getParticleCount(), 3u);
}

// WCSPH joined DFSPH/PBSPH in Phase 4 of
// docs/todo/PLAN_physics_ownership_and_coupling_unification.md (force-based,
// same Two-Way mechanism as DFSPH).
TEST(PhysicsSolverTest, SupportsTwoWayCoupling_DfsphPbsphAndWcsph) {
    PhysicsFluidFactory factory;

    PhysicsSolver dfsph;
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);
    auto dfsphSolver = factory.build();
    dfsph.setFluidSolver(dfsphSolver.get());
    EXPECT_TRUE(dfsph.supportsTwoWayCoupling());

    PhysicsSolver pbsph;
    factory.setFluidType(PhysicsFluidFactory::FluidType::PBSPH);
    auto pbsphSolver = factory.build();
    pbsph.setFluidSolver(pbsphSolver.get());
    EXPECT_TRUE(pbsph.supportsTwoWayCoupling());

    PhysicsSolver csph;
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);
    auto csphSolver = factory.build();
    csph.setFluidSolver(csphSolver.get());
    EXPECT_TRUE(csph.supportsTwoWayCoupling());
}

TEST(PhysicsSolverTest, SupportsTwoWayCoupling_FalseBeforeFluidSolverIsSet) {
    PhysicsSolver solver;
    EXPECT_FALSE(solver.supportsTwoWayCoupling());
}

TEST(PhysicsSolverTest, SetRunning_SyncsToRigidSolver) {
    PhysicsSolver solver;
    EXPECT_FALSE(solver.isRunning());
    EXPECT_FALSE(solver.rigidSolver().isRunning());

    solver.setRunning(true);

    EXPECT_TRUE(solver.isRunning());
    EXPECT_TRUE(solver.rigidSolver().isRunning());
}

TEST(PhysicsSolverTest, Step_NoOpWhenNotRunning) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());
    solver.setTimeStep(0.01f);

    auto before = factory.getParticlePositions();
    solver.step();  // running_ defaults to false.
    auto after = factory.getParticlePositions();

    ASSERT_EQ(before.size(), after.size());
    for (size_t i = 0; i < before.size(); ++i) {
        EXPECT_NEAR(getDistance(before[i], after[i]), 0.f, kTol);
    }
}

TEST(PhysicsSolverTest, Step_RunningAdvancesRigidBodyUnderGravity) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());
    solver.setTimeStep(0.01f);

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);
    solver.bindRigidBody(body, &sphere, CouplingMode::OneWay);

    solver.rigidSolver().params().gravity = { 0.f, -9.8f, 0.f };
    solver.rigidSolver().saveSnapshot();
    solver.setRunning(true);

    solver.step();

    const float expectedVy = -9.8f * solver.getTimeStep();
    EXPECT_NEAR(body->linearVelocity.y, expectedVy, kTol);
}

TEST(PhysicsSolverTest, StepUnconditional_AdvancesEvenWhenNotRunning) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());
    solver.setTimeStep(0.01f);

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 0.f, 5.f, 0.f };
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);
    solver.bindRigidBody(body, &sphere, CouplingMode::OneWay);

    solver.rigidSolver().params().gravity = { 0.f, -9.8f, 0.f };
    solver.rigidSolver().saveSnapshot();
    // running_ intentionally left false.

    solver.stepUnconditional();

    EXPECT_LT(body->position.y, 5.f);
}

TEST(PhysicsSolverTest, BindRigidBody_CreatesBindingWithBodyPose) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::CSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 1.f, 2.f, 3.f };
    bodyOwned->setShape(&sphere);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);

    auto& binding = solver.bindRigidBody(body, &sphere, CouplingMode::OneWay);

    EXPECT_EQ(binding.body, body);
    EXPECT_EQ(solver.rigidFluidSolver().getBindings().size(), 1u);
    EXPECT_NEAR(getDistance(binding.boundary.getPosition(), body->position), 0.f, kTol);
}

TEST(PhysicsSolverTest, ClearRigidBodyBindings_RemovesAllBindings) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->setShape(&sphere);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);
    solver.bindRigidBody(body, &sphere, CouplingMode::OneWay);
    ASSERT_EQ(solver.rigidFluidSolver().getBindings().size(), 1u);

    solver.clearRigidBodyBindings();

    EXPECT_TRUE(solver.rigidFluidSolver().getBindings().empty());
}

TEST(PhysicsSolverTest, SetRunning_SyncsToSoftSolver) {
    PhysicsSolver solver;
    EXPECT_FALSE(solver.softSolver().isRunning());

    solver.setRunning(true);

    EXPECT_TRUE(solver.softSolver().isRunning());
}

TEST(PhysicsSolverTest, BindSoftBody_CreatesBindingWithMeshBoundaryParticles) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());

    ClothBodyParams p;
    p.rows = 3;
    p.cols = 3;
    auto clothOwned = std::make_unique<ClothBody>(p);
    auto* cloth = clothOwned.get();
    solver.softSolver().addBody(cloth);

    auto& binding = solver.bindSoftBody(cloth);

    EXPECT_EQ(binding.body, cloth);
    EXPECT_EQ(solver.softFluidSolver().getBindings().size(), 1u);
    EXPECT_EQ(binding.particles.particles().size(), cloth->getMesh().particles.size());
}

TEST(PhysicsSolverTest, ClearSoftBodyBindings_RemovesAllBindings) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());

    ClothBodyParams p;
    p.rows = 3;
    p.cols = 3;
    auto clothOwned = std::make_unique<ClothBody>(p);
    auto* cloth = clothOwned.get();
    solver.softSolver().addBody(cloth);
    solver.bindSoftBody(cloth);
    ASSERT_EQ(solver.softFluidSolver().getBindings().size(), 1u);

    solver.clearSoftBodyBindings();

    EXPECT_TRUE(solver.softFluidSolver().getBindings().empty());
}

TEST(PhysicsSolverTest, StepUnconditional_CouplesFluidAndClothBody) {
    // Built directly (bypassing PhysicsFluidFactory) so rest density is a
    // known, deliberately-low constant rather than the factory's
    // synthetic-grid calibration -- that calibration's density is a similar
    // order of magnitude to what a single nearby SoftBoundaryParticle
    // contributes, making "does the coupling force clearly exceed rest
    // density" too close to call. A known-low rest density keeps the
    // outcome deterministic.
    DFSPHFluid fluid;
    fluid.density      = 1.0f;
    fluid.pressureCoe  = 1.0f;
    fluid.viscosityCoe = 0.f;
    fluid.setEffectLength(1.0f);  // comfortably covers the ~0.07 distance to the nearest cloth particle
    // Deliberately off-center (not equidistant from the cloth's 4 corners):
    // a perfectly centered particle produces a radially-symmetric force on
    // all 4 corners that both nets to zero translation AND can't deform the
    // fully-triangulated (structural + diagonal edges, near-zero compliance)
    // square -- an off-center placement breaks that symmetry so the net
    // force actually moves the (rigid but unpinned) cloth.
    fluid.createParticle(Vector3df(0.03f, 1.f, 0.f), 0.05f, 1.0f);

    auto dfsphSolver = std::make_unique<DFSPHSolver>();
    dfsphSolver->setTimeStep(0.01f);
    dfsphSolver->setExternalForce({ 0.f, 0.f, 0.f });  // isolate the coupling force from fluid-side gravity
    dfsphSolver->add(&fluid);

    PhysicsSolver solver;
    solver.setFluidSolver(dfsphSolver.get());
    solver.setSoftCouplingFluidInfo(fluid.getKernel(), fluid.getDensity());
    solver.setTimeStep(0.01f);

    ClothBodyParams p;
    p.rows        = 2;
    p.cols        = 2;
    p.width       = 0.1f;
    p.height      = 0.1f;
    p.pinTopLeft  = false;
    p.pinTopRight = false;
    p.pinTopEdge  = false;
    p.origin      = { 0.f, 1.f, 0.f };  // near the fluid particle, clear of the y=0 floor collider
    auto clothOwned = std::make_unique<ClothBody>(p);
    auto* cloth = clothOwned.get();
    solver.softSolver().addBody(cloth);
    solver.softSolver().solverParams().gravity = { 0.f, 0.f, 0.f };  // isolate the coupling force

    solver.bindSoftBody(cloth);
    solver.setRunning(true);

    solver.stepUnconditional();

    float maxSpeed = 0.f;
    for (const auto& v : cloth->getMesh().particles.velocities)
        maxSpeed = std::max(maxSpeed, glm::length(v));
    EXPECT_GT(maxSpeed, 0.f) << "cloth particles should have picked up velocity from the fluid coupling force";
}

TEST(PhysicsSolverTest, BindRigidSoftBody_CreatesBindingWithBodyPose) {
    PhysicsSolver solver;

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 1.f, 2.f, 3.f };
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);

    auto& binding = solver.bindRigidSoftBody(body, CouplingMode::OneWay);

    EXPECT_EQ(binding.rigidBody, body);
    EXPECT_EQ(solver.rigidSoftSolver().getBindings().size(), 1u);
    EXPECT_NEAR(getDistance(binding.boundary.getPosition(), body->position), 0.f, kTol);
}

TEST(PhysicsSolverTest, ClearRigidSoftBindings_RemovesAllBindings) {
    PhysicsSolver solver;

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->setShape(&sphere);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);
    solver.bindRigidSoftBody(body, CouplingMode::TwoWay);
    ASSERT_EQ(solver.rigidSoftSolver().getBindings().size(), 1u);

    solver.clearRigidSoftBindings();

    EXPECT_TRUE(solver.rigidSoftSolver().getBindings().empty());
}

// Regression guard for the three-way (fluid/rigid/soft) orchestrator: binding
// a rigid body to both the fluid (bindRigidBody) and a cloth
// (bindRigidSoftBody), plus the cloth to the fluid (bindSoftBody), and
// stepping once must not crash and must leave every body's state finite.
TEST(PhysicsSolverTest, StepUnconditional_FluidRigidAndSoftAllCoupledSimultaneously) {
    PhysicsFluidFactory factory;
    useTinyFluidBounds(factory);
    factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);

    PhysicsSolver solver;
    auto fluidSolver = factory.build();
    solver.setFluidSolver(fluidSolver.get());
    solver.setSoftCouplingFluidInfo(factory.getActiveKernel(), factory.getActiveRestDensity());
    solver.setTimeStep(0.01f);

    SphereShape sphere;
    sphere.radius = 0.5f;
    auto bodyOwned = std::make_unique<RigidBody>();
    bodyOwned->position = { 5.f, 5.f, 5.f };  // clear of the fluid bounds and the y=0 floor collider
    bodyOwned->setShape(&sphere);
    bodyOwned->setMass(1.f);
    RigidBody* body = bodyOwned.get();
    solver.rigidSolver().addBody(body);
    solver.bindRigidBody(body, &sphere, CouplingMode::OneWay);

    ClothBodyParams p;
    p.rows   = 3;
    p.cols   = 3;
    p.origin = { 5.f, 5.f, 5.f };  // near the rigid body, clear of the fluid and floor
    auto clothOwned = std::make_unique<ClothBody>(p);
    auto* cloth = clothOwned.get();
    solver.softSolver().addBody(cloth);
    solver.bindSoftBody(cloth);

    solver.bindRigidSoftBody(body, CouplingMode::TwoWay);

    solver.rigidSolver().params().gravity = { 0.f, -9.8f, 0.f };
    solver.rigidSolver().saveSnapshot();
    solver.setRunning(true);

    solver.stepUnconditional();

    EXPECT_TRUE(std::isfinite(body->position.y));
    for (const auto& pos : cloth->getMesh().particles.positions) {
        EXPECT_TRUE(std::isfinite(pos.y));
    }
}
