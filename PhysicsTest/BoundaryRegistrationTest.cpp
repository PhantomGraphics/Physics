#include "pch.h"
#include "../Physics/PhysicsSolver.h"
#include "../Physics/ClothBody.h"
#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHSolver.h"

#include <memory>
#include <vector>

// Boundary registration lifetime contract (docs/todo/PLAN_physics_refactoring.md
// Phase 1): add*() ignores nullptr and duplicates, remove*() drops exactly one
// pointer, and PhysicsSolver only ever unregisters what it registered itself.

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

// A fluid with a kernel (so psi can be computed) and a chosen rest density.
struct Fluid {
    DFSPHFluid fluid;
    DFSPHSolver solver;
    explicit Fluid(float restDensity, float effectLength = 1.0f)
    {
        fluid.density = restDensity;
        fluid.setEffectLength(effectLength);
        fluid.createParticle(Vector3df(0.f, 1.f, 0.f), 0.05f, 1.0f);
        solver.add(&fluid);
    }
};

void expectNoRegistrations(const ISPHSolver& s)
{
    EXPECT_EQ(s.getRigidBoundaryCount(), 0u);
    EXPECT_EQ(s.getRigidBoundaryParticlesCount(), 0u);
    EXPECT_EQ(s.getSoftBoundaryParticlesCount(), 0u);
}

} // namespace

// ------------------------------------------------ ISPHSolver add/remove ----

template <class SolverT>
class SolverRegistrationTest : public ::testing::Test {};
using RegistrationSolvers = ::testing::Types<DFSPHSolver, PBSPHSolver, WCSPHSolver>;
TYPED_TEST_SUITE(SolverRegistrationTest, RegistrationSolvers);

TYPED_TEST(SolverRegistrationTest, NullAndDuplicatesAreIgnoredAndRemovalIsExact)
{
    TypeParam solver;
    RigidBoundary a, b;
    RigidBoundaryParticles ra, rb;
    SoftBoundaryParticles sa, sb;

    solver.addRigidBoundary(nullptr);
    solver.addRigidBoundaryParticles(nullptr);
    solver.addSoftBoundaryParticles(nullptr);
    expectNoRegistrations(solver);

    solver.addRigidBoundary(&a);
    solver.addRigidBoundary(&a);
    solver.addRigidBoundary(&b);
    solver.addRigidBoundaryParticles(&ra);
    solver.addRigidBoundaryParticles(&ra);
    solver.addRigidBoundaryParticles(&rb);
    solver.addSoftBoundaryParticles(&sa);
    solver.addSoftBoundaryParticles(&sa);
    solver.addSoftBoundaryParticles(&sb);
    EXPECT_EQ(solver.getRigidBoundaryCount(), 2u);
    EXPECT_EQ(solver.getRigidBoundaryParticlesCount(), 2u);
    EXPECT_EQ(solver.getSoftBoundaryParticlesCount(), 2u);

    EXPECT_TRUE(solver.removeRigidBoundary(&a));
    EXPECT_FALSE(solver.removeRigidBoundary(&a));       // already gone
    EXPECT_FALSE(solver.removeRigidBoundary(nullptr));
    EXPECT_TRUE(solver.removeRigidBoundaryParticles(&ra));
    EXPECT_FALSE(solver.removeRigidBoundaryParticles(&ra));
    EXPECT_TRUE(solver.removeSoftBoundaryParticles(&sa));
    EXPECT_FALSE(solver.removeSoftBoundaryParticles(&sa));
    EXPECT_EQ(solver.getRigidBoundaryCount(), 1u);          // b remains
    EXPECT_EQ(solver.getRigidBoundaryParticlesCount(), 1u); // rb remains
    EXPECT_EQ(solver.getSoftBoundaryParticlesCount(), 1u);  // sb remains

    // The lists are independent: clearing one kind keeps the others.
    solver.clearRigidBoundaryParticles();
    EXPECT_EQ(solver.getRigidBoundaryParticlesCount(), 0u);
    EXPECT_EQ(solver.getRigidBoundaryCount(), 1u);
    EXPECT_EQ(solver.getSoftBoundaryParticlesCount(), 1u);
    solver.clearRigidBoundaries();
    solver.clearSoftBoundaryParticles();
    expectNoRegistrations(solver);
}

// ------------------------------------------------------ PhysicsSolver ------

TEST(BoundaryRegistrationTest, SwitchingFluidKeepsRegistrationsMadeByOtherOwners)
{
    Fluid oldFluid(1.0f), newFluid(1.0f);
    PhysicsSolver solver;
    SphereShape shape;
    RigidBody body;
    body.setShape(&shape);
    RigidBoundary external;                       // e.g. FluidWorld's mesh boundary
    oldFluid.solver.addRigidBoundary(&external);  // registered directly, not via PhysicsSolver

    solver.setFluidSolver(&oldFluid.solver);
    auto& binding = solver.bindRigidBody(&body, &shape, CouplingMode::OneWay);
    ASSERT_EQ(oldFluid.solver.getRigidBoundaryCount(), 2u);

    solver.setFluidSolver(&newFluid.solver);
    // Only the entry PhysicsSolver made left the old solver.
    EXPECT_EQ(oldFluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_TRUE(oldFluid.solver.removeRigidBoundary(&external));
    EXPECT_EQ(newFluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_TRUE(newFluid.solver.removeRigidBoundary(&binding.boundary));

    solver.setFluidSolver(nullptr);
}

TEST(BoundaryRegistrationTest, NullAndDuplicateRegistrationsThroughPhysicsSolverAreIgnored)
{
    Fluid fluid(1.0f);
    PhysicsSolver solver;
    solver.setFluidSolver(&fluid.solver);
    RigidBoundary boundary;
    RigidBoundaryParticles particles;

    EXPECT_FALSE(solver.addRigidBoundary(nullptr));
    EXPECT_FALSE(solver.addRigidBoundaryParticles(nullptr));
    EXPECT_TRUE(solver.addRigidBoundary(&boundary));
    EXPECT_FALSE(solver.addRigidBoundary(&boundary));
    EXPECT_TRUE(solver.addRigidBoundaryParticles(&particles));
    EXPECT_FALSE(solver.addRigidBoundaryParticles(&particles));
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_EQ(fluid.solver.getRigidBoundaryParticlesCount(), 1u);

    EXPECT_FALSE(solver.removeRigidBoundary(nullptr));
    EXPECT_TRUE(solver.removeRigidBoundary(&boundary));
    EXPECT_FALSE(solver.removeRigidBoundary(&boundary));
    EXPECT_TRUE(solver.removeRigidBoundaryParticles(&particles));
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 0u);
    EXPECT_EQ(fluid.solver.getRigidBoundaryParticlesCount(), 0u);
    solver.setFluidSolver(nullptr);
}

TEST(BoundaryRegistrationTest, ReinstallingTheSameSolverKeepsOneRegistrationEach)
{
    Fluid fluid(1.0f);
    PhysicsSolver solver;
    SphereShape shape;
    RigidBody body;
    body.setShape(&shape);
    ClothBody cloth(ClothBodyParams{});
    solver.setFluidSolver(&fluid.solver);
    solver.bindRigidBody(&body, &shape, CouplingMode::OneWay);
    solver.bindSoftBody(&cloth);

    solver.setFluidSolver(&fluid.solver);
    solver.resyncFluidRegistrations();
    solver.resyncFluidRegistrations();
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_EQ(fluid.solver.getSoftBoundaryParticlesCount(), 1u);
    solver.setFluidSolver(nullptr);
    expectNoRegistrations(fluid.solver);
}

TEST(BoundaryRegistrationTest, BindingsMadeWithoutAFluidAttachLaterAndDetachCleanly)
{
    Fluid fluid(1.0f);
    PhysicsSolver solver;
    SphereShape shape;
    RigidBody body;
    body.setShape(&shape);
    ClothBody cloth(ClothBodyParams{});

    auto& rigid = solver.bindRigidBody(&body, &shape, CouplingMode::OneWay);  // no fluid yet
    solver.bindSoftBody(&cloth);
    EXPECT_EQ(solver.fluidSolver(), nullptr);
    expectNoRegistrations(fluid.solver);

    solver.setFluidSolver(&fluid.solver);
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_EQ(fluid.solver.getSoftBoundaryParticlesCount(), 1u);

    // Disconnect and reconnect: the bindings survive and re-register.
    solver.setFluidSolver(nullptr);
    expectNoRegistrations(fluid.solver);
    EXPECT_EQ(solver.rigidFluidSolver().getBindings().size(), 1u);
    EXPECT_EQ(&solver.rigidFluidSolver().getBindings().front(), &rigid);
    solver.setFluidSolver(&fluid.solver);
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_EQ(fluid.solver.getSoftBoundaryParticlesCount(), 1u);
    solver.setFluidSolver(nullptr);
}

TEST(BoundaryRegistrationTest, ClearingBindingsUnregistersFromTheSolverBeforeTheyAreDestroyed)
{
    Fluid fluid(1.0f);
    PhysicsSolver solver;
    SphereShape shape;
    RigidBody body;
    body.setShape(&shape);
    ClothBody cloth(ClothBodyParams{});
    RigidBoundary external;
    fluid.solver.addRigidBoundary(&external);

    solver.setFluidSolver(&fluid.solver);
    auto& rigid = solver.bindRigidBody(&body, &shape, CouplingMode::TwoWay);
    solver.addRigidBoundaryParticles(&rigid.particles);
    solver.bindSoftBody(&cloth);
    ASSERT_EQ(fluid.solver.getRigidBoundaryCount(), 2u);
    ASSERT_EQ(fluid.solver.getRigidBoundaryParticlesCount(), 1u);

    solver.clearRigidBodyBindings();
    solver.clearSoftBodyBindings();
    // Nothing of the destroyed bindings is left in the solver; the external boundary is.
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);
    EXPECT_EQ(fluid.solver.getRigidBoundaryParticlesCount(), 0u);
    EXPECT_EQ(fluid.solver.getSoftBoundaryParticlesCount(), 0u);
    EXPECT_TRUE(solver.rigidFluidSolver().getBindings().empty());
    EXPECT_TRUE(solver.softFluidSolver().getBindings().empty());

    // The fluid can keep stepping safely: nothing refers to the destroyed bindings.
    fluid.solver.setTimeStep(0.001f);
    fluid.solver.simulate(0.001f, 2);
    solver.setFluidSolver(nullptr);
}

TEST(BoundaryRegistrationTest, ClearOnTheSolverIsRepairedByResync)
{
    Fluid fluid(1.0f);
    PhysicsSolver solver;
    SphereShape shape;
    RigidBody body;
    body.setShape(&shape);
    solver.setFluidSolver(&fluid.solver);
    solver.bindRigidBody(&body, &shape, CouplingMode::OneWay);
    ASSERT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);

    fluid.solver.clearRigidBoundaries();  // somebody bypassed PhysicsSolver
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 0u);
    solver.resyncFluidRegistrations();
    EXPECT_EQ(fluid.solver.getRigidBoundaryCount(), 1u);
    solver.setFluidSolver(nullptr);
}

TEST(BoundaryRegistrationTest, SwitchingFluidRefreshesPsiKernelAndStaleReaction)
{
    Fluid light(1.0f, 1.0f), heavy(4.0f, 1.5f);
    PhysicsSolver solver;
    SphereShape shape;
    shape.radius = 0.3f;
    RigidBody body;
    body.setShape(&shape);

    solver.setFluidSolver(&light.solver);
    auto& binding = solver.bindRigidBody(&body, &shape, CouplingMode::TwoWay);
    binding.particles.sample(shape, 0.1f);
    ASSERT_FALSE(binding.particles.particles().empty());
    solver.addRigidBoundaryParticles(&binding.particles);

    // Reference psi for each fluid, computed independently on a copy of the samples.
    auto referencePsi = [&](Fluid& f) {
        RigidBoundaryParticles ref;
        ref.sample(shape, 0.1f);
        ref.computePsi(*f.solver.getKernel(), f.solver.getRestDensity());
        return ref.particles().front().psi;
    };
    const float psiLight = referencePsi(light);
    const float psiHeavy = referencePsi(heavy);
    ASSERT_NE(psiLight, psiHeavy);
    EXPECT_FLOAT_EQ(binding.particles.particles().front().psi, psiLight);

    binding.particles.particles().front().accumForce = Vector3df(1.f, 2.f, 3.f);  // leftover reaction
    solver.setFluidSolver(&heavy.solver);
    EXPECT_FLOAT_EQ(binding.particles.particles().front().psi, psiHeavy);
    EXPECT_EQ(binding.particles.particles().front().accumForce, Vector3df(0.f, 0.f, 0.f));
    EXPECT_EQ(solver.fluidSolver()->getRigidBoundaryParticlesCount(), 1u);

    // Switching back recomputes for the first fluid again.
    solver.setFluidSolver(&light.solver);
    EXPECT_FLOAT_EQ(binding.particles.particles().front().psi, psiLight);
    EXPECT_EQ(heavy.solver.getRigidBoundaryParticlesCount(), 0u);
    solver.setFluidSolver(nullptr);
}

TEST(BoundaryRegistrationTest, OldFluidOutlivingTheSolverHoldsNoDanglingPointersAfterDetach)
{
    Fluid fluid(1.0f);
    {
        PhysicsSolver solver;
        SphereShape shape;
        RigidBody body;
        body.setShape(&shape);
        ClothBody cloth(ClothBodyParams{});
        solver.setFluidSolver(&fluid.solver);
        solver.bindRigidBody(&body, &shape, CouplingMode::OneWay);
        solver.bindSoftBody(&cloth);
        solver.setFluidSolver(nullptr);  // documented requirement before the solver is destroyed
    }
    expectNoRegistrations(fluid.solver);
    fluid.solver.setTimeStep(0.001f);
    fluid.solver.simulate(0.001f, 2);  // would touch freed bindings if any were left
}
