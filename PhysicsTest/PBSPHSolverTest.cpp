#include "pch.h"

#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/ISPHSolver.h"
#include "../Physics/SphereBoundary.h"
#include "../Physics/PlateBoundary.h"
#include "../Physics/PlaneBoundary.h"

#include <cmath>
#include <memory>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;

bool isFinite3(const Vector3df& v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Box3df makeBoundaryBox()
{
  return Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f));
}

// Create a two-particle fluid ready for simulation.
std::unique_ptr<PBSPHFluid> makeDynamicFluid()
{
  auto f = std::make_unique<PBSPHFluid>();
  f->setRestDensity(1000.0f);
  f->setStiffness(200.0f);
  f->setVicsosity(0.01f);
  f->setEffectLength(0.1f);
  f->setIsBoundary(false);
  f->createParticle(Vector3df(0.0f,  0.4f, 0.0f), 0.025f);
  f->createParticle(Vector3df(0.05f, 0.4f, 0.0f), 0.025f);
  return f;
}
}

// ---- PBSPHFluid getter/setter round-trips --------------------------------

TEST(PBSPHFluidTest, GettersReturnSetValues)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(800.0f);
  fluid.setStiffness(150.0f);
  fluid.setVicsosity(0.05f);
  fluid.setIsBoundary(true);

  EXPECT_FLOAT_EQ(fluid.getRestDensity(), 800.0f);
  EXPECT_FLOAT_EQ(fluid.getStiffness(), 150.0f);
  EXPECT_FLOAT_EQ(fluid.getViscosity(), 0.05f);
  EXPECT_TRUE(fluid.isBoundary());

  fluid.setIsBoundary(false);
  EXPECT_FALSE(fluid.isBoundary());
}

// ---- Particle addition ---------------------------------------------------

TEST(PBSPHFluidTest, AddParticleIncreasesCount)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.setEffectLength(0.1f);

  EXPECT_EQ(fluid.getParticles().size(), 0U);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f);
  EXPECT_EQ(fluid.getParticles().size(), 1U);

  fluid.createParticle(Vector3df(1.0f, 0.0f, 0.0f), 0.025f);
  EXPECT_EQ(fluid.getParticles().size(), 2U);
}

// ---- Bounding box -------------------------------------------------------

TEST(PBSPHFluidTest, BoundingBoxContainsAllParticles)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.setEffectLength(0.1f);
  fluid.createParticle(Vector3df(-0.5f,  0.3f,  0.1f), 0.025f);
  fluid.createParticle(Vector3df( 0.5f, -0.3f, -0.1f), 0.025f);

  const auto bb = fluid.getBoundingBox();
  EXPECT_LE(bb.getMin().x, -0.5f + kTol);
  EXPECT_LE(bb.getMin().y, -0.3f + kTol);
  EXPECT_GE(bb.getMax().x,  0.5f - kTol);
  EXPECT_GE(bb.getMax().y,  0.3f - kTol);
}

// ---- Emitter (internal design notes) -------------------

TEST(PBSPHFluidTest, UpdateEmittersIsNoOpWithoutRegisteredEmitters)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);

  fluid.updateEmitters(0.1f);

  EXPECT_EQ(fluid.getParticles().size(), 0U);
}

TEST(PBSPHFluidTest, UpdateEmittersEmitsAtTheConfiguredRate)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);

  Emitter e;
  e.rate = 100.0f;
  fluid.addEmitter(e);

  for (int step = 0; step < 10; ++step)
  {
    fluid.updateEmitters(0.01f);
  }

  EXPECT_EQ(fluid.getParticles().size(), 10U);
}

TEST(PBSPHFluidTest, UpdateEmittersStopsAtMaxParticles)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.setMaxParticles(5);

  Emitter e;
  e.rate = 10000.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(1.0f);

  EXPECT_EQ(fluid.getParticles().size(), 5U);
}

TEST(PBSPHFluidTest, UpdateEmittersGivesSpawnedParticlesVelocityAlongDirection)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);

  Emitter e;
  e.rate = 100.0f;
  e.direction = Vector3df(0.0f, 1.0f, 0.0f);
  e.speed = 2.0f;
  e.speedJitter = 0.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(0.01f);

  ASSERT_EQ(fluid.getParticles().size(), 1U);
  const Vector3df vel = fluid.getParticles().velocities[0];
  EXPECT_NEAR(vel.x, 0.0f, kTol);
  EXPECT_NEAR(vel.y, 2.0f, kTol);
  EXPECT_NEAR(vel.z, 0.0f, kTol);
}

TEST(PBSPHFluidTest, ClearEmittersRemovesRegisteredEmitters)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);

  Emitter e;
  fluid.addEmitter(e);
  EXPECT_EQ(fluid.getEmitters().size(), 1U);

  fluid.clearEmitters();
  EXPECT_TRUE(fluid.getEmitters().empty());

  fluid.updateEmitters(1.0f);
  EXPECT_EQ(fluid.getParticles().size(), 0U);
}

// ---- Outflow region (optional particle removal) --------------------------

TEST(PBSPHFluidTest, RemoveOutflowParticlesIsNoOpWithoutRegisteredRegions)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f);

  fluid.removeOutflowParticles();

  EXPECT_EQ(fluid.getParticles().size(), 1U);
}

TEST(PBSPHFluidTest, RemoveOutflowParticlesDeletesParticlesInsideRegionOnly)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, -10.0f, 0.0f), 0.025f); // inside
  fluid.createParticle(Vector3df(0.0f, 10.0f, 0.0f), 0.025f);  // outside

  OutflowRegion r;
  r.bounds = Box3df(Vector3df(-1.0f, -11.0f, -1.0f), Vector3df(1.0f, -9.0f, 1.0f));
  fluid.addOutflowRegion(r);

  fluid.removeOutflowParticles();

  ASSERT_EQ(fluid.getParticles().size(), 1U);
  EXPECT_NEAR(fluid.getParticles().positions[0].y, 10.0f, kTol);
}

TEST(PBSPHFluidTest, ClearOutflowRegionsRemovesRegisteredRegions)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f);

  OutflowRegion r;
  r.bounds = Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f));
  fluid.addOutflowRegion(r);
  EXPECT_EQ(fluid.getOutflowRegions().size(), 1U);

  fluid.clearOutflowRegions();
  EXPECT_TRUE(fluid.getOutflowRegions().empty());

  fluid.removeOutflowParticles();
  EXPECT_EQ(fluid.getParticles().size(), 1U);
}

// ---- Kernel effect length -----------------------------------------------

TEST(PBSPHFluidTest, EffectLengthUpdatesKernel)
{
  PBSPHFluid fluid;
  fluid.setEffectLength(0.25f);
  EXPECT_NEAR(fluid.getKernel()->getEffectLength(), 0.25f, kTol);

  fluid.setEffectLength(0.5f);
  EXPECT_NEAR(fluid.getKernel()->getEffectLength(), 0.5f, kTol);
}

// ---- Scale invariance: mass-weighted constraint gradient -----------------
// Locks in the fix from internal design notes section 4:
// PBSPHParticle::accumulateConstraintGradient()/calculatePressure() used to
// weight the Poly6 gradient without the neighbor's mass, inconsistent with
// addDensity(rhs) (which is mass-weighted) and with the boundary-particle
// counterparts (which weight by bp.psi in both places). That inconsistency
// made lambda's natural magnitude scale as effectLength^8, so the SAME
// stiffness value produced a wildly different relative position correction
// at different particle scales. After weighting both by the neighbor's
// mass, the SAME stiffness now produces the SAME dx/radius regardless of
// scale -- this test builds a single overlapping particle pair at three
// length scales (same relative geometry each time) and checks that holds.

TEST(PBSPHFluidTest, PositionCorrectionOverRadiusIsScaleInvariant)
{
  const float pbsphStiffness = 1.0f;
  const float restDensity = 1.0f;
  const float hRatio = 2.25f;
  const float rOverH = 0.5f;

  float firstDxOverRadius = -1.0f;
  for (float radius : {1.0f, 0.1f, 0.01f})
  {
    const float h = hRatio * radius;
    const float r = rOverH * h;

    PBSPHFluid fluid;
    fluid.setRestDensity(restDensity);
    fluid.setStiffness(pbsphStiffness);
    fluid.setEffectLength(h);
    fluid.setIsBoundary(false);
    fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius);
    fluid.createParticle(Vector3df(r, 0.0f, 0.0f), radius);

    auto& soa = fluid.getParticles();
    PBSPHParticle p0(soa, 0, &fluid);
    PBSPHParticle p1(soa, 1, &fluid);
    p0.init();
    p1.init();

    p0.addSelfDensity();
    p0.addDensity(p1);
    p1.addSelfDensity();
    p1.addDensity(p0);

    p0.resetConstraintGradient();
    p1.resetConstraintGradient();
    p0.accumulateConstraintGradient(p1);
    p1.accumulateConstraintGradient(p0);

    p0.calculateLambda();
    p1.calculateLambda();

    p0.setDx(Vector3df(0.0f, 0.0f, 0.0f));
    p0.calculatePressure(p1);

    const float dxOverRadius = glm::length(p0.getDx()) / radius;

    if (firstDxOverRadius < 0.0f)
    {
      firstDxOverRadius = dxOverRadius;
    }
    else
    {
      EXPECT_NEAR(dxOverRadius, firstDxOverRadius, firstDxOverRadius * 0.01f)
          << "radius=" << radius << " dx/radius should stay constant across scales";
    }
  }
}

// internal design notes Phase 7 re-verification: Phase 1
// found that before the accumulateConstraintGradient()/calculatePressure()
// mass-weighting fix, kLambdaCap=1.0f (PBSPHParticle.cpp) was already
// saturated at the codebase's conventional default scale (radius=1,
// restDensity=1, effectLength=2.25*radius) -- the raw (pre-clamp) lambda was
// 26.5 there. Unlike PositionCorrectionOverRadiusIsScaleInvariant above
// (which scales radius/effectLength/mass together as one uniform rescale),
// this varies effectLength's ratio to radius and restDensity independently,
// to confirm the fix isn't a coincidence specific to the one combination
// Phase 1 originally measured.
TEST(PBSPHFluidTest, LambdaDoesNotSaturateAcrossEffectLengthAndRestDensityScales)
{
  const float pbsphStiffness = 1.0f;
  const float radius = 1.0f;
  const float rOverH = 0.5f;
  // Mirrors PBSPHParticle.cpp's private kLambdaCap constant.
  constexpr float kLambdaCap = 1.0f;

  for (float restDensity : {1.0f, 1000.0f})
  {
    for (float hRatio : {2.25f, 4.0f})
    {
      const float h = hRatio * radius;
      const float r = rOverH * h;

      PBSPHFluid fluid;
      fluid.setRestDensity(restDensity);
      fluid.setStiffness(pbsphStiffness);
      fluid.setEffectLength(h);
      fluid.setIsBoundary(false);
      fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius);
      fluid.createParticle(Vector3df(r, 0.0f, 0.0f), radius);

      auto& soa = fluid.getParticles();
      PBSPHParticle p0(soa, 0, &fluid);
      PBSPHParticle p1(soa, 1, &fluid);
      p0.init();
      p1.init();

      p0.addSelfDensity();
      p0.addDensity(p1);
      p1.addSelfDensity();
      p1.addDensity(p0);

      p0.resetConstraintGradient();
      p1.resetConstraintGradient();
      p0.accumulateConstraintGradient(p1);
      p1.accumulateConstraintGradient(p0);

      p0.calculateLambda();

      EXPECT_TRUE(std::isfinite(p0.getLambda()));
      EXPECT_LT(std::fabs(p0.getLambda()), kLambdaCap)
          << "restDensity=" << restDensity << " hRatio=" << hRatio
          << " lambda should not saturate against kLambdaCap";
    }
  }
}

// ---- Particle state at construction -------------------------------------

TEST(PBSPHFluidTest, NewParticlesHaveFiniteState)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.setEffectLength(0.1f);
  fluid.createParticle(Vector3df(0.1f, 0.2f, 0.3f), 0.025f);

  const auto& soa = fluid.getParticles();
  for (size_t i = 0; i < soa.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa.positions[i]));
    EXPECT_TRUE(isFinite3(soa.velocities[i]));
  }
}

// ---- Gravity drops y after one step -------------------------------------

TEST(PBSPHSolverTest, GravityDropsYAfterOneStep)
{
  auto fluid = makeDynamicFluid();
  const float initialY = 0.4f;

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 1);

  const auto& soa = fluid->getParticles();
  for (size_t i = 0; i < soa.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa.positions[i]));
    EXPECT_LT(soa.positions[i].y, initialY);
  }
}

// ---- Clear and re-add ---------------------------------------------------
// Verify that clear() empties the solver and that a newly added fluid
// can be simulated correctly afterwards.

TEST(PBSPHSolverTest, ClearAndReAddSimulatesCorrectly)
{
  PBSPHSolver solver;
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);

  auto fluidA = makeDynamicFluid();
  solver.add(fluidA.get());
  solver.clear();

  // After clear, register a fresh fluid and simulate without crash.
  auto fluidB = makeDynamicFluid();
  solver.add(fluidB.get());
  solver.simulate(0.01f, 1);

  for (const auto& pos : fluidB->getParticles().positions)
  {
    EXPECT_TRUE(isFinite3(pos));
  }
}

// ---- Two-fluid simulation -----------------------------------------------

TEST(PBSPHSolverTest, TwoFluidsAllParticlesStayFinite)
{
  auto fluidA = makeDynamicFluid();
  auto fluidB = makeDynamicFluid();
  // Shift fluidB so particles do not overlap with fluidA.
  fluidB->getParticles().densities.front() = 1000.0f; // touch particle to confirm it exists

  PBSPHFluid secondFluid;
  secondFluid.setRestDensity(1000.0f);
  secondFluid.setStiffness(200.0f);
  secondFluid.setVicsosity(0.01f);
  secondFluid.setEffectLength(0.1f);
  secondFluid.setIsBoundary(false);
  secondFluid.createParticle(Vector3df(0.3f, 0.4f, 0.0f), 0.025f);
  secondFluid.createParticle(Vector3df(0.35f, 0.4f, 0.0f), 0.025f);

  PBSPHSolver solver;
  solver.add(fluidA.get());
  solver.add(&secondFluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 1);

  const auto& soaA = fluidA->getParticles();
  for (size_t i = 0; i < soaA.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soaA.positions[i]));
    EXPECT_TRUE(isFinite3(soaA.velocities[i]));
  }
  const auto& soaB = secondFluid.getParticles();
  for (size_t i = 0; i < soaB.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soaB.positions[i]));
    EXPECT_TRUE(isFinite3(soaB.velocities[i]));
  }
}

// ---- Time-step / kernel setter interface alignment ----------------------
// Mirrors DFSPHSolverTest / WCSPHSolverTest: PBSPH must behave the same when
// driven through the ISPHSolver interface as through its concrete API.

TEST(PBSPHSolverTest, CommonEffectLengthInterfaceUpdatesRegisteredFluid)
{
  PBSPHFluid fluid;
  PBSPHSolver concreteSolver;
  concreteSolver.add(&fluid);
  ISPHSolver* solver = &concreteSolver;

  solver->setEffectLength(0.125f);

  EXPECT_FLOAT_EQ(fluid.getKernel()->getEffectLength(), 0.125f);
}

TEST(PBSPHSolverTest, TimeStepAndEffectLengthSettersAreSafeWithNoFluidRegistered)
{
  PBSPHSolver solver;
  // No fluid added: both setters must be harmless no-ops (setEffectLength
  // iterates an empty fluid list; setTimeStep/setMaxSubstep only store a scalar).
  solver.setTimeStep(0.005f);
  solver.setMaxSubstep(0.002f);
  solver.setEffectLength(0.1f);
  SUCCEED();
}

TEST(PBSPHSolverTest, SimulateAdvancesRequestedFrameDurationNotMaxSubstep)
{
  // PBSPH advances simulate()'s dt in a single position-based step. maxTimeStep
  // (set via setMaxSubstep/setTimeStep) only feeds the dt^2 factor that converts
  // a boundary penalty acceleration into a position correction, so with no
  // analytic/rigid boundary registered it must not change the result at all.
  auto makeFallingFluid = [] {
    auto f = std::make_unique<PBSPHFluid>();
    f->setRestDensity(1000.0f);
    f->setStiffness(200.0f);
    f->setVicsosity(0.0f);
    f->setEffectLength(0.1f);
    f->setIsBoundary(false);
    f->createParticle(Vector3df(0.0f, 5.0f, 0.0f), 0.025f);
    return f;
  };

  auto fluidSmall = makeFallingFluid();
  PBSPHSolver solverSmall;
  solverSmall.add(fluidSmall.get());
  solverSmall.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solverSmall.setMaxSubstep(0.0001f);
  solverSmall.simulate(0.01f, 2);

  auto fluidLarge = makeFallingFluid();
  PBSPHSolver solverLarge;
  solverLarge.add(fluidLarge.get());
  solverLarge.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solverLarge.setMaxSubstep(0.05f);
  solverLarge.simulate(0.01f, 2);

  const float ySmall = fluidSmall->getParticles().positions[0].y;
  const float yLarge = fluidLarge->getParticles().positions[0].y;

  EXPECT_TRUE(std::isfinite(ySmall));
  EXPECT_LT(ySmall, 5.0f);                    // the requested frame advanced
  EXPECT_NEAR(ySmall, yLarge, 1.0e-6f);       // maxSubstep did not gate it
}

// ---- Common solve diagnostics + configuration validation ---------------
// Mirrors DFSPHSolverTest / WCSPHSolverTest: getLastSolveStats() must report
// the same fields a shared UI/runner reads, and simulate() must reject an
// invalid or inconsistent configuration without touching particle state.

TEST(PBSPHSolverTest, ReportsFrameAdvanceThroughCommonSolveStats)
{
  auto fluid = makeDynamicFluid();

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 3);

  const auto stats = solver.getLastSolveStats();
  EXPECT_TRUE(stats.validConfiguration);
  EXPECT_TRUE(stats.converged);
  EXPECT_EQ(stats.substeps, 1);                       // single-step solver
  EXPECT_NEAR(stats.advancedTime, 0.01f, 1.0e-6f);
  EXPECT_EQ(stats.densityIterations, 3);              // == maxIter
  EXPECT_EQ(stats.divergenceIterations, 0);           // no divergence solve
}

TEST(PBSPHSolverTest, SimulateIsNoOpWhenEffectLengthNeverSet)
{
  // effectLength stays 0.f until setEffectLength() is called -> inert solver.
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.setStiffness(200.0f);
  fluid.setIsBoundary(false);
  fluid.createParticle(Vector3df(0.0f, 0.4f, 0.0f), 0.025f);
  const Vector3df before = fluid.getParticles().positions[0];

  PBSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.simulate(0.01f, 3);

  EXPECT_FALSE(solver.getLastSolveStats().validConfiguration);
  const Vector3df after = fluid.getParticles().positions[0];
  EXPECT_FLOAT_EQ(after.x, before.x);
  EXPECT_FLOAT_EQ(after.y, before.y);
  EXPECT_FLOAT_EQ(after.z, before.z);
}

TEST(PBSPHSolverTest, RejectsFluidsWithMismatchedKernelConfiguration)
{
  auto a = makeDynamicFluid();   // effectLength 0.1
  auto b = makeDynamicFluid();
  b->setEffectLength(0.2f);      // inconsistent SPH scale

  const Vector3df beforeA = a->getParticles().positions[0];
  const Vector3df beforeB = b->getParticles().positions[0];

  PBSPHSolver solver;
  solver.add(a.get());
  solver.add(b.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 3);

  EXPECT_FALSE(solver.getLastSolveStats().validConfiguration);
  EXPECT_FLOAT_EQ(a->getParticles().positions[0].y, beforeA.y);
  EXPECT_FLOAT_EQ(b->getParticles().positions[0].y, beforeB.y);
}

TEST(PBSPHSolverTest, RejectsFluidsWithMismatchedRestDensity)
{
  auto a = makeDynamicFluid();   // restDensity 1000
  auto b = makeDynamicFluid();
  b->setRestDensity(800.0f);     // inconsistent rest density

  const Vector3df beforeA = a->getParticles().positions[0];

  PBSPHSolver solver;
  solver.add(a.get());
  solver.add(b.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 3);

  EXPECT_FALSE(solver.getLastSolveStats().validConfiguration);
  EXPECT_FLOAT_EQ(a->getParticles().positions[0].y, beforeA.y);
}

TEST(PBSPHSolverTest, RejectsInvalidTimeStepWithoutTouchingParticleState)
{
  auto fluid = makeDynamicFluid();
  const Vector3df beforePos = fluid->getParticles().positions[0];
  const Vector3df beforeVel = fluid->getParticles().velocities[0];

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);

  solver.simulate(0.0f, 3);          // dt <= 0
  EXPECT_FALSE(solver.getLastSolveStats().validConfiguration);

  solver.simulate(0.01f, 0);         // maxIter <= 0
  EXPECT_FALSE(solver.getLastSolveStats().validConfiguration);

  const Vector3df afterPos = fluid->getParticles().positions[0];
  const Vector3df afterVel = fluid->getParticles().velocities[0];
  EXPECT_FLOAT_EQ(afterPos.y, beforePos.y);
  EXPECT_FLOAT_EQ(afterVel.y, beforeVel.y);
}

// ---- Analytic sphere boundary (common IShapeBoundary path) --------------
// Mirrors DFSPHSolverTest.SphereBoundaryUsesCommonShapeInterface: a closed
// spherical container registered via setBoundarySpheres() must keep the fluid
// inside through the same sample()/clampPosition() contract, and a distant
// sphere must be inert.

namespace
{
std::unique_ptr<PBSPHFluid> makeSphereTestFluid(const Vector3df& at)
{
  auto f = std::make_unique<PBSPHFluid>();
  f->setRestDensity(1000.0f);
  f->setStiffness(200.0f);
  f->setVicsosity(0.0f);
  f->setEffectLength(0.2f);
  f->setIsBoundary(false);
  f->createParticle(at, 0.025f);
  return f;
}
}

TEST(PBSPHSolverTest, SphereBoundaryUsesCommonShapeInterface)
{
  // Particle just outside a unit sphere container, no gravity: the wall must
  // push it back onto/into the sphere.
  auto fluid = makeSphereTestFluid(Vector3df(1.05f, 0.0f, 0.0f));

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setBoundarySpheres({ SphereBoundary(Vector3df(0.0f), 1.0f, 0.5f) }, 0.005f);
  solver.simulate(0.005f, 3);

  const Vector3df after = fluid->getParticles().positions[0];
  EXPECT_TRUE(std::isfinite(after.x));
  EXPECT_LT(after.x, 1.05f);                    // pushed back toward the center
  EXPECT_LE(glm::length(after), 1.0f + 1.0e-4f); // ended up inside the container
  EXPECT_TRUE(solver.getLastSolveStats().validConfiguration);
}

TEST(PBSPHSolverTest, SphereBoundaryKeepsFallingParticleInsideContainer)
{
  // Particle inside the sphere, gravity on: after many steps it must still be
  // inside -- the curved wall (not a box clamp) is what holds it.
  auto fluid = makeSphereTestFluid(Vector3df(0.0f, 0.5f, 0.0f));

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setBoundarySpheres({ SphereBoundary(Vector3df(0.0f), 1.0f, 0.5f) }, 0.005f);
  for (int step = 0; step < 60; ++step)
  {
    solver.simulate(0.005f, 3);
  }

  const Vector3df after = fluid->getParticles().positions[0];
  EXPECT_TRUE(std::isfinite(after.x) && std::isfinite(after.y) && std::isfinite(after.z));
  EXPECT_LE(glm::length(after), 1.0f + 1.0e-3f);
}

TEST(PBSPHSolverTest, DistantSphereBoundaryIsNoOp)
{
  // Particle well inside a large sphere, no other forces: the boundary must
  // not perturb it at all.
  auto fluid = makeSphereTestFluid(Vector3df(0.0f, 0.0f, 0.0f));
  const Vector3df before = fluid->getParticles().positions[0];

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setBoundarySpheres({ SphereBoundary(Vector3df(0.0f), 5.0f) }, 0.005f);
  solver.simulate(0.005f, 3);

  const Vector3df after = fluid->getParticles().positions[0];
  EXPECT_FLOAT_EQ(after.x, before.x);
  EXPECT_FLOAT_EQ(after.y, before.y);
  EXPECT_FLOAT_EQ(after.z, before.z);
}

// ---- Plate + generic shape boundary (common IShapeBoundary path) --------
// Mirrors DFSPHSolverTest.PlateBoundaryUsesSameRegistrationAndForcePath and
// ShapeBoundariesCanBeAddedAndClearedThroughCommonInterface.

TEST(PBSPHSolverTest, PlateBoundaryUsesSameRegistrationAndCorrectionPath)
{
  // Particle inside a thin plate slab, no external force: the finite plate
  // must push it back out through its nearest face (nonzero normal velocity).
  auto fluid = makeSphereTestFluid(Vector3df(0.0f, 0.0f, 0.0f));

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setBoundaryPlates(
      { PlateBoundary(Vector3df(0.0f), Vector3df(0.0f, 0.0f, 1.0f),
                      Vector3df(1.0f, 0.0f, 0.0f), 1.0f, 1.0f, 0.05f) },
      0.005f);
  solver.simulate(0.005f, 3);

  const float vz = fluid->getParticles().velocities[0].z;
  EXPECT_TRUE(std::isfinite(vz));
  EXPECT_GT(std::abs(vz), 0.0f);
  EXPECT_TRUE(solver.getLastSolveStats().validConfiguration);
}

TEST(PBSPHSolverTest, PlateBoundaryLetsParticlePastItsEdgeFallFree)
{
  // Same plate, but the particle sits well past the plate's finite edge:
  // the plate must be inert there and the particle just falls under gravity.
  auto fluid = makeSphereTestFluid(Vector3df(5.0f, 0.0f, 0.0f)); // far outside halfU = 1
  const float y0 = fluid->getParticles().positions[0].y;

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setBoundaryPlates(
      { PlateBoundary(Vector3df(0.0f), Vector3df(0.0f, 0.0f, 1.0f),
                      Vector3df(1.0f, 0.0f, 0.0f), 1.0f, 1.0f, 0.05f) },
      0.005f);
  for (int step = 0; step < 20; ++step) solver.simulate(0.005f, 3);

  const Vector3df after = fluid->getParticles().positions[0];
  EXPECT_LT(after.y, y0);                 // fell
  EXPECT_FLOAT_EQ(after.x, 5.0f);         // no sideways nudge from the plate
  EXPECT_FLOAT_EQ(after.z, 0.0f);
}

TEST(PBSPHSolverTest, ShapeBoundariesCanBeAddedAndClearedThroughCommonInterface)
{
  auto fluid = makeSphereTestFluid(Vector3df(1.05f, 0.0f, 0.0f));

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.addShapeBoundary(std::make_shared<SphereBoundary>(Vector3df(0.0f), 1.0f, 0.5f));
  solver.clearShapeBoundaries();          // drops it again
  solver.simulate(0.005f, 3);

  // No boundary left -> nothing acts on the (force-free) particle.
  EXPECT_FLOAT_EQ(fluid->getParticles().positions[0].x, 1.05f);
}

TEST(PBSPHSolverTest, PlaneRegisteredAsGenericShapeMatchesTypedPlane)
{
  // The same PlaneBoundary must produce the same correction whether it is
  // registered through setBoundaryPlanes() or through addShapeBoundary().
  const PlaneBoundary floor(Vector3df(0.0f, 1.0f, 0.0f), 0.0f); // y >= 0 is valid

  auto run = [&](bool generic) {
    auto fluid = makeSphereTestFluid(Vector3df(0.0f, -0.05f, 0.0f)); // penetrating the floor
    auto solver = std::make_unique<PBSPHSolver>();
    solver->add(fluid.get());
    solver->setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver->setTimeStep(0.005f);
    if (generic)
      solver->addShapeBoundary(std::make_shared<PlaneBoundary>(floor));
    else
      solver->setBoundaryPlanes({ floor }, 0.005f);
    for (int step = 0; step < 15; ++step) solver->simulate(0.005f, 3);
    return fluid->getParticles().positions[0];
  };

  const Vector3df typed = run(false);
  const Vector3df viaShape = run(true);
  EXPECT_NEAR(typed.x, viaShape.x, 1.0e-6f);
  EXPECT_NEAR(typed.y, viaShape.y, 1.0e-6f);
  EXPECT_NEAR(typed.z, viaShape.z, 1.0e-6f);
}

// ---- Common solver contract: static fluids, clear, particle getters ----
// Mirrors WCSPHSolverTest.StaticFluidDoesNotMoveUnderGravity /
// DynamicFluidFallsWhileStaticFluidStaysFixed / TwoFluidsAllParticlesStayFinite.
// (PBSPH-specific tests -- lambda scale invariance, no-saturation -- stay in
// PBSPHFluidTest and are not part of this shared matrix.)

TEST(PBSPHSolverTest, StaticFluidDoesNotMoveUnderGravity)
{
  auto fluid = makeDynamicFluid();
  fluid->setStatic(true);
  const Vector3df pos0Before = fluid->getParticles().positions[0];
  const Vector3df pos1Before = fluid->getParticles().positions[1];

  PBSPHSolver solver;
  solver.add(fluid.get());
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 3);

  EXPECT_NEAR(fluid->getParticles().positions[0].y, pos0Before.y, kTol);
  EXPECT_NEAR(fluid->getParticles().positions[1].y, pos1Before.y, kTol);
}

TEST(PBSPHSolverTest, DynamicFluidFallsWhileStaticFluidStaysFixed)
{
  auto dynamicFluid = makeDynamicFluid();
  dynamicFluid->setStatic(false);
  const float initialY = dynamicFluid->getParticles().positions[0].y;

  PBSPHFluid staticFluid;
  staticFluid.setRestDensity(1000.0f);
  staticFluid.setStiffness(200.0f);
  staticFluid.setVicsosity(0.01f);
  staticFluid.setEffectLength(0.1f);
  staticFluid.setStatic(true);
  staticFluid.createParticle(Vector3df(0.5f, initialY, 0.0f), 0.025f);
  const float staticY = staticFluid.getParticles().positions[0].y;

  PBSPHSolver solver;
  solver.add(dynamicFluid.get());
  solver.add(&staticFluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(makeBoundaryBox(), 0.01f);
  solver.simulate(0.01f, 3);

  EXPECT_NEAR(staticFluid.getParticles().positions[0].y, staticY, kTol);
  for (const auto& pos : dynamicFluid->getParticles().positions)
  {
    EXPECT_LT(pos.y, initialY);
  }
}

TEST(PBSPHSolverTest, ClearRemovesAllRegisteredFluids)
{
  auto fluidA = makeDynamicFluid();
  auto fluidB = makeDynamicFluid();

  PBSPHSolver solver;
  solver.add(fluidA.get());
  solver.add(fluidB.get());
  EXPECT_EQ(solver.getParticleCount(), 4);

  solver.clear();
  EXPECT_EQ(solver.getParticleCount(), 0);
  EXPECT_TRUE(solver.getParticlePositions().empty());
  EXPECT_TRUE(solver.getParticleVelocities().empty());
  EXPECT_TRUE(solver.getParticleDensities().empty());
}

TEST(PBSPHSolverTest, ParticleGettersAreConsistentInOrderAndCountAcrossFluids)
{
  PBSPHFluid fluidA;
  fluidA.setRestDensity(1000.0f);
  fluidA.setEffectLength(0.1f);
  fluidA.createParticle(Vector3df(0.0f, 0.4f, 0.0f), 0.025f);
  fluidA.createParticle(Vector3df(0.05f, 0.4f, 0.0f), 0.025f);
  fluidA.createParticle(Vector3df(0.10f, 0.4f, 0.0f), 0.025f);

  PBSPHFluid fluidB;
  fluidB.setRestDensity(1000.0f);
  fluidB.setEffectLength(0.1f);
  fluidB.createParticle(Vector3df(0.5f, 0.4f, 0.0f), 0.025f);
  fluidB.createParticle(Vector3df(0.6f, 0.4f, 0.0f), 0.025f);

  PBSPHSolver solver;
  solver.add(&fluidA);
  solver.add(&fluidB);

  const int n = solver.getParticleCount();
  EXPECT_EQ(n, 5);

  const auto positions  = solver.getParticlePositions();
  const auto velocities = solver.getParticleVelocities();
  const auto densities  = solver.getParticleDensities();
  ASSERT_EQ(positions.size(), static_cast<size_t>(n));
  ASSERT_EQ(velocities.size(), static_cast<size_t>(n));
  ASSERT_EQ(densities.size(), static_cast<size_t>(n));

  // Order: all of fluidA's particles, then all of fluidB's, matching each
  // fluid's own getParticles() order.
  for (size_t i = 0; i < fluidA.getParticles().size(); ++i)
  {
    EXPECT_FLOAT_EQ(positions[i].x, fluidA.getParticles().positions[i].x);
  }
  for (size_t i = 0; i < fluidB.getParticles().size(); ++i)
  {
    EXPECT_FLOAT_EQ(positions[fluidA.getParticles().size() + i].x,
                    fluidB.getParticles().positions[i].x);
  }
}
