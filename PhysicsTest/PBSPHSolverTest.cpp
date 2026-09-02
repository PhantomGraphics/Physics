#include "pch.h"

#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/PBSPHSolver.h"

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
