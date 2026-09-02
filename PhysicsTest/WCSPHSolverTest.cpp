#include "pch.h"

#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/SPHKernel.h"
#include "../Physics/SphereBoundary.h"
#include "../Physics/PlateBoundary.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;

bool isFinite3(const Vector3df& v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

Box3df makeDefaultBox()
{
  return Box3df(Vector3df(-2.0f, -2.0f, -2.0f), Vector3df(2.0f, 2.0f, 2.0f));
}
}

// ---- WCSPHFluid getter/setter round-trips --------------------------------

TEST(WCSPHFluidTest, GettersReturnSetValues)
{
  WCSPHFluid fluid;
  fluid.setDensity(800.0f);
  fluid.setPressureCoe(150.0f);
  fluid.setVicosityCoe(0.03f);
  fluid.setTensionCoe(0.07f);
  fluid.setEffectLength(0.2f);
  fluid.setStatic(true);

  EXPECT_FLOAT_EQ(fluid.getDensity(), 800.0f);
  EXPECT_FLOAT_EQ(fluid.getPressureCoe(), 150.0f);
  EXPECT_FLOAT_EQ(fluid.getViscosityCoe(), 0.03f);
  EXPECT_FLOAT_EQ(fluid.getTensionCoe(), 0.07f);
  EXPECT_FLOAT_EQ(fluid.getEffectLength(), 0.2f);
  EXPECT_TRUE(fluid.isStatic());

  fluid.setStatic(false);
  EXPECT_FALSE(fluid.isStatic());
}

// ---- Particle creation and bounding box ---------------------------------

TEST(WCSPHFluidTest, CreateParticleIncreasesCount)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setEffectLength(0.1f);

  EXPECT_EQ(fluid.getNumParticles(), 0);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.001f);
  EXPECT_EQ(fluid.getNumParticles(), 1);

  fluid.createParticle(Vector3df(1.0f, 0.0f, 0.0f), 0.001f);
  EXPECT_EQ(fluid.getNumParticles(), 2);
}

TEST(WCSPHFluidTest, BoundingBoxContainsAllParticles)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setEffectLength(0.1f);
  fluid.createParticle(Vector3df(-0.5f, 0.3f, 0.1f), 0.001f);
  fluid.createParticle(Vector3df( 0.5f, -0.3f, -0.1f), 0.001f);

  const auto bb = fluid.getBoundingBox();
  EXPECT_LE(bb.getMin().x, -0.5f + kTol);
  EXPECT_LE(bb.getMin().y, -0.3f + kTol);
  EXPECT_GE(bb.getMax().x,  0.5f - kTol);
  EXPECT_GE(bb.getMax().y,  0.3f - kTol);
}

// ---- Emitter (docs/todo/PLAN_physics_fluid_emitter.md) -------------------

TEST(WCSPHFluidTest, UpdateEmittersIsNoOpWithoutRegisteredEmitters)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);

  fluid.updateEmitters(0.1f);

  EXPECT_EQ(fluid.getNumParticles(), 0);
}

// rate=100/sec * dt=0.01 == 1.0 exactly each step, so the accumulator carries
// no leftover fraction and the emitted count is deterministic (not dependent
// on the RNG, which only affects position/velocity jitter).
TEST(WCSPHFluidTest, UpdateEmittersEmitsAtTheConfiguredRate)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);

  Emitter e;
  e.center = Vector3df(0.0f, 0.0f, 0.0f);
  e.radius = 0.05f;
  e.rate = 100.0f;
  fluid.addEmitter(e);

  for (int step = 0; step < 10; ++step)
  {
    fluid.updateEmitters(0.01f);
  }

  EXPECT_EQ(fluid.getNumParticles(), 10);
}

TEST(WCSPHFluidTest, UpdateEmittersStopsAtMaxParticles)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setMaxParticles(5);

  Emitter e;
  e.rate = 10000.0f; // far more than fits under the cap in one step.
  fluid.addEmitter(e);

  fluid.updateEmitters(1.0f);

  EXPECT_EQ(fluid.getNumParticles(), 5);
}

// direction=(0,1,0), speedJitter=0 -> every spawned particle's velocity is
// exactly (0, speed, 0), independent of the RNG (which only jitters position
// within the emission disk here).
TEST(WCSPHFluidTest, UpdateEmittersGivesSpawnedParticlesVelocityAlongDirection)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);

  Emitter e;
  e.rate = 100.0f;
  e.direction = Vector3df(0.0f, 1.0f, 0.0f);
  e.speed = 2.0f;
  e.speedJitter = 0.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(0.01f);

  ASSERT_EQ(fluid.getNumParticles(), 1);
  const Vector3df vel = fluid.getParticles().velocities[0];
  EXPECT_NEAR(vel.x, 0.0f, kTol);
  EXPECT_NEAR(vel.y, 2.0f, kTol);
  EXPECT_NEAR(vel.z, 0.0f, kTol);
}

// The emission disk must lie *across* the jet, not edge-on to it. With
// direction=(0,0,-1) the spawned particles have to spread in both X and Y and
// have zero thickness along Z; the old XZ-plane-only emission offset produced
// exactly the opposite -- a zero-thickness ribbon in Y standing parallel to
// the flow, whose SPH density had nothing to do with the intended jet's
// (docs/todo/PLAN_sph_showcase_water_sphere.md section 2.3).
TEST(WCSPHFluidTest, UpdateEmittersSpawnsDiskPerpendicularToDirection)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);

  const Vector3df center(0.0f, 0.0f, 0.15f);
  const float diskRadius = 0.075f;

  Emitter e;
  e.center = center;
  e.radius = diskRadius;
  e.particleRadius = 0.0025f;
  e.direction = Vector3df(0.0f, 0.0f, -1.0f);
  e.speed = 1.0f;
  e.speedJitter = 0.0f;
  e.rate = 100000.0f;   // 100000 * 0.01 == 1000 particles, under maxParticles_.
  fluid.addEmitter(e);

  fluid.updateEmitters(0.01f);
  ASSERT_EQ(fluid.getNumParticles(), 1000);

  float maxAbsX = 0.0f;
  float maxAbsY = 0.0f;
  float maxAbsZ = 0.0f;
  for (const auto& pos : fluid.getParticles().positions)
  {
    const Vector3df offset = pos - center;
    maxAbsX = std::max(maxAbsX, std::abs(offset.x));
    maxAbsY = std::max(maxAbsY, std::abs(offset.y));
    maxAbsZ = std::max(maxAbsZ, std::abs(offset.z));
    // Still a disk of the requested radius, just in a different plane.
    EXPECT_LE(std::sqrt(offset.x * offset.x + offset.y * offset.y), diskRadius + kTol);
  }

  EXPECT_NEAR(maxAbsZ, 0.0f, kTol);
  // 1000 uniform samples reach well past 80% of the radius in both spanning
  // axes with overwhelming probability; a degenerate ribbon would leave one of
  // them at exactly zero.
  EXPECT_GT(maxAbsX, diskRadius * 0.8f);
  EXPECT_GT(maxAbsY, diskRadius * 0.8f);
}

// No two particles may be born on top of each other. Uniform-random sampling
// of the emission disk makes the stream a Poisson point process, and a Poisson
// process clumps hard: at the correct mean density (one particle per
// spacing^3) roughly 41% of particles land within half a spacing of a
// neighbor. SPH's rest density is calibrated against a regular lattice, so
// each of those pairs starts at rho >> rho0 and WCSPH's pressure term blows
// them apart the instant they exist -- the jet shreds into spray at the nozzle
// instead of falling as a column. Observed for real in the water-sphere
// showcase before the lattice emitter replaced the random one.
//
// One updateEmitters() call spawns every particle at the same instant, so the
// whole batch lies in one plane and the separation being checked here is
// purely the in-disk lattice spacing.
TEST(WCSPHFluidTest, UpdateEmittersNeverSpawnsParticlesOnTopOfEachOther)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);

  const float particleRadius = 0.0025f;
  const float spacing = particleRadius * 2.0f;
  const float diskRadius = 0.075f;

  Emitter e;
  e.center = Vector3df(0.0f, 0.0f, 0.15f);
  e.radius = diskRadius;
  e.particleRadius = particleRadius;
  e.direction = Vector3df(0.0f, 0.0f, -1.0f);
  e.speed = 2.45f;
  e.speedJitter = 0.0f;
  // One full lattice cycle: pi*R^2/spacing^2 ~ 707 sites for this disk.
  e.rate = 700.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(1.0f);
  const auto& positions = fluid.getParticles().positions;
  ASSERT_EQ(positions.size(), 700U);

  float minSeparation = std::numeric_limits<float>::max();
  for (size_t i = 0; i < positions.size(); ++i)
  {
    for (size_t j = i + 1; j < positions.size(); ++j)
    {
      minSeparation = std::min(minSeparation, getDistance(positions[i], positions[j]));
    }
  }

  // On the lattice the closest pair is exactly one spacing apart. The old
  // random sampler routinely produced pairs an order of magnitude closer.
  EXPECT_NEAR(minSeparation, spacing, 1.0e-4f);
}

// A correctly configured emitter (rate = pi*R^2*v / spacing^3) must complete
// exactly one lattice cycle in the time the jet travels one spacing --
// that identity is what makes successive cycles stack into the cubic lattice
// SPH's rest density assumes, and it is why cycling the lattice needs no
// bookkeeping beyond an index.
TEST(WCSPHFluidTest, LatticeCycleMatchesOneParticleSpacingOfJetTravel)
{
  const float particleRadius = 0.0025f;
  const float spacing = particleRadius * 2.0f;
  const float diskRadius = 0.075f;
  const float speed = 2.45f;

  const auto offsets = makeDiskLatticeOffsets(
      diskRadius, spacing, Vector3df(0.0f, 0.0f, -1.0f));

  // Rate the preset derives from the volumetric flow (design section 3.4).
  const float flow = 3.14159265f * diskRadius * diskRadius * speed;
  const float rate = flow / (spacing * spacing * spacing);

  const float cycleSeconds = offsets.size() / rate;
  const float travelled = cycleSeconds * speed;

  EXPECT_NEAR(travelled, spacing, spacing * 0.02f);
}

TEST(WCSPHFluidTest, ClearEmittersRemovesRegisteredEmitters)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);

  Emitter e;
  fluid.addEmitter(e);
  EXPECT_EQ(fluid.getEmitters().size(), 1U);

  fluid.clearEmitters();
  EXPECT_TRUE(fluid.getEmitters().empty());

  fluid.updateEmitters(1.0f);
  EXPECT_EQ(fluid.getNumParticles(), 0);
}

// ---- Outflow region (optional particle removal) --------------------------

TEST(WCSPHFluidTest, RemoveOutflowParticlesIsNoOpWithoutRegisteredRegions)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.05f);

  fluid.removeOutflowParticles();

  EXPECT_EQ(fluid.getNumParticles(), 1);
}

TEST(WCSPHFluidTest, RemoveOutflowParticlesDeletesParticlesInsideRegionOnly)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, -10.0f, 0.0f), 0.05f); // inside
  fluid.createParticle(Vector3df(0.0f, 10.0f, 0.0f), 0.05f);  // outside

  OutflowRegion r;
  r.bounds = Box3df(Vector3df(-1.0f, -11.0f, -1.0f), Vector3df(1.0f, -9.0f, 1.0f));
  fluid.addOutflowRegion(r);

  fluid.removeOutflowParticles();

  ASSERT_EQ(fluid.getNumParticles(), 1);
  const Vector3df remaining = fluid.getPosition(0);
  EXPECT_NEAR(remaining.y, 10.0f, kTol);
}

TEST(WCSPHFluidTest, ClearOutflowRegionsRemovesRegisteredRegions)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.05f);

  OutflowRegion r;
  r.bounds = Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f));
  fluid.addOutflowRegion(r);
  EXPECT_EQ(fluid.getOutflowRegions().size(), 1U);

  fluid.clearOutflowRegions();
  EXPECT_TRUE(fluid.getOutflowRegions().empty());

  fluid.removeOutflowParticles();
  EXPECT_EQ(fluid.getNumParticles(), 1);
}

// ---- Solver registration ------------------------------------------------

TEST(WCSPHSolverTest, GetFluidsReturnsRegisteredFluids)
{
  WCSPHFluid fluidA;
  WCSPHFluid fluidB;

  WCSPHSolver solver;
  solver.add(&fluidA);
  solver.add(&fluidB);

  const auto& fluids = solver.getFluids();
  ASSERT_EQ(fluids.size(), 2U);
  EXPECT_EQ(fluids[0], &fluidA);
  EXPECT_EQ(fluids[1], &fluidB);
}

TEST(WCSPHSolverTest, ReportsFrameAdvanceThroughCommonSolveStats)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(0.0f);
  fluid.setVicosityCoe(0.0f);
  fluid.setEffectLength(0.2f);
  fluid.createParticle(Vector3df(0.0f), 0.025f);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f));
  solver.setMaxSubstep(0.005f);
  solver.simulate(0.005f, 2);

  const auto stats = solver.getLastSolveStats();
  EXPECT_TRUE(stats.validConfiguration);
  EXPECT_TRUE(stats.converged);
  EXPECT_EQ(stats.substeps, 1);
  EXPECT_FLOAT_EQ(stats.advancedTime, 0.005f);
}

TEST(WCSPHSolverTest, RejectsFluidsWithMismatchedKernelConfiguration)
{
  WCSPHFluid a;
  WCSPHFluid b;
  for (auto* fluid : { &a, &b }) {
    fluid->setDensity(1000.0f);
    fluid->setPressureCoe(0.0f);
    fluid->setVicosityCoe(0.0f);
    fluid->createParticle(Vector3df(0.0f), 0.025f);
  }
  a.setEffectLength(0.1f);
  b.setEffectLength(0.2f);

  WCSPHSolver solver;
  solver.add(&a);
  solver.add(&b);
  solver.simulate(0.005f, 1);

  EXPECT_FALSE(solver.getLastSolveStats().validConfiguration);
  EXPECT_FLOAT_EQ(a.getParticles().positions[0].y, 0.0f);
  EXPECT_FLOAT_EQ(b.getParticles().positions[0].y, 0.0f);
}

// ---- simulate() is a no-op when effectLength was never set --------------
// docs/todo/PLAN_sph_scale_invariance.md Phase 5: effectLength defaults to
// 0.f (SPHKernel/WCSPHFluid) until setEffectLength() is called, so a caller
// that forgets to call it must get an inert solver, not an undefined/UB
// search radius fed to IndexedSortBasedSearcher.

TEST(WCSPHSolverTest, Simulate_IsNoOpWhenEffectLengthNeverSet)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.createParticle(Vector3df(0.0f, 1.0f, 0.0f), 0.001f);

  const Vector3df posBefore = fluid.getParticles().positions[0];

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  // Deliberately not calling setEffectLength() on either the fluid or the
  // solver.
  solver.simulate(0.01f, 1);

  EXPECT_TRUE(isFinite3(fluid.getParticles().positions[0]));
  EXPECT_FLOAT_EQ(fluid.getParticles().positions[0].x, posBefore.x);
  EXPECT_FLOAT_EQ(fluid.getParticles().positions[0].y, posBefore.y);
  EXPECT_FLOAT_EQ(fluid.getParticles().positions[0].z, posBefore.z);
}

// ---- Static fluid does not move -----------------------------------------

TEST(WCSPHSolverTest, StaticFluidDoesNotMoveUnderGravity)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(200.0f);
  fluid.setVicosityCoe(0.05f);
  fluid.setEffectLength(0.1f);
  fluid.setStatic(true);
  fluid.createParticle(Vector3df(0.0f, 0.5f, 0.0f), 0.001f);
  fluid.createParticle(Vector3df(0.1f, 0.5f, 0.0f), 0.001f);

  const Vector3df pos0Before = fluid.getParticles().positions[0];
  const Vector3df pos1Before = fluid.getParticles().positions[1];

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setEffectLength(0.1f);
  solver.setBoundary(makeDefaultBox(), 0.01f);
  solver.simulate(0.01f, 1);

  EXPECT_NEAR(fluid.getParticles().positions[0].y, pos0Before.y, kTol);
  EXPECT_NEAR(fluid.getParticles().positions[1].y, pos1Before.y, kTol);
}

// ---- Dynamic fluid falls while static fluid stays -----------------------
// Register both a static and a dynamic fluid in one solver and verify
// that after one step only the dynamic fluid has moved downward.

TEST(WCSPHSolverTest, DynamicFluidFallsWhileStaticFluidStaysFixed)
{
  const float initialY = 0.5f;

  WCSPHFluid dynamic;
  dynamic.setDensity(1000.0f);
  dynamic.setPressureCoe(200.0f);
  dynamic.setVicosityCoe(0.05f);
  dynamic.setEffectLength(0.1f);
  dynamic.setStatic(false);
  dynamic.createParticle(Vector3df(0.0f,  initialY, 0.0f), 0.001f);
  dynamic.createParticle(Vector3df(0.05f, initialY, 0.0f), 0.001f);

  WCSPHFluid staticFluid;
  staticFluid.setDensity(1000.0f);
  staticFluid.setPressureCoe(200.0f);
  staticFluid.setVicosityCoe(0.05f);
  staticFluid.setEffectLength(0.1f);
  staticFluid.setStatic(true);
  staticFluid.createParticle(Vector3df(0.3f, initialY, 0.0f), 0.001f);

  const float staticY = staticFluid.getParticles().positions[0].y;

  WCSPHSolver solver;
  solver.add(&dynamic);
  solver.add(&staticFluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setEffectLength(0.1f);
  solver.setBoundary(makeDefaultBox(), 0.01f);
  solver.simulate(0.01f, 1);

  // Static fluid must not move.
  EXPECT_NEAR(staticFluid.getParticles().positions[0].y, staticY, kTol);

  // Dynamic fluid must be below its initial position.
  for (const auto& pos : dynamic.getParticles().positions)
  {
    EXPECT_LT(pos.y, initialY);
  }
}

// ---- Two-fluid simulation -----------------------------------------------

TEST(WCSPHSolverTest, TwoFluidsAllParticlesStayFinite)
{
    WCSPHFluid fluidA;
  fluidA.setDensity(1000.0f);
  fluidA.setPressureCoe(200.0f);
  fluidA.setVicosityCoe(0.05f);
  fluidA.setEffectLength(0.1f);
  fluidA.setStatic(false);
  fluidA.createParticle(Vector3df(-0.2f, 0.4f, 0.0f), 0.001f);
  fluidA.createParticle(Vector3df(-0.1f, 0.4f, 0.0f), 0.001f);

  WCSPHFluid fluidB;
  fluidB.setDensity(1000.0f);
  fluidB.setPressureCoe(200.0f);
  fluidB.setVicosityCoe(0.05f);
  fluidB.setEffectLength(0.1f);
  fluidB.setStatic(false);
  fluidB.createParticle(Vector3df(0.1f, 0.4f, 0.0f), 0.001f);
  fluidB.createParticle(Vector3df(0.2f, 0.4f, 0.0f), 0.001f);

  WCSPHSolver solver;
  solver.add(&fluidA);
  solver.add(&fluidB);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setEffectLength(0.1f);
  solver.setBoundary(makeDefaultBox(), 0.01f);
  solver.simulate(0.01f, 1);

  for (size_t i = 0; i < fluidA.getParticles().size(); ++i)
  {
    EXPECT_TRUE(isFinite3(fluidA.getParticles().positions[i]));
    EXPECT_TRUE(isFinite3(fluidA.getParticles().velocities[i]));
  }
  for (size_t i = 0; i < fluidB.getParticles().size(); ++i)
  {
    EXPECT_TRUE(isFinite3(fluidB.getParticles().positions[i]));
    EXPECT_TRUE(isFinite3(fluidB.getParticles().velocities[i]));
  }
}

// ---- Position accessor consistency --------------------------------------

TEST(CSPHFluidTest, GetPositionMatchesParticlePosition)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setEffectLength(0.1f);
  fluid.createParticle(Vector3df(0.3f, 0.7f, -0.1f), 0.001f);
  fluid.createParticle(Vector3df(-0.2f, 0.1f, 0.5f), 0.001f);

  for (int i = 0; i < fluid.getNumParticles(); ++i)
  {
    const auto via_accessor = fluid.getPosition(i);
    const auto via_particle = fluid.getParticles().positions[i];
    EXPECT_NEAR(via_accessor.x, via_particle.x, kTol);
    EXPECT_NEAR(via_accessor.y, via_particle.y, kTol);
    EXPECT_NEAR(via_accessor.z, via_particle.z, kTol);
  }
}

// ---- estimatePressureCoe(): pressureCoe = pressureCoeScale * effectLength -
// 2026-08-17: the earlier gravity/target-density-error-ratio/rest-density
// based derivation (docs/todo/PLAN_sph_scale_invariance.md section 4) was
// retired in favor of a plain proportionality, per user request. These tests
// lock in the new, simpler contract.

TEST(WCSPHFluidTest, EstimatePressureCoeIsLinearInEffectLength)
{
  const float scale = 1960.0f;
  for (float h : {0.01f, 0.6f, 1.0f, 10.0f})
  {
    EXPECT_NEAR(WCSPHFluid::estimatePressureCoe(h, scale), scale * h, kTol);
  }
}

TEST(WCSPHFluidTest, EstimatePressureCoeIsLinearInPressureCoeScale)
{
  const float h = 0.6f;
  const float base = WCSPHFluid::estimatePressureCoe(h, 98.0f);
  const float doubled = WCSPHFluid::estimatePressureCoe(h, 196.0f);
  EXPECT_NEAR(doubled, 2.0f * base, kTol);
}

TEST(WCSPHFluidTest, SetPressureCoeFromScaleUsesEffectLengthTimesScale)
{
  WCSPHFluid fluid;
  fluid.setEffectLength(0.6f);
  fluid.setPressureCoeFromScale(98.0f);
  EXPECT_NEAR(fluid.getPressureCoe(), 58.8f, kTol);
}

// ---- viscosityCoe is a kinematic viscosity, so it must NOT be scaled ------
//
// This replaces a test that asserted the opposite. The old
// WCSPHFluid::estimateViscosityCoe() derived viscosityCoe ~ effectLength^1.5
// and the old test confirmed it -- but it did so by placing exactly *two*
// particles and driving them with a fixed absolute velocity difference. Real
// fluid does not behave that way: the neighbor count is invariant under h, and
// the velocity difference between adjacent particles shrinks as
// dv/dx * spacing ~ h, which exactly cancels the 1/h^2 the two-particle setup
// measures.
//
// solveViscosityForce() adds viscosityCoe * dv * laplacian(W_visc) * m_j and
// forwardTime() divides by rho, so relative to Muller et al. 2003's
// mu * sum_j m_j dv/rho_j * laplacian(W) the 1/rho_j is missing and
// viscosityCoe == mu/rho: the kinematic viscosity (m^2/s) itself. Through the
// SPH Laplacian the acceleration is viscosityCoe * laplacian(v), which carries
// no h at all.
//
// So the property worth pinning is resolution independence: the same
// viscosityCoe must damp the same physical flow by the same amount at two
// resolutions. Measured in docs/issue/
// water_sphere_showcase_emitter_instability.md 11.2 (agreement within 5%
// across a 2x resolution change); this is that measurement, shrunk to unit
// test size. The estimator and its "scale viscosity with h" guidance are
// gone -- viscosityCoe cannot be derived from the kernel radius, because the
// length scale that matters belongs to the flow, not to the discretization.

namespace
{
// Two blocks of fluid colliding head on, with no gravity and no walls: the
// same physical scene at whatever resolution `spacing` asks for (the blocks
// are always the same 0.06 m cubes closing at the same speed). Returns the
// peak speed left after `seconds`, which is how much relative motion the
// viscous term failed to remove.
//
// Pressure is left on, at the codebase's usual k = scale * h. Turning it off
// to "isolate" viscosity does not work: without it the two blocks pass
// straight through each other and the peak speed stops measuring damping at
// all -- it goes *up* with viscosity, because the Muller viscous force points
// along dv and drags interpenetrating layers past one another.
float headOnCollisionPeakSpeed(const float spacing, const float viscosityCoe)
{
  constexpr float kCube = 0.06f;
  constexpr float kClosingSpeed = 1.7f;    // 3.4 m/s of relative motion
  constexpr float kSeconds = 0.06f;
  constexpr float kPressureCoeScale = 20.0f;
  constexpr float kDt = 1.0e-4f;

  const float radius = spacing * 0.5f;
  const float h = spacing * 2.0f;
  const int n = static_cast<int>(std::lround(kCube / spacing));

  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(kPressureCoeScale * h);
  fluid.setVicosityCoe(viscosityCoe);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(h);
  fluid.setMaxParticles(2 * n * n * n + 1);

  const float gap = kCube + spacing;
  for (int block = 0; block < 2; ++block) {
    for (int i = 0; i < n; ++i) {
      for (int j = 0; j < n; ++j) {
        for (int k = 0; k < n; ++k) {
          fluid.createParticle(
              Vector3df(i * spacing, j * spacing, k * spacing + block * gap), radius);
        }
      }
    }
  }

  auto& soa = fluid.getParticles();
  const size_t half = soa.size() / 2;
  for (size_t i = 0; i < soa.size(); ++i) {
    soa.velocities[i] = Vector3df(0.0f, 0.0f, (i < half) ? kClosingSpeed : -kClosingSpeed);
  }

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(h);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));

  const int steps = static_cast<int>(std::lround(kSeconds / kDt));
  for (int step = 0; step < steps; ++step) {
    solver.simulate(kDt, 1);
  }

  float peak = 0.0f;
  for (size_t i = 0; i < soa.size(); ++i) {
    const auto& v = soa.velocities[i];
    peak = std::max(peak, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
  }
  return peak;
}
}

TEST(WCSPHFluidTest, ViscousDampingIsResolutionIndependentForAFixedViscosityCoe)
{
  constexpr float kViscosityCoe = 0.01f;

  const float coarse = headOnCollisionPeakSpeed(0.010f, kViscosityCoe);   //  432 particles
  const float fine   = headOnCollisionPeakSpeed(0.005f, kViscosityCoe);   // 3456 particles

  // Undamped control: without it, a bug that killed all motion would pass.
  const float undamped = headOnCollisionPeakSpeed(0.010f, 0.0f);
  ASSERT_GT(undamped, coarse * 2.0f)
      << "viscosity is not actually damping anything; the comparison below is vacuous"
      << " (undamped=" << undamped << " damped=" << coarse << ")";

  // Doubling the resolution must not change the damping. It would if
  // viscosityCoe carried a hidden power of h: the deleted effectLength^1.5
  // rule would have called for 2.8x the coefficient at the finer spacing.
  // Measured: 0.845 vs 0.834, i.e. 1.3% apart (docs/issue/
  // water_sphere_showcase_emitter_instability.md 11.2 measured the same pair
  // as 0.843 / 0.833).
  EXPECT_NEAR(fine, coarse, coarse * 0.10f)
      << "coarse=" << coarse << " fine=" << fine;
}

// ---- Surface tension: cohesion (attraction) -------------------------------
// docs/todo/PLAN_sph_surface_tension.md Phase 2: the classic Muller et al.
// 2003 color-field model implemented in solveSurfaceTension() is only
// attractive once two particles are far enough apart that the Poly6
// Laplacian's sign flips positive (r > sqrt(3/7)*effectLength =~
// 0.6547*effectLength -- SPHKernel::getPoly6KernelLaplacian()'s (42r^2-18h^2)
// factor); closer than that it is repulsive, which prevents particles from
// collapsing into each other. This places two particles inside that
// attractive band and checks that tensionCoe > 0 pulls them toward each
// other (with pressure/viscosity/gravity zeroed out so tension is the only
// force in play).

TEST(WCSPHSolverTest, SurfaceTensionPullsTwoNearbyParticlesTogether)
{
  const float h = 1.0f;
  const float r = 0.85f * h;

  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(0.0f);
  fluid.setVicosityCoe(0.0f);
  fluid.setTensionCoe(1.0f);
  fluid.setEffectLength(h);
  fluid.setStatic(false);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.3f);
  fluid.createParticle(Vector3df(r, 0.0f, 0.0f), 0.3f);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.001f);
  solver.setEffectLength(h);
  solver.simulate(0.001f, 1);

  ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[0]));
  ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[1]));
  // p0 sits at x=0, p1 at x=r>0: attraction means each is accelerated toward
  // the other.
  EXPECT_GT(fluid.getParticles().velocities[0].x, 0.0f);
  EXPECT_LT(fluid.getParticles().velocities[1].x, 0.0f);
}

// ---- Surface tension: threshold is scale-invariant -------------------------
// docs/todo/PLAN_sph_surface_tension.md Phase 2 point 3: surfaceNormalHat()'s
// "is this a surface particle" threshold compares getLengthSquared(normal) *
// effectLength^2 against a fixed dimensionless constant (not the raw,
// unit-bearing normal magnitude) specifically so the same threshold applies
// regardless of scene scale. Reruns the cohesion test above at radius=1,
// 0.1, 0.01 (same relative geometry each time) and checks attraction still
// triggers at every scale -- an absolute-magnitude threshold would fail this
// at small scales, since the raw normal magnitude shrinks with h.

TEST(WCSPHSolverTest, SurfaceTensionCohesionTriggersAcrossScales)
{
  for (const float h : {1.0f, 0.1f, 0.01f})
  {
    const float r = 0.85f * h;

    WCSPHFluid fluid;
    fluid.setDensity(1000.0f);
    fluid.setPressureCoe(0.0f);
    fluid.setVicosityCoe(0.0f);
    fluid.setTensionCoe(1.0f);
    fluid.setEffectLength(h);
    fluid.setStatic(false);
    fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.3f * h);
    fluid.createParticle(Vector3df(r, 0.0f, 0.0f), 0.3f * h);

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
    solver.setTimeStep(0.001f);
    solver.setEffectLength(h);
    solver.simulate(0.001f, 1);

    ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[0])) << "h=" << h;
    ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[1])) << "h=" << h;
    EXPECT_GT(fluid.getParticles().velocities[0].x, 0.0f) << "h=" << h;
    EXPECT_LT(fluid.getParticles().velocities[1].x, 0.0f) << "h=" << h;
  }
}

// ---- Surface tension: stability while settling -----------------------------
// Mirrors FluidPoolStabilityTest's tension-free baseline (a small block
// falling into a box boundary) with tensionCoe > 0 added on top of pressure
// and viscosity, checking the extra force pass doesn't destabilize the
// solver.

TEST(WCSPHSolverTest, SurfaceTensionBlockStaysFiniteWhileSettling)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(200.0f);
  fluid.setVicosityCoe(0.05f);
  fluid.setTensionCoe(0.01f);
  fluid.setEffectLength(0.1f);
  fluid.setStatic(false);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        fluid.createParticle(Vector3df(-0.05f + 0.05f * i, 0.3f + 0.05f * j, -0.05f + 0.05f * k), 0.001f);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setEffectLength(0.1f);
  // Matches FluidPoolStabilityTest's tension-free baseline box (not
  // makeDefaultBox()'s wider +-2, which the -0.6/0.6 settle-bound checks
  // below aren't sized for).
  solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.01f);

  for (int step = 0; step < 200; ++step)
  {
    solver.simulate(0.005f, 1);
    for (size_t i = 0; i < fluid.getParticles().size(); ++i)
    {
      ASSERT_TRUE(isFinite3(fluid.getParticles().positions[i])) << "step " << step;
      ASSERT_TRUE(isFinite3(fluid.getParticles().velocities[i])) << "step " << step;
    }
  }

  for (const auto& pos : fluid.getParticles().positions)
  {
    EXPECT_GT(pos.y, -0.6f);
    EXPECT_LT(pos.y, 0.6f);
  }
}

// ---- Surface tension: normals vanish in the interior -----------------------
// solveNormal() sums (m_j/rho_j) * gradient(Poly6) over neighbors; for a
// fully-surrounded interior particle those contributions cancel out from
// symmetry, while a particle at the corner of the block only has neighbors
// on one side and keeps a large net normal. This is the property
// surfaceNormalHat()'s threshold (and thus solveSurfaceTension() being a
// surface-only effect) depends on.

TEST(WCSPHSolverTest, InteriorParticleNormalIsNegligibleComparedToCornerParticle)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setEffectLength(0.1f);
  fluid.setStatic(true);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        fluid.createParticle(Vector3df(-0.05f + 0.05f * i, -0.05f + 0.05f * j, -0.05f + 0.05f * k), 0.001f);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.001f);
  solver.setEffectLength(0.1f);
  solver.simulate(0.001f, 1);

  const int centerIndex = 1 * 9 + 1 * 3 + 1;  // i=j=k=1: fully surrounded
  const int cornerIndex = 0;                  // i=j=k=0: neighbors on one side only

  const float centerNormalSq = getLengthSquared(fluid.getParticles().normals[centerIndex]);
  const float cornerNormalSq = getLengthSquared(fluid.getParticles().normals[cornerIndex]);

  EXPECT_GT(cornerNormalSq, 0.0f);
  EXPECT_LT(centerNormalSq, cornerNormalSq * 0.01f);
}

// ---- Walls contribute density ---------------------------------------------
// A particle resting on a wall only ever sums the fluid above it, so without
// WCSPHSolver::addBoundaryDensity() its density saturates near half the
// bulk value. WCSPHParticle::getPressure()'s max(0, rho - rho0) clamp then
// makes its pressure exactly zero, i.e. fluid against a wall generates no
// repulsion at all and is free to collapse onto itself under load. The wall
// is instead treated as a solid half-space filled with fluid at rest density,
// integrated analytically.

TEST(WCSPHSolverTest, WallContributesDensityToParticlesRestingOnIt)
{
  const float radius = 0.003f;
  const float spacing = radius * 2.0f;
  const float effectLength = radius * 4.0f;   // the documented h = 2 * spacing
  const float restDensity = 1000.0f;
  const float floorY = -0.015f - spacing * 0.5f;

  WCSPHFluid fluid;
  fluid.setDensity(restDensity);
  fluid.setPressureCoeFromScale();
  fluid.setVicosityCoe(0.006f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);

  // A slab wide enough that the particle sampled below is laterally interior,
  // so the only truncation of its neighbourhood is the floor underneath it.
  for (int i = -6; i <= 6; ++i)
    for (int k = -6; k <= 6; ++k)
      for (int j = 0; j < 4; ++j)
        fluid.createParticle(
            Vector3df(i * spacing, -0.015f + j * spacing, k * spacing), radius);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));   // density only, no settling
  solver.setTimeStep(0.0003f);
  solver.setEffectLength(effectLength);
  solver.setBoundary(
      Box3df(Vector3df(-0.5f, floorY, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.0003f);

  solver.simulate(0.0003f, 1);

  // Bottom-layer particle at the centre of the slab (i == k == 0, j == 0).
  const auto& soa = fluid.getParticles();
  size_t bottomCentre = 0;
  float best = std::numeric_limits<float>::max();
  for (size_t i = 0; i < soa.size(); ++i)
  {
    const auto& p = soa.positions[i];
    const float score = std::fabs(p.x) + std::fabs(p.z) + (p.y + 0.015f) * 100.0f;
    if (score < best) { best = score; bottomCentre = i; }
  }

  WCSPHParticle p(fluid.getParticles(), bottomCentre, &fluid);
  // The floor sits half a spacing below, where the analytic half-space
  // integral contributes ~0.35 * restDensity; the fluid above supplies the
  // rest, so the particle lands close to rest density instead of the ~620
  // (0.62 * rest) a 4-layer slab's bottom row reaches on its own.
  EXPECT_GT(p.getDensity(), 0.85f * restDensity);
  EXPECT_LT(p.getDensity(), 1.20f * restDensity);
}

TEST(WCSPHSolverTest, WallDensityLeavesParticlesFurtherThanSupportUntouched)
{
  const float radius = 0.003f;
  const float effectLength = radius * 4.0f;
  const float restDensity = 1000.0f;

  // Two identical single-particle fluids, one far from every wall and one
  // whose only difference is a floor placed just beyond the kernel support.
  auto densityWithFloorAt = [&](const float floorY) {
    WCSPHFluid fluid;
    fluid.setDensity(restDensity);
    fluid.setPressureCoeFromScale();
    fluid.setVicosityCoe(0.0f);
    fluid.setTensionCoe(0.0f);
    fluid.setEffectLength(effectLength);
    fluid.setStatic(false);
    fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius);

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
    solver.setTimeStep(0.0003f);
    solver.setEffectLength(effectLength);
    solver.setBoundary(
        Box3df(Vector3df(-0.5f, floorY, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.0003f);
    solver.simulate(0.0003f, 1);

    WCSPHParticle p(fluid.getParticles(), 0, &fluid);
    return p.getDensity();
  };

  // Exactly at the support radius the contribution is 0 by construction
  // (I(h) == 0), so this must match the self-density-only case.
  const float justOutside = densityWithFloorAt(-effectLength);
  const float farAway = densityWithFloorAt(-0.4f);
  EXPECT_NEAR(justOutside, farAway, kTol * farAway);

  // Half a support in, it must be strictly denser.
  const float halfIn = densityWithFloorAt(-effectLength * 0.5f);
  EXPECT_GT(halfIn, farAway);
}

// ---- SphereBoundary integration (docs/todo/PLAN_sph_showcase_water_sphere.md section 8-B) ----

TEST(WCSPHSolverTest, ParticlesSettlingInsideSphereContainerStayWithinRadius)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(200.0f);
  fluid.setVicosityCoe(0.05f);
  fluid.setEffectLength(0.1f);
  fluid.setStatic(false);

  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      for (int k = 0; k < 3; ++k)
        fluid.createParticle(Vector3df(-0.05f + 0.05f * i, 0.3f + 0.05f * j, -0.05f + 0.05f * k), 0.001f);

  const Vector3df sphereCenter(0.0f, 0.0f, 0.0f);
  const float sphereRadius = 0.45f;

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.005f);
  solver.setEffectLength(0.1f);
  // Domain box is deliberately larger than the sphere so it never engages --
  // the sphere alone must hold the pool (mirrors the water-sphere showcase,
  // where the box only exists because build_simulation() requires one).
  solver.setBoundary(Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f)), 0.01f);
  solver.setBoundarySpheres({ SphereBoundary(sphereCenter, sphereRadius) }, 0.01f);

  for (int step = 0; step < 200; ++step)
  {
    solver.simulate(0.005f, 1);
    for (size_t i = 0; i < fluid.getParticles().size(); ++i)
    {
      ASSERT_TRUE(isFinite3(fluid.getParticles().positions[i])) << "step " << step;
    }
  }

  // Generous tolerance: the penalty force only decelerates a penetrating
  // particle, it doesn't hard-clamp it, so a fast-moving particle can
  // overshoot the surface by a bit before being pushed back the next step.
  const float tolerance = 0.05f;
  for (const auto& pos : fluid.getParticles().positions)
  {
    EXPECT_LE(getDistance(pos, sphereCenter), sphereRadius + tolerance);
  }
}

TEST(WCSPHSolverTest, SphereWallDensityMatchesPlaneWallDensityAtSameDistance)
{
  const float radius = 0.003f;
  const float effectLength = radius * 4.0f;
  const float restDensity = 1000.0f;
  const float d = effectLength * 0.5f;   // distance from the isolated particle to the wall

  auto makeFluid = [&](WCSPHFluid& fluid) {
    fluid.setDensity(restDensity);
    fluid.setPressureCoeFromScale();
    fluid.setVicosityCoe(0.0f);
    fluid.setTensionCoe(0.0f);
    fluid.setEffectLength(effectLength);
    fluid.setStatic(false);
    fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius);
  };

  // Plane wall: floor at y = -d, so the particle at the origin sits distance d above it.
  WCSPHFluid planeFluid;
  makeFluid(planeFluid);
  WCSPHSolver planeSolver;
  planeSolver.add(&planeFluid);
  planeSolver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  planeSolver.setTimeStep(0.0003f);
  planeSolver.setEffectLength(effectLength);
  planeSolver.setBoundary(Box3df(Vector3df(-0.5f, -d, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.0003f);
  planeSolver.simulate(0.0003f, 1);
  const float planeDensity = WCSPHParticle(planeFluid.getParticles(), 0, &planeFluid).getDensity();

  // Sphere wall: R = 25 * effectLength (>= the design's R >= 10h floor), centered
  // so the same particle sits exactly distance d inside the surface.
  const float sphereRadius = 25.0f * effectLength;
  WCSPHFluid sphereFluid;
  makeFluid(sphereFluid);
  WCSPHSolver sphereSolver;
  sphereSolver.add(&sphereFluid);
  sphereSolver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  sphereSolver.setTimeStep(0.0003f);
  sphereSolver.setEffectLength(effectLength);
  sphereSolver.setBoundary(Box3df(Vector3df(-2.0f, -2.0f, -2.0f), Vector3df(2.0f, 2.0f, 2.0f)), 0.0003f);
  sphereSolver.setBoundarySpheres(
      { SphereBoundary(Vector3df(-(sphereRadius - d), 0.0f, 0.0f), sphereRadius) }, 0.0003f);
  sphereSolver.simulate(0.0003f, 1);
  const float sphereDensity = WCSPHParticle(sphereFluid.getParticles(), 0, &sphereFluid).getDensity();

  EXPECT_NEAR(sphereDensity, planeDensity, 0.05f * planeDensity);
}

TEST(WCSPHSolverTest, NoRegisteredSpheresLeavesPlaneOnlyBehaviorUnchanged)
{
  const float radius = 0.003f;
  const float effectLength = radius * 4.0f;
  const float restDensity = 1000.0f;
  const float floorY = -effectLength * 0.5f;

  auto densityWithBoundary = [&](bool registerEmptySphereList) {
    WCSPHFluid fluid;
    fluid.setDensity(restDensity);
    fluid.setPressureCoeFromScale();
    fluid.setVicosityCoe(0.0f);
    fluid.setTensionCoe(0.0f);
    fluid.setEffectLength(effectLength);
    fluid.setStatic(false);
    fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius);

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
    solver.setTimeStep(0.0003f);
    solver.setEffectLength(effectLength);
    solver.setBoundary(Box3df(Vector3df(-0.5f, floorY, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.0003f);
    if (registerEmptySphereList)
    {
      solver.setBoundarySpheres({}, 0.0003f);
    }
    solver.simulate(0.0003f, 1);
    return WCSPHParticle(fluid.getParticles(), 0, &fluid).getDensity();
  };

  const float neverCalled = densityWithBoundary(false);
  const float explicitlyEmpty = densityWithBoundary(true);
  EXPECT_NEAR(neverCalled, explicitlyEmpty, kTol * neverCalled);
}

TEST(WCSPHSolverTest, PlaneAndSphereCombinedDensityIsCappedAtRestDensity)
{
  const float radius = 0.003f;
  const float effectLength = radius * 4.0f;
  const float restDensity = 1000.0f;

  WCSPHFluid fluid;
  fluid.setDensity(restDensity);
  fluid.setPressureCoeFromScale();
  fluid.setVicosityCoe(0.0f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius);

  // Two planes and a sphere, each with the particle sitting exactly on its
  // surface (signed distance 0, the poly6HalfSpaceFraction maximum of 0.5 *
  // restDensity per wall). Summed naively that is 1.5 * restDensity; capped,
  // it must not exceed restDensity.
  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.0003f);
  solver.setEffectLength(effectLength);
  solver.setBoundaryPlanes(
      { PlaneBoundary(Vector3df(0.0f, 1.0f, 0.0f), 0.0f), PlaneBoundary(Vector3df(1.0f, 0.0f, 0.0f), 0.0f) },
      0.0003f);
  const float sphereRadius = 25.0f * effectLength;
  solver.setBoundarySpheres(
      { SphereBoundary(Vector3df(0.0f, 0.0f, -sphereRadius), sphereRadius) }, 0.0003f);
  solver.simulate(0.0003f, 1);

  const float density = WCSPHParticle(fluid.getParticles(), 0, &fluid).getDensity();
  // The particle's own self-density term is on top of the (capped) wall
  // contribution, so this isn't restDensity exactly -- just bounded well
  // below the uncapped 1.5 * restDensity the three walls would otherwise sum to.
  EXPECT_LT(density, 1.3f * restDensity);
}

// WCSPHSolver::addBoundaryDensity() lets a wall stand in for the neighbors a
// particle is missing *because the wall is there*. It must therefore only ever
// fill a deficit: a particle whose fluid neighbors alone already reach rest
// density is not missing anything, and adding the wall's half-space share on
// top counts the same space twice.
//
// That double count is what wrecked the water-sphere showcase: the first layer
// of the jet to reach the dry container floor had a full fluid neighborhood
// (the column ramming into it) *and* the wall's unconditional ~0.5*rho0, so it
// was handed rho ~ 1.5*rho0 and the pressure spike blew it back up at 2.4x its
// own impact speed. See docs/issue/water_sphere_showcase_emitter_instability.md.
namespace {

// One over-packed lattice block (spacing below the rest spacing, so its
// fluid-only density is already above rest density on its own), advanced a
// single step against a floor plane placed `floorGap` below it.
float overPackedBlockPeakDensity(const float floorGap)
{
    const float radius = 0.005f;
    const float effectLength = radius * 4.0f;
    const float spacing = radius * 2.0f * 0.85f;   // over-packed on purpose
    const float timeStep = 2.0e-4f;

    WCSPHFluid fluid;
    fluid.setDensity(1000.0f);
    fluid.setEffectLength(effectLength);
    fluid.setPressureCoeFromScale(20.0f);
    fluid.setVicosityCoe(0.01f);
    fluid.setTensionCoe(0.0f);
    fluid.setStatic(false);

    for (int i = -3; i <= 3; ++i)
        for (int j = 0; j <= 6; ++j)
            for (int k = -3; k <= 3; ++k)
                fluid.createParticle(
                    Vector3df(i * spacing, floorGap + j * spacing, k * spacing), radius);

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
    solver.setTimeStep(timeStep);
    // Floor at y = 0; everything else far enough away to stay out of range.
    solver.setBoundaryPlanes({ PlaneBoundary(Vector3df(0.0f, 1.0f, 0.0f), 0.0f) }, timeStep);
    solver.simulate(timeStep, 1);

    float peak = 0.0f;
    for (const float d : fluid.getParticles().densities) peak = std::max(peak, d);
    return peak;
}

}

TEST(WCSPHSolverTest, WallDensityOnlyFillsTheDeficitOfAnAlreadyDenseParticle)
{
    // Same block, once sitting on the floor and once a full support radius
    // clear of it. The block is over-packed, so its bottom layer is already
    // at/above rest density from fluid neighbors alone -- the wall must add
    // nothing, leaving both cases with the same peak density.
    const float onFloor = overPackedBlockPeakDensity(0.0f);
    const float clearOfFloor = overPackedBlockPeakDensity(0.05f);

    EXPECT_GT(clearOfFloor, 1000.0f) << "block is not over-packed; test proves nothing";
    EXPECT_NEAR(onFloor, clearOfFloor, 1.0f);
}

// ---- Coincident particles must not poison the fluid with NaN -------------
//
// getSpikyKernelGradient() normalizes by |r_ij|, so a pair at exactly the same
// position used to evaluate 0/0 == NaN, and the NaN then spread out of the
// force accumulator into velocity/position and on to everything downstream.
//
// This is reachable in ordinary scenes: the domain-wall penalty restores a
// penetrating particle exactly onto the wall, so particles pressed into a box
// corner -- three planes acting at once -- land on bit-identical coordinates
// and stay there. Measured before the fix on a *resting* pool of 2744
// particles: two of them collapsed onto the same corner point at t = 0.277 s
// and went NaN on the following step.

TEST(SPHKernelTest, SpikyGradientOfACoincidentPairIsZeroNotNaN)
{
  SPHKernel kernel(0.1f);

  const auto g = kernel.getSpikyKernelGradient(Vector3df(0.0f, 0.0f, 0.0f));
  EXPECT_TRUE(isFinite3(g));
  EXPECT_FLOAT_EQ(g.x, 0.0f);
  EXPECT_FLOAT_EQ(g.y, 0.0f);
  EXPECT_FLOAT_EQ(g.z, 0.0f);

  EXPECT_FLOAT_EQ(kernel.getSpikyKernelGradientWeight(0.0f), 0.0f);
}

TEST(SPHKernelTest, SpikyGradientStillFullStrengthJustOffCoincidence)
{
  // The guard may only remove the undefined *direction* at r == 0, not the
  // force: |gradW| = C*(h-r)^2 has a finite limit C*h^2 there, so a pair a
  // hair apart must still get essentially the full magnitude.
  const float h = 0.1f;
  SPHKernel kernel(h);

  const float expected = (45.0f / (3.14159265358979323846f * std::pow(h, 6.0f))) * h * h;
  const auto g = kernel.getSpikyKernelGradient(Vector3df(1.0e-7f, 0.0f, 0.0f));
  const float magnitude = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);

  EXPECT_TRUE(isFinite3(g));
  EXPECT_NEAR(magnitude, expected, expected * 1.0e-3f);
}

TEST(WCSPHSolverTest, CoincidentParticlesDoNotContaminateTheFluidWithNaN)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(0.4f);
  fluid.setVicosityCoe(0.01f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(0.02f);
  fluid.setMaxParticles(256);

  // A small block plus one duplicate sitting exactly on top of an existing
  // particle -- the state a box corner drives particles into.
  const float spacing = 0.01f;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        fluid.createParticle(Vector3df(i * spacing, j * spacing, k * spacing), 0.005f);
      }
    }
  }
  const auto duplicated = fluid.getParticles().positions[13];
  fluid.createParticle(duplicated, 0.005f);
  const auto& added = fluid.getParticles().positions.back();
  // Bit-identical, not merely close: anything else misses the guard entirely.
  ASSERT_FLOAT_EQ(added.x, duplicated.x);
  ASSERT_FLOAT_EQ(added.y, duplicated.y);
  ASSERT_FLOAT_EQ(added.z, duplicated.z);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(0.02f);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));

  for (int step = 0; step < 20; ++step) {
    solver.simulate(2.0e-4f, 1);
  }

  const auto& soa = fluid.getParticles();
  for (size_t i = 0; i < soa.size(); ++i) {
    EXPECT_TRUE(isFinite3(soa.positions[i])) << "position " << i << " went non-finite";
    EXPECT_TRUE(isFinite3(soa.velocities[i])) << "velocity " << i << " went non-finite";
    EXPECT_TRUE(std::isfinite(soa.densities[i])) << "density " << i << " went non-finite";
  }
}


// ---- PlateBoundary integration (docs/todo/PLAN_sph_showcase_waterfall.md section 3) ----
//
// A finite plate is a thin OBB whose *outside* is valid. In the solver it has
// to do everything an infinite wall does for the fluid sitting on it -- supply
// the missing density so the resting layer reaches rest pressure -- while
// letting water past its edge fall freely.

namespace
{
// One isolated particle advanced a single step (no gravity) against the given
// plates plus a far-away domain box; returns its density. Mirrors
// SphereWallDensityMatchesPlaneWallDensityAtSameDistance's setup.
float isolatedParticleDensityAgainstPlates(const Vector3df& particlePos,
                                           const std::vector<PlateBoundary>& plates)
{
  const float radius = 0.003f;
  const float effectLength = radius * 4.0f;

  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoeFromScale();
  fluid.setVicosityCoe(0.0f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);
  fluid.createParticle(particlePos, radius);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.0003f);
  solver.setEffectLength(effectLength);
  solver.setBoundary(Box3df(Vector3df(-2.0f, -2.0f, -2.0f), Vector3df(2.0f, 2.0f, 2.0f)), 0.0003f);
  if (!plates.empty())
  {
    solver.setBoundaryPlates(plates, 0.0003f);
  }
  solver.simulate(0.0003f, 1);
  return WCSPHParticle(fluid.getParticles(), 0, &fluid).getDensity();
}
}

TEST(WCSPHSolverTest, ParticlesRestingOnPlateReachRestDensity)
{
  const float radius = 0.003f;
  const float spacing = radius * 2.0f;
  const float effectLength = radius * 4.0f;
  const float restDensity = 1000.0f;
  const float floorY = -0.015f - spacing * 0.5f;

  WCSPHFluid fluid;
  fluid.setDensity(restDensity);
  fluid.setPressureCoeFromScale();
  fluid.setVicosityCoe(0.006f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);

  for (int i = -6; i <= 6; ++i)
    for (int k = -6; k <= 6; ++k)
      for (int j = 0; j < 4; ++j)
        fluid.createParticle(
            Vector3df(i * spacing, -0.015f + j * spacing, k * spacing), radius);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(0.0003f);
  solver.setEffectLength(effectLength);
  // Domain box far below so only the plate engages -- the finite plate must
  // carry the wall-density job on its own (section 2.3: RigidBoundary can't,
  // which is why the plate exists).
  solver.setBoundary(
      Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.0003f);
  // Top face exactly at floorY, footprint wide enough that the sampled
  // particle is laterally interior (rim taper == 1 there).
  solver.setBoundaryPlates(
      { PlateBoundary(Vector3df(0.0f, floorY - 0.01f, 0.0f), Vector3df(0.0f, 1.0f, 0.0f),
                      Vector3df(1.0f, 0.0f, 0.0f), 0.20f, 0.20f, 0.010f) },
      0.0003f);

  solver.simulate(0.0003f, 1);

  const auto& soa = fluid.getParticles();
  size_t bottomCentre = 0;
  float best = std::numeric_limits<float>::max();
  for (size_t i = 0; i < soa.size(); ++i)
  {
    const auto& p = soa.positions[i];
    const float score = std::fabs(p.x) + std::fabs(p.z) + (p.y + 0.015f) * 100.0f;
    if (score < best) { best = score; bottomCentre = i; }
  }

  WCSPHParticle p(fluid.getParticles(), bottomCentre, &fluid);
  EXPECT_GT(p.getDensity(), 0.85f * restDensity);
  EXPECT_LT(p.getDensity(), 1.20f * restDensity);
}

TEST(WCSPHSolverTest, PoolOnFinitePlateDoesNotLeakThroughIt)
{
  const float radius = 0.004f;
  const float spacing = radius * 2.0f;
  const float effectLength = radius * 4.0f;
  const float topFaceY = 0.0f;

  const PlateBoundary plate(Vector3df(0.0f, topFaceY - 0.012f, 0.0f), Vector3df(0.0f, 1.0f, 0.0f),
                            Vector3df(1.0f, 0.0f, 0.0f), 0.15f, 0.15f, 0.012f);

  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoeFromScale(6.0f);
  fluid.setVicosityCoe(0.004f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);

  for (int i = -3; i <= 3; ++i)
    for (int k = -3; k <= 3; ++k)
      for (int j = 0; j < 4; ++j)
        fluid.createParticle(
            Vector3df(i * spacing, 0.004f + j * spacing, k * spacing), radius);

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(2.5e-4f);
  solver.setEffectLength(effectLength);
  solver.setBoundary(Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f)), 2.5e-4f);
  solver.setBoundaryPlates({ plate }, 2.5e-4f);

  for (int step = 0; step < 300; ++step)
  {
    solver.simulate(2.5e-4f, 1);
  }

  const auto& soa = fluid.getParticles();
  float minY = std::numeric_limits<float>::max();
  float meanY = 0.0f;
  for (size_t i = 0; i < soa.size(); ++i)
  {
    ASSERT_TRUE(isFinite3(soa.positions[i])) << "particle " << i;
    minY = std::min(minY, soa.positions[i].y);
    meanY += soa.positions[i].y;
  }
  meanY /= static_cast<float>(soa.size());

  // No particle sank deeper than the plate's own half-thickness (plus a hair):
  // the slab bounds penetration, there is no tunnelling through it.
  EXPECT_GT(minY, topFaceY - plate.getHalfThickness() - 0.002f);
  // The pool is being held up, not draining through -- most of it is still
  // near the top face rather than far below it.
  EXPECT_GT(meanY, topFaceY - 0.01f);
}

TEST(WCSPHSolverTest, WaterRunsOffThePlateEdge)
{
  const float radius = 0.005f;
  const float effectLength = radius * 4.0f;

  // Top face at y = 0, footprint only +/-50 mm in u.
  const PlateBoundary plate(Vector3df(0.0f, -0.01f, 0.0f), Vector3df(0.0f, 1.0f, 0.0f),
                            Vector3df(1.0f, 0.0f, 0.0f), 0.05f, 0.05f, 0.01f);

  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoeFromScale(6.0f);
  fluid.setVicosityCoe(0.0f);
  fluid.setTensionCoe(0.0f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);
  fluid.createParticle(Vector3df(0.00f, 0.003f, 0.0f), radius);   // 0: over the footprint
  fluid.createParticle(Vector3df(0.30f, 0.003f, 0.0f), radius);   // 1: well past the +u edge

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(5.0e-4f);
  solver.setEffectLength(effectLength);
  solver.setBoundary(Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f)), 5.0e-4f);
  solver.setBoundaryPlates({ plate }, 5.0e-4f);

  const int steps = 120;
  const float T = steps * 5.0e-4f;
  for (int step = 0; step < steps; ++step)
  {
    solver.simulate(5.0e-4f, 1);
  }

  const auto& soa = fluid.getParticles();
  const Vector3df supported = soa.positions[0];
  const Vector3df fell = soa.positions[1];

  // The particle past the edge is in free fall: nothing under it. Semi-implicit
  // Euler gives v_y == -g*T exactly.
  EXPECT_NEAR(soa.velocities[1].y, -9.8f * T, 0.03f * 9.8f * T);
  const float freeFallDrop = 0.5f * 9.8f * T * T;
  EXPECT_NEAR(fell.y, 0.003f - freeFallDrop, 0.08f * freeFallDrop + 1.0e-3f);

  // The particle over the footprint did not fall through (half-thickness
  // bounds it) and is not in free fall (its downward speed never reaches the
  // free-fall value).
  EXPECT_GT(supported.y, -0.011f);
  EXPECT_GT(soa.velocities[0].y, -0.30f);
  EXPECT_GT(supported.y, fell.y + 0.005f);
}

TEST(WCSPHSolverTest, PlateDensityTapersAtTheRim)
{
  const float effectLength = 0.003f * 4.0f;   // h = 0.012
  const float halfThickness = 0.008f;
  const PlateBoundary plate(Vector3df(0.0f, 0.0f, 0.0f), Vector3df(0.0f, 0.0f, 1.0f),
                            Vector3df(1.0f, 0.0f, 0.0f), 0.05f, 0.05f, halfThickness);

  const float z = halfThickness + 0.5f * effectLength;   // face distance == 0.5 h

  const float control = isolatedParticleDensityAgainstPlates(Vector3df(0.0f, 0.0f, z), {});
  const float centre  = isolatedParticleDensityAgainstPlates(Vector3df(0.0f, 0.0f, z), { plate });
  const float rim     = isolatedParticleDensityAgainstPlates(Vector3df(0.05f, 0.0f, z), { plate });

  EXPECT_GT(centre, control);            // the plate contributes density
  EXPECT_GT(rim, control);               // still some contribution right at the edge
  EXPECT_LT(rim, centre);                // ... but tapered relative to the interior
}

TEST(WCSPHSolverTest, TwoOverlappingPlatesDoNotExceedRestDensity)
{
  const float restDensity = 1000.0f;
  const float halfThickness = 0.008f;

  // A floor plate (top face y = 0) and a wall plate (face x = 0) overlapping
  // at the origin, with the particle sitting exactly on both faces -- each
  // half-space integral is at its 0.5*restDensity maximum, so summed naively
  // that is restDensity from the walls alone, on top of the self-density.
  const PlateBoundary floorPlate(Vector3df(0.0f, -halfThickness, 0.0f), Vector3df(0.0f, 1.0f, 0.0f),
                                 Vector3df(1.0f, 0.0f, 0.0f), 0.05f, 0.05f, halfThickness);
  const PlateBoundary wallPlate(Vector3df(-halfThickness, 0.0f, 0.0f), Vector3df(1.0f, 0.0f, 0.0f),
                                Vector3df(0.0f, 0.0f, 1.0f), 0.05f, 0.05f, halfThickness);

  const float density =
      isolatedParticleDensityAgainstPlates(Vector3df(0.0f, 0.0f, 0.0f), { floorPlate, wallPlate });

  EXPECT_LE(density, restDensity * 1.001f);   // the single headroom clamp holds at the seam
  EXPECT_GT(density, restDensity * 0.75f);    // ... and both plates really did contribute
}
