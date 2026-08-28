#include "pch.h"

#include "../Physics/PlaneBoundary.h"
#include "../Physics/PlateBoundary.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/SPHKernel.h"
#include "../Physics/Emitter.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <tuple>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;

float meanY(const std::vector<Vector3df>& positions)
{
  float sum = 0.0f;
  for (const auto& p : positions)
  {
    sum += p.y;
  }
  return positions.empty() ? 0.0f : sum / static_cast<float>(positions.size());
}

bool isFinite3(const Vector3df& v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Builds an N x N x N grid of positions, centered on the X/Z axes and
// starting at originY, spaced `spacing` apart. Used by the determinism
// tests below to exercise the OpenMP-parallel density/pressure passes with
// enough particles (and enough shared-neighbor pairs) that a data race
// would show up as run-to-run divergence if one were reintroduced (see
// WCSPHSolver.cpp / PBSPHSolver.cpp "gather" comments, §2.7 of
// docs/todo/PLAN_sph_addon_showcase_simulations.md).
std::vector<Vector3df> makeParticleGrid(const int n, const float spacing, const float originY)
{
  std::vector<Vector3df> positions;
  positions.reserve(static_cast<size_t>(n) * n * n);
  for (int i = 0; i < n; ++i)
  {
    for (int j = 0; j < n; ++j)
    {
      for (int k = 0; k < n; ++k)
      {
        positions.emplace_back(
            (i - n / 2) * spacing,
            originY + j * spacing,
            (k - n / 2) * spacing);
      }
    }
  }
  return positions;
}
}

TEST(FluidKernelTest, Poly6AndSpikyStayFinite)
{
  SPHKernel kernel(0.1f);

  const float w = kernel.getPoly6Kernel(0.02f);
  const auto g = kernel.getSpikyKernelGradient(Vector3df(0.01f, -0.02f, 0.005f));

  EXPECT_TRUE(std::isfinite(w));
  EXPECT_TRUE(isFinite3(g));
  EXPECT_NEAR(kernel.getEffectLength(), 0.1f, kTol);
}

TEST(FluidBoundaryTest, BoxPlaneBoundariesPushOutsidePointInside)
{
  const auto planes = makeBoxPlaneBoundaries(Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f)));

  Vector3df inside(0.0f, 0.0f, 0.0f);
  Vector3df outsideX(0.0f, 0.0f, 0.0f);
  for (const auto& plane : planes) {
    inside += plane.getBoundaryForce(Vector3df(0.0f, 0.0f, 0.0f), 0.01f);
    outsideX += plane.getBoundaryForce(Vector3df(1.2f, 0.0f, 0.0f), 0.01f);
  }

  EXPECT_NEAR(inside.x, 0.0f, kTol);
  EXPECT_NEAR(inside.y, 0.0f, kTol);
  EXPECT_NEAR(inside.z, 0.0f, kTol);
  EXPECT_LT(outsideX.x, 0.0f);
}

TEST(FluidSolverBaseline, WCSPHGravitySingleStep)
{
  WCSPHFluid fluid;
  fluid.setDensity(1000.0f);
  fluid.setPressureCoe(200.0f);
  fluid.setVicosityCoe(0.05f);
  fluid.setEffectLength(0.1f);
  fluid.setStatic(false);
  fluid.createParticle(Vector3df(0.0f, 0.35f, 0.0f), 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.35f, 0.0f), 0.001f);

  std::vector<Vector3df> initial;
  for (const auto& pos : fluid.getParticles().positions)
  {
    initial.push_back(pos);
  }

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setEffectLength(0.1f);
  solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.01f);
  solver.simulate(0.01f, 1);

  std::vector<Vector3df> finalPos;
  for (size_t i = 0; i < fluid.getParticles().size(); ++i)
  {
    finalPos.push_back(fluid.getParticles().positions[i]);
    EXPECT_TRUE(isFinite3(fluid.getParticles().positions[i]));
    EXPECT_TRUE(isFinite3(fluid.getParticles().velocities[i]));
  }

  EXPECT_EQ(initial.size(), finalPos.size());
  EXPECT_LT(meanY(finalPos), meanY(initial));
}

TEST(FluidSolverBaseline, DFSPHRestDensityAndParticleLifecycle)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.pressureCoe = 0.0f;
  fluid.viscosityCoe = 0.01f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.0f, 0.35f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.35f, 0.0f), 0.025f, 0.001f);

  EXPECT_EQ(fluid.getParticles().size(), 2U);

  for (size_t i = 0; i < fluid.getParticles().size(); ++i)
  {
    EXPECT_TRUE(isFinite3(fluid.getParticles().positions[i]));
    EXPECT_TRUE(isFinite3(fluid.getParticles().velocities[i]));
    DFSPHParticle p(fluid.getParticles(), i, &fluid);
    EXPECT_EQ(p.getParent(), &fluid);
  }

  const float rho = DFSPHSolver::calculateRestDensity(0.1f, 0.025f, 0.001f, &fluid);
  EXPECT_GT(rho, 0.0f);
  EXPECT_TRUE(std::isfinite(rho));
}

TEST(FluidSolverBaseline, PBSPHParticleLifecycleAndFlags)
{
  PBSPHFluid fluid;
  fluid.setRestDensity(1000.0f);
  fluid.setStiffness(200.0f);
  fluid.setVicsosity(0.01f);
  fluid.setEffectLength(0.1f);
  fluid.setIsBoundary(false);

  fluid.createParticle(Vector3df(0.0f, 0.35f, 0.0f), 0.025f);
  fluid.createParticle(Vector3df(0.05f, 0.35f, 0.0f), 0.025f);

  EXPECT_EQ(fluid.getParticles().size(), 2U);
  EXPECT_FLOAT_EQ(fluid.getRestDensity(), 1000.0f);
  EXPECT_FLOAT_EQ(fluid.getStiffness(), 200.0f);
  EXPECT_FLOAT_EQ(fluid.getViscosity(), 0.01f);
  EXPECT_FALSE(fluid.isBoundary());

  fluid.setIsBoundary(true);
  EXPECT_TRUE(fluid.isBoundary());

  const auto& soa = fluid.getParticles();
  for (size_t i = 0; i < soa.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa.positions[i]));
    EXPECT_TRUE(isFinite3(soa.velocities[i]));
  }
}

// ---------------------------------------------------------------------------
// G0 (docs/todo/PLAN_sph_addon_showcase_simulations.md §2.7): repeat-run
// determinism. Each solver's OpenMP-parallel density/pressure passes used to
// be parallelized over neighbor *pairs*, letting two pairs sharing a
// particle land on different threads and race on that particle's
// accumulators. Both WCSPHSolver and PBSPHSolver were rewritten to gather
// per-particle instead (see their "gather" comments); DFSPHSolver's design
// was already per-particle. These tests build an identical grid twice and
// assert the two independent runs produce bit-identical output -- the
// regression this guards against is a scheduling-dependent divergence that
// would NOT show up from a single run's finite-ness check alone.
// ---------------------------------------------------------------------------

namespace
{
constexpr int kGridN = 6; // 216 particles: enough pairs/particle to expose a race.
}

TEST(FluidDeterminism, WCSPHRepeatedRunsMatch)
{
  const float radius = 0.025f;
  const float spacing = radius * 2.0f;
  const auto positions = makeParticleGrid(kGridN, spacing, 0.2f);

  auto run = [&]() {
    WCSPHFluid fluid;
    fluid.setDensity(1000.0f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(1.0f);
    fluid.setEffectLength(radius * 2.25f);
    fluid.setStatic(false);
    for (const auto& p : positions)
    {
      fluid.createParticle(p, radius);
    }

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver.setTimeStep(0.001f);
    solver.setEffectLength(radius * 2.25f);
    solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.001f);

    for (int step = 0; step < 5; ++step)
    {
      solver.simulate(0.001f, 1);
    }
    return fluid.getParticles();
  };

  const auto soa1 = run();
  const auto soa2 = run();

  ASSERT_EQ(soa1.size(), positions.size());
  ASSERT_EQ(soa2.size(), soa1.size());
  for (size_t i = 0; i < soa1.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa1.positions[i]));
    EXPECT_FLOAT_EQ(soa1.positions[i].x, soa2.positions[i].x);
    EXPECT_FLOAT_EQ(soa1.positions[i].y, soa2.positions[i].y);
    EXPECT_FLOAT_EQ(soa1.positions[i].z, soa2.positions[i].z);
    EXPECT_FLOAT_EQ(soa1.velocities[i].x, soa2.velocities[i].x);
    EXPECT_FLOAT_EQ(soa1.velocities[i].y, soa2.velocities[i].y);
    EXPECT_FLOAT_EQ(soa1.velocities[i].z, soa2.velocities[i].z);
    EXPECT_FLOAT_EQ(soa1.densities[i], soa2.densities[i]);
  }
}

// Same bit-for-bit reproducibility check with two overlapping finite plates
// (one tilted) under the grid, so PlateBoundary's force loop AND its
// rim-tapered, headroom-capped density loop both run inside the OpenMP-parallel
// passes. A gather written per-pair instead of per-particle would diverge here.
TEST(FluidDeterminism, WCSPHPlateBoundarySceneRepeatedRunsMatch)
{
  const float radius = 0.02f;
  const float spacing = radius * 2.0f;
  const auto positions = makeParticleGrid(kGridN, spacing, 0.0f);

  auto run = [&]() {
    WCSPHFluid fluid;
    fluid.setDensity(1000.0f);
    fluid.setPressureCoe(1.0f);
    fluid.setVicosityCoe(1.0f);
    fluid.setEffectLength(radius * 2.25f);
    fluid.setStatic(false);
    for (const auto& p : positions)
    {
      fluid.createParticle(p, radius);
    }

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver.setTimeStep(0.001f);
    solver.setEffectLength(radius * 2.25f);
    solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.001f);
    const Vector3df tilted = glm::normalize(Vector3df(0.15f, 1.0f, 0.0f));
    solver.setBoundaryPlates(
        { PlateBoundary(Vector3df(0.0f, -0.01f, 0.0f), Vector3df(0.0f, 1.0f, 0.0f),
                        Vector3df(1.0f, 0.0f, 0.0f), 0.30f, 0.30f, 0.02f),
          PlateBoundary(Vector3df(0.0f, 0.01f, 0.0f), tilted,
                        Vector3df(1.0f, 0.0f, 0.0f), 0.30f, 0.30f, 0.02f) },
        0.001f);

    for (int step = 0; step < 5; ++step)
    {
      solver.simulate(0.001f, 1);
    }
    return fluid.getParticles();
  };

  const auto soa1 = run();
  const auto soa2 = run();

  ASSERT_EQ(soa1.size(), positions.size());
  ASSERT_EQ(soa2.size(), soa1.size());
  for (size_t i = 0; i < soa1.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa1.positions[i]));
    EXPECT_FLOAT_EQ(soa1.positions[i].x, soa2.positions[i].x);
    EXPECT_FLOAT_EQ(soa1.positions[i].y, soa2.positions[i].y);
    EXPECT_FLOAT_EQ(soa1.positions[i].z, soa2.positions[i].z);
    EXPECT_FLOAT_EQ(soa1.velocities[i].x, soa2.velocities[i].x);
    EXPECT_FLOAT_EQ(soa1.velocities[i].y, soa2.velocities[i].y);
    EXPECT_FLOAT_EQ(soa1.velocities[i].z, soa2.velocities[i].z);
    EXPECT_FLOAT_EQ(soa1.densities[i], soa2.densities[i]);
  }
}

TEST(FluidDeterminism, DFSPHRepeatedRunsMatch)
{
  const float radius = 0.025f;
  const float spacing = radius * 2.0f;
  const float mass = 1000.0f * (4.0f / 3.0f * 3.14159265f * radius * radius * radius);
  const auto positions = makeParticleGrid(kGridN, spacing, 0.2f);

  auto run = [&]() {
    DFSPHFluid fluid;
    fluid.density = 1000.0f;
    fluid.viscosityCoe = 0.05f;
    fluid.setEffectLength(radius * 2.25f);
    fluid.setStatic(false);
    for (const auto& p : positions)
    {
      fluid.createParticle(p, radius, mass);
    }

    DFSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver.setTimeStep(0.001f);
    solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.001f);

    for (int step = 0; step < 5; ++step)
    {
      solver.simulate(0.001f, 1);
    }
    return fluid.getParticles();
  };

  const auto soa1 = run();
  const auto soa2 = run();

  ASSERT_EQ(soa1.size(), positions.size());
  ASSERT_EQ(soa2.size(), soa1.size());
  for (size_t i = 0; i < soa1.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa1.positions[i]));
    EXPECT_FLOAT_EQ(soa1.positions[i].x, soa2.positions[i].x);
    EXPECT_FLOAT_EQ(soa1.positions[i].y, soa2.positions[i].y);
    EXPECT_FLOAT_EQ(soa1.positions[i].z, soa2.positions[i].z);
    EXPECT_FLOAT_EQ(soa1.velocities[i].x, soa2.velocities[i].x);
    EXPECT_FLOAT_EQ(soa1.velocities[i].y, soa2.velocities[i].y);
    EXPECT_FLOAT_EQ(soa1.velocities[i].z, soa2.velocities[i].z);
    EXPECT_FLOAT_EQ(soa1.densities[i], soa2.densities[i]);
  }
}

TEST(FluidDeterminism, PBSPHRepeatedRunsMatch)
{
  const float radius = 0.025f;
  const float spacing = radius * 2.0f;
  const auto positions = makeParticleGrid(kGridN, spacing, 0.2f);

  auto run = [&]() {
    PBSPHFluid fluid;
    fluid.setRestDensity(1000.0f);
    fluid.setStiffness(200.0f);
    fluid.setVicsosity(0.01f);
    fluid.setEffectLength(radius * 2.25f);
    fluid.setIsBoundary(false);
    for (const auto& p : positions)
    {
      fluid.createParticle(p, radius);
    }

    PBSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver.setTimeStep(0.001f);
    solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), 0.001f);

    for (int step = 0; step < 5; ++step)
    {
      solver.simulate(0.001f, 3);
    }

    std::vector<Vector3df> outPositions;
    std::vector<Vector3df> outVelocities;
    std::vector<float> outDensities;
    const auto& soa = fluid.getParticles();
    for (size_t i = 0; i < soa.size(); ++i)
    {
      outPositions.push_back(soa.positions[i]);
      outVelocities.push_back(soa.velocities[i]);
      outDensities.push_back(soa.densities[i]);
    }
    return std::make_tuple(outPositions, outVelocities, outDensities);
  };

  const auto [pos1, vel1, dens1] = run();
  const auto [pos2, vel2, dens2] = run();

  ASSERT_EQ(pos1.size(), positions.size());
  ASSERT_EQ(pos2.size(), pos1.size());
  for (size_t i = 0; i < pos1.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(pos1[i]));
    EXPECT_FLOAT_EQ(pos1[i].x, pos2[i].x);
    EXPECT_FLOAT_EQ(pos1[i].y, pos2[i].y);
    EXPECT_FLOAT_EQ(pos1[i].z, pos2[i].z);
    EXPECT_FLOAT_EQ(vel1[i].x, vel2[i].x);
    EXPECT_FLOAT_EQ(vel1[i].y, vel2[i].y);
    EXPECT_FLOAT_EQ(vel1[i].z, vel2[i].z);
    EXPECT_FLOAT_EQ(dens1[i], dens2[i]);
  }
}

// ---- Emitter determinism -------------------------------------------------
// Every test above seeds its scene by hand, so none of them ever touched the
// one stochastic input these solvers have: Emitter::speedJitter, drawn from
// each *Fluid's own std::mt19937. That RNG used to be seeded from
// std::random_device{}(), which made every emitter-driven scene -- including
// all three SPH showcase bakes -- give a different result on every run, with
// none of the tests above noticing. See Physics/Physics/RandomSeed.h and
// docs/issue/water_sphere_showcase_emitter_instability.md section 11.4.
//
// A non-zero speedJitter is the whole point of these: with jitter at 0 the
// RNG is never consulted, so a repeat-run check would pass even with
// random_device seeding restored. Each test asserts that too, so it cannot
// quietly become vacuous.

namespace
{
Emitter makeJitteredEmitter(const float particleRadius)
{
  Emitter e;
  e.center = Vector3df(0.0f, 0.10f, 0.0f);
  e.radius = 0.03f;
  e.particleRadius = particleRadius;
  e.direction = Vector3df(0.0f, -1.0f, 0.0f);
  e.speed = 2.45f;
  e.speedJitter = 0.30f;
  // One particle per spacing^3 of jet volume per second (see Emitter.h).
  const float spacing = particleRadius * 2.0f;
  e.rate = 3.14159265f * e.radius * e.radius * e.speed / (spacing * spacing * spacing);
  return e;
}

// The emitted speeds, which are the only thing the RNG feeds. Kept separate
// from the solver so this can be checked for every fluid type without having
// to calibrate a stable scene for each of them.
template <typename FluidT>
std::vector<float> emittedSpeeds()
{
  constexpr float radius = 0.005f;
  constexpr float dt = 2.0e-4f;

  FluidT fluid;
  fluid.setEffectLength(radius * 4.0f);
  fluid.setStatic(false);
  fluid.addEmitter(makeJitteredEmitter(radius));

  for (int step = 0; step < 100; ++step)
  {
    fluid.updateEmitters(dt);
  }

  const auto& soa = fluid.getParticles();
  std::vector<float> speeds;
  speeds.reserve(soa.velocities.size());
  for (const auto& v : soa.velocities)
  {
    speeds.push_back(v.y);
  }
  return speeds;
}

template <typename FluidT>
void expectEmitterJitterIsReproducible()
{
  const auto a = emittedSpeeds<FluidT>();
  const auto b = emittedSpeeds<FluidT>();

  ASSERT_GT(a.size(), 100u) << "emitter produced too few particles to be a meaningful check";
  ASSERT_EQ(a.size(), b.size());

  for (size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_FLOAT_EQ(a[i], b[i]) << "particle " << i;
  }

  // Without this the loop above would pass on a constant array, i.e. it would
  // still pass with random_device seeding restored.
  const auto minMax = std::minmax_element(a.begin(), a.end());
  EXPECT_GT(*minMax.second - *minMax.first, 1.0e-3f)
      << "speedJitter had no effect -- this test proves nothing";
}
}

TEST(FluidDeterminism, WCSPHEmitterJitterIsReproducible)
{
  expectEmitterJitterIsReproducible<WCSPHFluid>();
}

TEST(FluidDeterminism, DFSPHEmitterJitterIsReproducible)
{
  expectEmitterJitterIsReproducible<DFSPHFluid>();
}

TEST(FluidDeterminism, PBSPHEmitterJitterIsReproducible)
{
  expectEmitterJitterIsReproducible<PBSPHFluid>();
}

// The same thing end to end: a jittered jet actually stepped through the
// solver has to land in exactly the same place twice. Parameters are the
// water-sphere showcase's (see WaterSphereEmitterStabilityTest), which is the
// scene this whole investigation came from; the run stops while the jet is
// still falling, so it stays cheap and cannot diverge for unrelated reasons.
TEST(FluidDeterminism, WCSPHEmitterDrivenSceneRepeatedRunsMatch)
{
  constexpr float radius = 0.005f;
  constexpr float effectLength = 0.02f;
  constexpr float dt = 2.0e-4f;

  auto run = [&]() {
    WCSPHFluid fluid;
    fluid.setDensity(1000.0f);
    fluid.setEffectLength(effectLength);
    fluid.setPressureCoeFromScale(20.0f);
    fluid.setVicosityCoe(0.010f);
    fluid.setTensionCoe(0.0f);
    fluid.setStatic(false);
    fluid.addEmitter(makeJitteredEmitter(radius));

    WCSPHSolver solver;
    solver.add(&fluid);
    solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
    solver.setTimeStep(dt);
    solver.setBoundary(Box3df(Vector3df(-0.5f, -0.5f, -0.5f), Vector3df(0.5f, 0.5f, 0.5f)), dt);

    for (int step = 0; step < 100; ++step)
    {
      fluid.updateEmitters(dt);
      solver.simulate(dt, 1);
    }
    return fluid.getParticles();
  };

  const auto soa1 = run();
  const auto soa2 = run();

  ASSERT_GT(soa1.size(), 100u);
  ASSERT_EQ(soa1.size(), soa2.size());
  for (size_t i = 0; i < soa1.size(); ++i)
  {
    EXPECT_TRUE(isFinite3(soa1.positions[i])) << "particle " << i;
    EXPECT_FLOAT_EQ(soa1.positions[i].x, soa2.positions[i].x) << "particle " << i;
    EXPECT_FLOAT_EQ(soa1.positions[i].y, soa2.positions[i].y) << "particle " << i;
    EXPECT_FLOAT_EQ(soa1.positions[i].z, soa2.positions[i].z) << "particle " << i;
    EXPECT_FLOAT_EQ(soa1.velocities[i].y, soa2.velocities[i].y) << "particle " << i;
    EXPECT_FLOAT_EQ(soa1.densities[i], soa2.densities[i]) << "particle " << i;
  }
}

// setRandomSeed() is the deliberate escape hatch. Two seeds must give two
// different draws -- otherwise making the default seed fixed would have
// quietly removed the ability to vary a shot at all -- and re-seeding to the
// same value must reproduce a draw exactly.
TEST(FluidDeterminism, EmitterRandomSeedSelectsTheDraw)
{
  constexpr float radius = 0.005f;
  constexpr float dt = 2.0e-4f;

  auto run = [&](const unsigned int seed) {
    WCSPHFluid fluid;
    fluid.setEffectLength(radius * 4.0f);
    fluid.setStatic(false);
    fluid.setRandomSeed(seed);
    fluid.addEmitter(makeJitteredEmitter(radius));

    for (int step = 0; step < 100; ++step)
    {
      fluid.updateEmitters(dt);
    }
    const auto& soa = fluid.getParticles();
    std::vector<float> speeds;
    for (const auto& v : soa.velocities)
    {
      speeds.push_back(v.y);
    }
    return speeds;
  };

  const auto a = run(1u);
  const auto b = run(2u);
  const auto aAgain = run(1u);

  ASSERT_GT(a.size(), 100u);
  ASSERT_EQ(a.size(), b.size());
  ASSERT_EQ(a.size(), aAgain.size());

  for (size_t i = 0; i < a.size(); ++i)
  {
    EXPECT_FLOAT_EQ(a[i], aAgain[i]) << "same seed must reproduce the draw (particle " << i << ")";
  }

  bool differs = false;
  for (size_t i = 0; i < a.size() && !differs; ++i)
  {
    differs = (a[i] != b[i]);
  }
  EXPECT_TRUE(differs) << "different seeds produced an identical draw -- setRandomSeed() is not wired up";
}
