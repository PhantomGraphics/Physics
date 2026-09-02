#include "pch.h"

#include "../Physics/DFSPHFluid.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/DFSPHSolver.h"
#include "../Physics/WCSPHFluid.h"

#include <algorithm>
#include <cmath>
#include <limits>
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
}

// ---- DFSPHFluid public-field access -------------------------------------

TEST(DFSPHFluidTest, PublicFieldsSetAndRead)
{
  DFSPHFluid fluid;
  fluid.density      = 800.0f;
  fluid.viscosityCoe = 0.05f;
  fluid.pressureCoe  = 150.0f;
  fluid.effectLength = 0.2f;

  EXPECT_FLOAT_EQ(fluid.density,      800.0f);
  EXPECT_FLOAT_EQ(fluid.viscosityCoe, 0.05f);
  EXPECT_FLOAT_EQ(fluid.pressureCoe,  150.0f);
  EXPECT_FLOAT_EQ(fluid.effectLength,  0.2f);
  EXPECT_FLOAT_EQ(fluid.getDensity(), 800.0f);
  EXPECT_FLOAT_EQ(fluid.getViscosityCoe(), 0.05f);
}

// ---- Surface tension coefficient (docs/todo/PLAN_sph_surface_tension.md
// Phase 5) -- defaults to 0.f (disabled) rather than being left indeterminate
// like the other public fields above, since unlike those there is no
// existing caller that always sets it (see DFSPHFluid.h's field comment).

TEST(DFSPHFluidTest, TensionCoeDefaultsToZeroAndIsSettable)
{
  DFSPHFluid fluid;
  EXPECT_FLOAT_EQ(fluid.tensionCoe, 0.0f);
  EXPECT_FLOAT_EQ(fluid.getTensionCoe(), 0.0f);

  fluid.tensionCoe = 0.07f;
  EXPECT_FLOAT_EQ(fluid.tensionCoe, 0.07f);
  EXPECT_FLOAT_EQ(fluid.getTensionCoe(), 0.07f);
}

// ---- Particle addition --------------------------------------------------

TEST(DFSPHFluidTest, AddParticleIncreasesCount)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  EXPECT_EQ(fluid.getParticles().size(), 0U);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f, 0.001f);
  EXPECT_EQ(fluid.getParticles().size(), 1U);

  fluid.createParticle(Vector3df(0.1f, 0.0f, 0.0f), 0.025f, 0.001f);
  EXPECT_EQ(fluid.getParticles().size(), 2U);
}

// ---- Bounding box -------------------------------------------------------

TEST(DFSPHFluidTest, BoundingBoxContainsAllParticles)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);
  fluid.createParticle(Vector3df(-0.5f,  0.3f,  0.1f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df( 0.5f, -0.3f, -0.1f), 0.025f, 0.001f);

  const auto bb = fluid.getBoundingBox();
  EXPECT_LE(bb.getMin().x, -0.5f + kTol);
  EXPECT_LE(bb.getMin().y, -0.3f + kTol);
  EXPECT_GE(bb.getMax().x,  0.5f - kTol);
  EXPECT_GE(bb.getMax().y,  0.3f - kTol);
}

// ---- Emitter (docs/todo/PLAN_physics_fluid_emitter.md) -------------------

TEST(DFSPHFluidTest, UpdateEmittersIsNoOpWithoutRegisteredEmitters)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;

  fluid.updateEmitters(0.1f);

  EXPECT_EQ(fluid.getParticles().size(), 0U);
}

TEST(DFSPHFluidTest, UpdateEmittersEmitsAtTheConfiguredRate)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;

  Emitter e;
  e.rate = 100.0f;
  fluid.addEmitter(e);

  for (int step = 0; step < 10; ++step)
  {
    fluid.updateEmitters(0.01f);
  }

  EXPECT_EQ(fluid.getParticles().size(), 10U);
}

TEST(DFSPHFluidTest, UpdateEmittersStopsAtMaxParticles)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setMaxParticles(5);

  Emitter e;
  e.rate = 10000.0f;
  fluid.addEmitter(e);

  fluid.updateEmitters(1.0f);

  EXPECT_EQ(fluid.getParticles().size(), 5U);
}

TEST(DFSPHFluidTest, UpdateEmittersGivesSpawnedParticlesVelocityAlongDirection)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;

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

TEST(DFSPHFluidTest, ClearEmittersRemovesRegisteredEmitters)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;

  Emitter e;
  fluid.addEmitter(e);
  EXPECT_EQ(fluid.getEmitters().size(), 1U);

  fluid.clearEmitters();
  EXPECT_TRUE(fluid.getEmitters().empty());

  fluid.updateEmitters(1.0f);
  EXPECT_EQ(fluid.getParticles().size(), 0U);
}

// ---- Outflow region (optional particle removal) --------------------------

TEST(DFSPHFluidTest, RemoveOutflowParticlesIsNoOpWithoutRegisteredRegions)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f, 0.001f);

  fluid.removeOutflowParticles();

  EXPECT_EQ(fluid.getParticles().size(), 1U);
}

TEST(DFSPHFluidTest, RemoveOutflowParticlesDeletesParticlesInsideRegionOnly)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.createParticle(Vector3df(0.0f, -10.0f, 0.0f), 0.025f, 0.001f); // inside
  fluid.createParticle(Vector3df(0.0f, 10.0f, 0.0f), 0.025f, 0.001f);  // outside

  OutflowRegion r;
  r.bounds = Box3df(Vector3df(-1.0f, -11.0f, -1.0f), Vector3df(1.0f, -9.0f, 1.0f));
  fluid.addOutflowRegion(r);

  fluid.removeOutflowParticles();

  ASSERT_EQ(fluid.getParticles().size(), 1U);
  EXPECT_NEAR(fluid.getParticles().positions[0].y, 10.0f, kTol);
}

TEST(DFSPHFluidTest, ClearOutflowRegionsRemovesRegisteredRegions)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f, 0.001f);

  OutflowRegion r;
  r.bounds = Box3df(Vector3df(-1.0f, -1.0f, -1.0f), Vector3df(1.0f, 1.0f, 1.0f));
  fluid.addOutflowRegion(r);
  EXPECT_EQ(fluid.getOutflowRegions().size(), 1U);

  fluid.clearOutflowRegions();
  EXPECT_TRUE(fluid.getOutflowRegions().empty());

  fluid.removeOutflowParticles();
  EXPECT_EQ(fluid.getParticles().size(), 1U);
}

// ---- Regression: emitter particleRadius must match the scene's particle
// radius, or the solver diverges -----------------------------------------
// Reproduces a real reported divergence: FluidWorld::createDFSPH() calibrates
// density/pressureCoe assuming every particle uses params_.radius (WCSPH/
// DFSPH/PBSPH all derive a particle's SPH mass from its radius, see
// Emitter::particleRadius's doc comment). Emitter::particleRadius used to
// default to 0.05 regardless of the scene's actual radius (1.0 by default,
// same as this test uses) -- a ~1:8000 mass ratio between emitted and
// existing particles -- and the solver blew up within about a second of
// emission. FluidWorld::addEmitter() now forces particleRadius to match
// params_.radius before registering; this test locks in that the *fluid*
// stays stable when an emitter's particleRadius does match the scene's
// radius, mirroring exactly what FluidWorld::addEmitter() now guarantees.

TEST(DFSPHSolverTest, EmittedParticlesAtSceneRadiusStayFiniteWhileFallingIntoExistingFluid)
{
  const float radius = 1.0f;
  const float effectLength = 2.25f;
  const float diameter = radius * 2.0f;
  const float mass = diameter * diameter * diameter;

  DFSPHFluid fluid;
  fluid.setEffectLength(effectLength);
  fluid.viscosityCoe = 5.0f;
  fluid.pressureCoe = WCSPHFluid::estimatePressureCoe(effectLength, 1960.0f);
  fluid.density = DFSPHSolver::calculateRestDensity(effectLength, radius, mass, &fluid);

  // One pre-existing "seed" particle, matching how FluidWorld builds its
  // initial block from the same radius/mass.
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), radius, mass);

  Emitter e;
  e.center = Vector3df(0.0f, 10.0f, 0.0f);
  e.radius = 1.0f;
  e.rate = 20.0f;
  e.direction = Vector3df(0.0f, -1.0f, 0.0f);
  e.speed = 3.0f;
  e.particleRadius = radius; // what FluidWorld::addEmitter() now forces.
  fluid.addEmitter(e);

  DFSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.01f);
  solver.setBoundary(Box3df(Vector3df(-8.0f, -8.0f, -8.0f), Vector3df(8.0f, 14.0f, 8.0f)), 0.01f);

  for (int step = 0; step < 100; ++step)
  {
    fluid.updateEmitters(0.01f);
    solver.simulate(0.01f, 3);

    for (const auto& pos : fluid.getParticles().positions)
    {
      ASSERT_TRUE(std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z)) << "step " << step;
    }
    for (const auto& vel : fluid.getParticles().velocities)
    {
      ASSERT_TRUE(isFinite3(vel)) << "step " << step;
    }
  }

  // 1 seed + ~20 emitted particles/sec * 1.0s (100 steps * dt=0.01); not
  // pinned to exactly 21 since rate*dt=0.2 isn't exactly representable in
  // float, so the accumulator can drift a step either way over 100 additions
  // (unlike WCSPHFluidTest.UpdateEmittersEmitsAtTheConfiguredRate's
  // rate*dt=1.0 case, which needs no repeated fractional accumulation).
  EXPECT_GE(fluid.getParticles().size(), 19U);
  EXPECT_LE(fluid.getParticles().size(), 22U);
}

// ---- Kernel effect length -----------------------------------------------

TEST(DFSPHFluidTest, EffectLengthUpdatesKernel)
{
  DFSPHFluid fluid;
  fluid.setEffectLength(0.25f);
  EXPECT_NEAR(fluid.getKernel()->getEffectLength(), 0.25f, kTol);

  fluid.setEffectLength(0.5f);
  EXPECT_NEAR(fluid.getKernel()->getEffectLength(), 0.5f, kTol);
}

// ---- DFSPHParticle construction and neighbor-driven calculations --------
//
// DFSPHParticle is a non-owning (soa, index) view (see its class doc); it no
// longer stores a neighbor list on itself the way the pre-SoA design did
// (addNeighbor()/getNeighbors()/clearNeighbors() are gone). Callers now pass
// the neighbor index list explicitly to each calculation, so these tests
// build a small working set of views over a fluid's SoA storage and supply
// neighbor indices directly.

TEST(DFSPHParticleTest, ConstructionInitializesFiniteState)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.1f, 0.2f, 0.3f), 0.025f, 0.001f);
  DFSPHParticle p(fluid.getParticles(), 0, &fluid);

  EXPECT_TRUE(isFinite3(p.getPosition()));
  EXPECT_TRUE(isFinite3(p.getVelocity()));
  EXPECT_NEAR(p.getPosition().x, 0.1f, kTol);
  EXPECT_NEAR(p.getPosition().y, 0.2f, kTol);
  EXPECT_NEAR(p.getPosition().z, 0.3f, kTol);
  EXPECT_FLOAT_EQ(p.getRadius(), 0.025f);
  EXPECT_EQ(p.getParent(), &fluid);
}

TEST(DFSPHParticleTest, ExternalNeighborIndicesDriveDensityCalculation)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.0f,  0.0f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.0f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.0f,  0.05f, 0.0f), 0.025f, 0.001f);

  std::vector<DFSPHParticle> particles{
    DFSPHParticle(fluid.getParticles(), 0, &fluid),
    DFSPHParticle(fluid.getParticles(), 1, &fluid),
    DFSPHParticle(fluid.getParticles(), 2, &fluid),
  };

  const std::vector<int> noNeighbors{};
  particles[0].calculateDensity(particles, noNeighbors);
  const float selfOnlyDensity = particles[0].getDensity();
  EXPECT_GT(selfOnlyDensity, 0.0f);

  const std::vector<int> twoNeighbors{ 1, 2 };
  particles[0].calculateDensity(particles, twoNeighbors);
  EXPECT_GT(particles[0].getDensity(), selfOnlyDensity);
}

TEST(DFSPHParticleTest, CalculateDpDtIsZeroForUniformVelocityField)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.0f, 0.0f), 0.025f, 0.001f);

  std::vector<DFSPHParticle> particles{
    DFSPHParticle(fluid.getParticles(), 0, &fluid),
    DFSPHParticle(fluid.getParticles(), 1, &fluid),
  };
  particles[0].setVelocity(Vector3df(1.0f, -2.0f, 0.5f));
  particles[1].setVelocity(particles[0].getVelocity());

  const std::vector<int> neighbors{ 1 };
  particles[0].calculateDensity(particles, neighbors);
  particles[0].calculateDpDt(particles, neighbors);

  EXPECT_NEAR(particles[0].getDpDt(), 0.0f, kTol);
}

TEST(DFSPHParticleTest, DivergenceCorrectionKeepsUniformVelocity)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.0f, 0.0f), 0.025f, 0.001f);

  std::vector<DFSPHParticle> particles{
    DFSPHParticle(fluid.getParticles(), 0, &fluid),
    DFSPHParticle(fluid.getParticles(), 1, &fluid),
  };
  particles[0].setVelocity(Vector3df(0.2f, -0.1f, 0.0f));
  particles[1].setVelocity(particles[0].getVelocity());

  const std::vector<int> n0{ 1 };
  const std::vector<int> n1{ 0 };

  particles[0].calculateDensity(particles, n0);
  particles[1].calculateDensity(particles, n1);
  particles[0].calculateAlpha(particles, n0);
  particles[1].calculateAlpha(particles, n1);
  particles[0].calculateDpDt(particles, n0);
  particles[1].calculateDpDt(particles, n1);

  const auto v0Before = particles[0].getVelocity();
  const auto v1Before = particles[1].getVelocity();

  particles[0].calculateVelocityInDivergenceError(0.001f, particles, n0);
  particles[1].calculateVelocityInDivergenceError(0.001f, particles, n1);

  EXPECT_NEAR(particles[0].getVelocity().x, v0Before.x, kTol);
  EXPECT_NEAR(particles[0].getVelocity().y, v0Before.y, kTol);
  EXPECT_NEAR(particles[0].getVelocity().z, v0Before.z, kTol);
  EXPECT_NEAR(particles[1].getVelocity().x, v1Before.x, kTol);
  EXPECT_NEAR(particles[1].getVelocity().y, v1Before.y, kTol);
  EXPECT_NEAR(particles[1].getVelocity().z, v1Before.z, kTol);
}

// ---- Surface normal / surface tension (docs/todo/PLAN_sph_surface_tension.md
// Phase 5) -- mirrors WCSPHParticle::solveNormal()/solveSurfaceTension() and
// the WCSPHSolverTest coverage for them, adapted to DFSPHParticle's explicit
// neighbor-list-argument style (see this file's DFSPHParticleTest comment
// above).

TEST(DFSPHParticleTest, CalculateNormalIsZeroWithNoNeighbors)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.025f, 0.001f);
  std::vector<DFSPHParticle> particles{ DFSPHParticle(fluid.getParticles(), 0, &fluid) };

  const std::vector<int> noNeighbors{};
  particles[0].calculateDensity(particles, noNeighbors);
  particles[0].calculateNormal(particles, noNeighbors);

  EXPECT_NEAR(getLengthSquared(particles[0].getNormal()), 0.0f, kTol);
}

TEST(DFSPHParticleTest, CalculateNormalIsNonZeroForAsymmetricNeighbor)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  fluid.createParticle(Vector3df(0.0f,  0.0f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.0f, 0.0f), 0.025f, 0.001f);

  std::vector<DFSPHParticle> particles{
    DFSPHParticle(fluid.getParticles(), 0, &fluid),
    DFSPHParticle(fluid.getParticles(), 1, &fluid),
  };

  const std::vector<int> n0{ 1 };
  const std::vector<int> n1{ 0 };
  particles[0].calculateDensity(particles, n0);
  particles[1].calculateDensity(particles, n1);
  particles[0].calculateNormal(particles, n0);

  EXPECT_GT(getLengthSquared(particles[0].getNormal()), 0.0f);
}

// Mirrors WCSPHSolverTest.SurfaceTensionPullsTwoNearbyParticlesTogether: two
// particles placed inside the Poly6 Laplacian's attractive band (r >
// sqrt(3/7)*effectLength =~ 0.6547*effectLength) should be pulled toward
// each other when tensionCoe > 0. Exercises calculateNormal()/
// calculateSurfaceTension() directly (rather than through
// DFSPHSolver::simulate()) so the assertion isn't entangled with DFSPH's
// divergence-free/density-error correction passes, which also perturb
// velocity based on neighbor proximity.
TEST(DFSPHParticleTest, SurfaceTensionPullsTwoNearbyParticlesTogether)
{
  const float h = 1.0f;
  const float r = 0.85f * h;

  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.pressureCoe = 0.0f;
  fluid.viscosityCoe = 0.0f;
  fluid.tensionCoe = 1.0f;
  fluid.setEffectLength(h);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.3f, 1.0f);
  fluid.createParticle(Vector3df(r, 0.0f, 0.0f), 0.3f, 1.0f);

  std::vector<DFSPHParticle> particles{
    DFSPHParticle(fluid.getParticles(), 0, &fluid),
    DFSPHParticle(fluid.getParticles(), 1, &fluid),
  };

  const std::vector<int> n0{ 1 };
  const std::vector<int> n1{ 0 };
  particles[0].calculateDensity(particles, n0);
  particles[1].calculateDensity(particles, n1);
  particles[0].calculateNormal(particles, n0);
  particles[1].calculateNormal(particles, n1);
  particles[0].calculateSurfaceTension(particles, n0);
  particles[1].calculateSurfaceTension(particles, n1);

  ASSERT_TRUE(isFinite3(particles[0].getForce()));
  ASSERT_TRUE(isFinite3(particles[1].getForce()));
  // p0 sits at x=0, p1 at x=r>0: attraction means each is pulled toward the
  // other.
  EXPECT_GT(particles[0].getForce().x, 0.0f);
  EXPECT_LT(particles[1].getForce().x, 0.0f);
}

// ---- DFSPHSolver: static rest-density calculation -----------------------

TEST(DFSPHSolverTest, CalculateRestDensityIsPositiveAndFinite)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  const float rho = DFSPHSolver::calculateRestDensity(0.1f, 0.025f, 0.001f, &fluid);
  EXPECT_GT(rho, 0.0f);
  EXPECT_TRUE(std::isfinite(rho));
}

TEST(DFSPHSolverTest, CalculateRestDensityScalesWithMass)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  const float rho1 = DFSPHSolver::calculateRestDensity(0.1f, 0.025f, 0.001f, &fluid);
  const float rho2 = DFSPHSolver::calculateRestDensity(0.1f, 0.025f, 0.002f, &fluid);

  // Doubling the mass should roughly double the density.
  EXPECT_GT(rho2, rho1);
  EXPECT_TRUE(std::isfinite(rho2));
}

// ---- DFSPHParticle: density from neighbor -------------------------------

TEST(DFSPHParticleTest, CalculateDensityWithNeighborIsPositive)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  // Two particles within the search radius (0.025 * 2.25 = 0.05625).
  fluid.createParticle(Vector3df(0.0f,  0.0f, 0.0f), 0.025f, 0.001f);
  fluid.createParticle(Vector3df(0.05f, 0.0f, 0.0f), 0.025f, 0.001f);

  std::vector<DFSPHParticle> particles{
    DFSPHParticle(fluid.getParticles(), 0, &fluid),
    DFSPHParticle(fluid.getParticles(), 1, &fluid),
  };

  const std::vector<int> neighbors{ 1 };
  particles[0].calculateDensity(particles, neighbors);

  EXPECT_GT(particles[0].getDensity(), 0.0f);
  EXPECT_TRUE(std::isfinite(particles[0].getDensity()));
}

// ---- DFSPHSolver: rest density varies with particle radius --------------

TEST(DFSPHSolverTest, CalculateRestDensityVariesWithParticleRadius)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(0.1f);

  // Smaller radius -> tighter lattice spacing -> larger kernel weight at each
  // neighbor -> higher computed rest density.
  const float rhoSmall = DFSPHSolver::calculateRestDensity(0.1f, 0.020f, 0.001f, &fluid);
  const float rhoLarge = DFSPHSolver::calculateRestDensity(0.1f, 0.040f, 0.001f, &fluid);

  EXPECT_GT(rhoSmall, 0.0f);
  EXPECT_GT(rhoLarge, 0.0f);
  EXPECT_TRUE(std::isfinite(rhoSmall));
  EXPECT_TRUE(std::isfinite(rhoLarge));
  EXPECT_GT(rhoSmall, rhoLarge);
}

// ---- DFSPHSolver: neighbor search radius tracks the fluid's effectLength,
// not a hardcoded 2.25x-radius ratio (docs/todo/PLAN_sph_scale_invariance.md
// Phase 4, item #8) ---------------------------------------------------------

TEST(DFSPHSolverTest, CalculateRestDensityUsesFluidsEffectLengthNotHardcodedRatio)
{
  const float particleRadius = 0.025f;
  const float spacing = 2.0f * particleRadius;               // 0.05f
  const float diagonal = std::sqrt(2.0f) * spacing;          // ~0.0707f
  const float mass = 0.001f;

  // calculateRestDensity() samples the centre of a cubic lattice of spacing
  // 2*particleRadius, so the reference particle's only neighbors inside a
  // 3.0x-radius support (0.075f) are the 6 axis ones at `spacing` and the 12
  // face-diagonal ones at `diagonal`; the 8 body diagonals (sqrt(3)*spacing
  // =~ 0.0866f) and everything beyond are already outside it.
  //
  // The face diagonals are the discriminating set: they sit outside the
  // pre-Phase-4 hardcoded search radius of particleRadius*2.25f (0.05625f) but
  // inside this fluid's own effectLength. So the lattice sum *with* them is
  // what a search radius driven by setEffectLength() must produce, and the sum
  // without them is what the old hardcoded radius produced.
  //
  // Comparing against those two explicit sums (rather than against a second
  // fluid with a different effectLength) keeps the test about the search
  // radius alone: the cubic spline is normalized as 1/h^3, so widening h also
  // shrinks every individual weight, and a plain "wider effectLength => higher
  // density" comparison measures that normalization instead -- it in fact goes
  // the other way, since the 12 face diagonals contribute ~0.3% of the total
  // while the 1/h^3 factor drops it by (2.25/3)^3.
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.setEffectLength(3.0f * particleRadius);

  SPHKernel* kernel = fluid.getKernel();
  ASSERT_NE(kernel, nullptr);
  ASSERT_LT(diagonal, kernel->getEffectLength());     // inside this fluid's support
  ASSERT_GT(diagonal, 2.25f * particleRadius);        // outside the old hardcoded one

  const float axisOnly = mass * (kernel->getCubicSpline(0.0f)
                                 + 6.0f * kernel->getCubicSpline(spacing));
  const float withFaceDiagonals =
      axisOnly + mass * 12.0f * kernel->getCubicSpline(diagonal);

  const float rho = DFSPHSolver::calculateRestDensity(0.0f, particleRadius, mass, &fluid);

  ASSERT_TRUE(std::isfinite(rho));
  // Strictly more than the hardcoded-2.25x-radius neighborhood would give ...
  EXPECT_GT(rho, axisOnly);
  // ... and exactly the effectLength-driven neighborhood.
  EXPECT_NEAR(rho, withFaceDiagonals, withFaceDiagonals * 1.0e-4f);
}

// ---- DFSPHSolver: divergence-error convergence check is relative, not
// absolute (docs/todo/PLAN_sph_scale_invariance.md Phase 3) --------------

TEST(DFSPHSolverTest, IsDivergenceErrorAcceptable_ReproducesHistoricalThresholdAtDefaultScale)
{
  // Pre-Phase-3 behavior was the unconditional absolute check
  // `averageDpDt < 2.0f`. At the historical default maxTimeStep=0.01f (dt
  // capped to maxTimeStep/2=0.005f by calculateTimeStep()'s CFL rule) and
  // restDensity=1, the new ratio-based check must land on the same
  // accept/reject boundary so existing default-scale scenarios keep their
  // iteration counts.
  const float dt = 0.005f;
  EXPECT_TRUE(DFSPHSolver::isDivergenceErrorAcceptable(1.9f, 1.0f, dt));
  EXPECT_FALSE(DFSPHSolver::isDivergenceErrorAcceptable(2.1f, 1.0f, dt));
}

TEST(DFSPHSolverTest, IsDivergenceErrorAcceptable_IsInvariantUnderRestDensityScale)
{
  // calculateDpDt() sums neighbor mass * dot(relativeVelocity, gradient), so
  // a physically-consistent rescale of restDensity (e.g. 1 -> 100, achieved
  // by scaling every particle's mass by the same factor for an unchanged
  // velocity field) scales averageDpDt by that same factor. The
  // restDensity-normalized ratio -- and therefore the accept/reject decision
  // that drives correctDivergenceError()'s iteration count -- must stay the
  // same either way; the old unnormalized check would not have.
  const float dt = 0.005f;
  EXPECT_EQ(DFSPHSolver::isDivergenceErrorAcceptable(1.5f, 1.0f, dt),
            DFSPHSolver::isDivergenceErrorAcceptable(150.0f, 100.0f, dt));
  EXPECT_TRUE(DFSPHSolver::isDivergenceErrorAcceptable(1.5f, 1.0f, dt));

  EXPECT_EQ(DFSPHSolver::isDivergenceErrorAcceptable(2.5f, 1.0f, dt),
            DFSPHSolver::isDivergenceErrorAcceptable(250.0f, 100.0f, dt));
  EXPECT_FALSE(DFSPHSolver::isDivergenceErrorAcceptable(2.5f, 1.0f, dt));
}

// ---- The wall penalty must be calibrated against the substep -------------
//
// PlaneBoundary's penalty spring is a = -d/T^2, which undoes the penetration
// d in exactly one step of length T (dv = -d/T, dx = -d), so the deepest a
// particle gets is d_max = v_impact * T. DFSPH does not integrate at the
// caller's time step -- it substeps adaptively (calculateTimeStep(), clamped
// to [maxTimeStep*1e-4, maxTimeStep/2]) -- but addBoundaryPressure() used to
// feed the spring the frame-level timeStep stored by setBoundary() instead.
// T > dt makes the per-step correction collapse to -d*(dt/T)^2, and since the
// CFL substep shrinks as 1/v, the faster a particle hit a wall the softer
// that wall became.
//
// Now that the spring gets the substep, d_max = v * min(0.4*2r/v, maxTimeStep/2)
// saturates at 0.4*2r once the CFL branch is active, i.e. penetration stops
// growing with impact speed. Measured on this scene before the fix: 0.037 /
// 0.089 / 0.171 at 5 / 10 / 20 m/s (tracking v * 0.01 = v * frame dt).

namespace
{
// Fires one particle at a floor at y = 0 with no gravity and no neighbors, so
// nothing but the wall acts on it, and returns the deepest y it reaches.
float deepestPenetrationAgainstFloor(const float impactSpeed)
{
  constexpr float radius    = 0.025f;
  constexpr float frameStep = 0.01f;
  constexpr float startY    = 0.5f;

  DFSPHFluid fluid;
  fluid.density      = 1000.0f;
  fluid.viscosityCoe = 0.0f;
  fluid.pressureCoe  = 0.0f;
  fluid.setEffectLength(2.25f * radius);   // the repo h = 2.25*radius convention
  fluid.createParticle(Vector3df(0.0f, startY, 0.0f), radius, 1.0f);
  fluid.getParticles().velocities[0] = Vector3df(0.0f, -impactSpeed, 0.0f);

  DFSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, 0.0f, 0.0f));
  solver.setTimeStep(frameStep);
  // Ceiling far away so the rebound never reaches it and the only wall event
  // in the run is the one being measured.
  solver.setBoundary(Box3df(Vector3df(-1.0f, 0.0f, -1.0f), Vector3df(1.0f, 10.0f, 1.0f)),
                     frameStep);

  float deepest = startY;
  // 0.2 s covers the descent (0.1 s at the slowest speed tested) plus the
  // whole contact and rebound.
  const int frames = static_cast<int>(0.2f / frameStep);
  for (int i = 0; i < frames; ++i) {
    solver.simulate(frameStep, 1);
    const float y = fluid.getParticles().positions[0].y;
    if (!std::isfinite(y)) return -std::numeric_limits<float>::infinity();
    deepest = std::min(deepest, y);
  }
  return deepest;
}
}

TEST(DFSPHSolverTest, WallPenetrationDoesNotGrowWithImpactSpeed)
{
  // 0.4 * 2 * radius = 0.02 is the ceiling the CFL substep imposes on
  // v * substep; allow 2x for the discrete phase of first detection.
  constexpr float kMaxPenetration = 0.04f;

  const float slow   = deepestPenetrationAgainstFloor(5.0f);
  const float medium = deepestPenetrationAgainstFloor(10.0f);
  const float fast   = deepestPenetrationAgainstFloor(20.0f);

  EXPECT_GT(slow,   -kMaxPenetration);
  EXPECT_GT(medium, -kMaxPenetration);
  EXPECT_GT(fast,   -kMaxPenetration);

  // The point of the fix: 4x the impact speed must not mean 4x the sinking.
  EXPECT_LT(std::abs(fast), std::abs(slow) * 2.0f)
      << "penetration is still scaling with impact speed: slow=" << slow
      << " fast=" << fast;
}

TEST(DFSPHSolverTest, SimulateAdvancesRequestedFrameDurationNotMaxSubstep)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.viscosityCoe = 0.0f;
  fluid.pressureCoe = 0.0f;
  fluid.setEffectLength(0.1f);
  fluid.createParticle(Vector3df(0.0f), 0.025f, 1.0f);

  DFSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, -9.8f, 0.0f));
  solver.setTimeStep(0.002f); // maximum internal substep, not frame duration
  solver.simulate(0.01f, 2);

  EXPECT_NEAR(fluid.getParticles().velocities[0].y, -0.098f, 1.0e-5f);
}

TEST(DFSPHSolverTest, SphereBoundaryUsesCommonShapeInterface)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.viscosityCoe = 0.0f;
  fluid.pressureCoe = 0.0f;
  fluid.setEffectLength(0.2f);
  fluid.createParticle(Vector3df(1.01f, 0.0f, 0.0f), 0.025f, 1.0f);

  DFSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f));
  solver.setTimeStep(0.005f);
  solver.setShapeBoundaries(
      { std::make_shared<SphereBoundary>(Vector3df(0.0f), 1.0f, 0.2f) }, 0.005f);
  solver.simulate(0.005f, 2);

  EXPECT_LT(fluid.getParticles().velocities[0].x, 0.0f);
  EXPECT_LT(fluid.getParticles().positions[0].x, 1.01f);
}

TEST(DFSPHSolverTest, CommonEffectLengthInterfaceUpdatesRegisteredFluid)
{
  DFSPHFluid fluid;
  DFSPHSolver concreteSolver;
  concreteSolver.add(&fluid);
  ISPHSolver* solver = &concreteSolver;

  solver->setEffectLength(0.125f);

  EXPECT_FLOAT_EQ(fluid.getKernel()->getEffectLength(), 0.125f);
}

TEST(DFSPHSolverTest, PlateBoundaryUsesSameRegistrationAndForcePath)
{
  DFSPHFluid fluid;
  fluid.density = 1000.0f;
  fluid.viscosityCoe = 0.0f;
  fluid.pressureCoe = 0.0f;
  fluid.setEffectLength(0.2f);
  fluid.createParticle(Vector3df(0.0f), 0.025f, 1.0f);

  DFSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f));
  solver.setTimeStep(0.005f);
  solver.setBoundaryPlates(
      { PlateBoundary(Vector3df(0.0f), Vector3df(0.0f, 0.0f, 1.0f),
                      Vector3df(1.0f, 0.0f, 0.0f), 1.0f, 1.0f, 0.05f) },
      0.005f);
  solver.simulate(0.005f, 2);

  EXPECT_TRUE(std::isfinite(fluid.getParticles().velocities[0].z));
  EXPECT_GT(std::abs(fluid.getParticles().velocities[0].z), 0.0f);
}
