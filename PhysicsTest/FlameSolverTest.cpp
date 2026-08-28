#include "pch.h"

#include "../Physics/FlameFluid.h"
#include "../Physics/FlameParticle.h"
#include "../Physics/FlameSolver.h"

#include <cmath>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
constexpr float kTol = 1.0e-5f;

float averageY(const FlameFluid& fluid)
{
  float sum = 0.0f;
  for (const auto& pos : fluid.getParticles().positions) {
    sum += pos.y;
  }
  return sum / static_cast<float>(fluid.getNumParticles());
}

float averageTemperature(const FlameFluid& fluid)
{
  float sum = 0.0f;
  for (const auto& t : fluid.getParticles().temperatures) {
    sum += t;
  }
  return sum / static_cast<float>(fluid.getNumParticles());
}

float velocityVarianceProxy(const FlameFluid& fluid)
{
  // Sum of squared velocity magnitudes; a simple spread/kinetic-energy proxy,
  // not a statistically rigorous variance (see plan: "loose smoke test").
  float sum = 0.0f;
  for (const auto& v : fluid.getParticles().velocities) {
    sum += getLengthSquared(v);
  }
  return sum;
}

// Builds a small deterministic ring of particles with a mild inherent shear
// (no emitter/RNG involved), so vorticity-confinement/curl-noise runs are
// bit-for-bit reproducible across the on/off comparison.
void buildShearRing(FlameFluid& fluid)
{
  constexpr int kCount = 24;
  for (int i = 0; i < kCount; ++i) {
    const float theta = (2.0f * 3.14159265f * i) / kCount;
    const Vector3df pos(0.1f * std::cos(theta), 0.5f, 0.1f * std::sin(theta));
    fluid.createParticle(pos, 0.02f);
    FlameParticle p(fluid.getParticles(), fluid.getParticles().size() - 1, &fluid);
    p.setFuel(1.0f);
    p.setTemperature(fluid.getIgnitionTemperature());
    // Tangential shear velocity so neighbors have differing velocities (feeds vorticity).
    p.setVelocity(Vector3df(-std::sin(theta) * 0.5f, 0.2f, std::cos(theta) * 0.5f));
  }
}
}

// ---- FlameParticle::react() -------------------------------------------------

TEST(FlameParticleTest, ReactBurnsDownFuelMonotonicallyAndCoolsAfterBurnout)
{
  FlameFluid fluid;
  fluid.setBurnRate(2.0f);
  fluid.setHeatRelease(1000.0f);
  fluid.setCoolRate(1.0f);
  fluid.setAmbientTemperature(300.0f);
  fluid.setIgnitionTemperature(1200.0f);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle p(fluid.getParticles(), 0, &fluid);
  p.setFuel(1.0f);
  p.setTemperature(fluid.getIgnitionTemperature());

  const float dt = 0.01f;
  float prevFuel = p.getFuel();
  for (int i = 0; i < 3000; ++i) {
    p.react(dt);
    EXPECT_LE(p.getFuel(), prevFuel + kTol);
    prevFuel = p.getFuel();
  }

  EXPECT_NEAR(p.getFuel(), 0.0f, 1.0e-3f);
  // Fuel exhausted -> heat production stops -> temperature relaxes back to ambient.
  EXPECT_NEAR(p.getTemperature(), fluid.getAmbientTemperature(), 5.0f);
}

TEST(FlameParticleTest, ReactRisesAboveAmbientWhileFuelBurns)
{
  FlameFluid fluid;
  fluid.setBurnRate(1.0f);
  fluid.setHeatRelease(2000.0f);
  fluid.setCoolRate(3.0f);
  fluid.setAmbientTemperature(300.0f);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle p(fluid.getParticles(), 0, &fluid);
  p.setFuel(1.0f);
  p.setTemperature(fluid.getAmbientTemperature());

  p.react(0.01f);

  EXPECT_GT(p.getTemperature(), fluid.getAmbientTemperature());
}

// ---- FlameFluid emitter / lifetime ------------------------------------------

TEST(FlameFluidTest, UpdateEmittersStopsAtMaxParticles)
{
  FlameFluid fluid;
  fluid.setMaxParticles(50);

  FlameFluid::Emitter e;
  e.center = Vector3df(0.0f, 0.0f, 0.0f);
  e.radius = 0.05f;
  e.rate = 1000.0f; // far more than maxParticles over the test duration
  fluid.addEmitter(e);

  for (int i = 0; i < 20; ++i) {
    fluid.updateEmitters(0.01f);
  }

  EXPECT_EQ(fluid.getNumParticles(), fluid.getMaxParticles());
}

TEST(FlameFluidTest, RemoveDeadDropsBurnedOutCooledParticles)
{
  FlameFluid fluid;
  fluid.setLifeMax(1.0f);
  fluid.setAmbientTemperature(300.0f);

  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle alive(fluid.getParticles(), 0, &fluid);
  alive.setFuel(1.0f);
  alive.setTemperature(1000.0f);
  alive.setAge(0.1f);

  fluid.createParticle(Vector3df(1.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle deadByAge(fluid.getParticles(), 1, &fluid);
  deadByAge.setFuel(1.0f);
  deadByAge.setTemperature(1000.0f);
  deadByAge.setAge(2.0f); // past lifeMax

  fluid.createParticle(Vector3df(2.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle deadByCooling(fluid.getParticles(), 2, &fluid);
  deadByCooling.setFuel(0.0f);
  deadByCooling.setTemperature(301.0f); // burned out and back near ambient
  deadByCooling.setAge(0.1f);

  ASSERT_EQ(fluid.getNumParticles(), 3);
  fluid.removeDead();

  EXPECT_EQ(fluid.getNumParticles(), 1);
  EXPECT_NEAR(fluid.getParticles().positions[0].x, 0.0f, kTol);
}

// ---- FlameSolver::simulate() -------------------------------------------------

TEST(FlameSolverTest, GetFluidsReturnsRegisteredFluids)
{
  FlameFluid fluidA;
  FlameFluid fluidB;

  FlameSolver solver;
  solver.add(&fluidA);
  solver.add(&fluidB);

  const auto& fluids = solver.getFluids();
  ASSERT_EQ(fluids.size(), 2U);
  EXPECT_EQ(fluids[0], &fluidA);
  EXPECT_EQ(fluids[1], &fluidB);
}

TEST(FlameSolverTest, EmittedParticlesStayFiniteAndCapAtMaxParticles)
{
  FlameFluid fluid;
  fluid.setMaxParticles(120);
  fluid.setEffectLength(0.15f);
  fluid.setDensity(1.0f);
  fluid.setPressureCoe(20.0f);
  fluid.setVicosityCoe(0.001f);
  fluid.setLifeMax(50.0f); // effectively no death within this test's duration

  FlameFluid::Emitter e;
  e.center = Vector3df(0.0f, 0.0f, 0.0f);
  e.radius = 0.05f;
  e.rate = 200.0f;
  fluid.addEmitter(e);

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(0.15f);

  const float dt = 0.01f;
  for (int step = 0; step < 300; ++step) {
    solver.simulate(dt);
  }

  EXPECT_LE(fluid.getNumParticles(), fluid.getMaxParticles());
  EXPECT_GT(fluid.getNumParticles(), 0);

  const auto& soa = fluid.getParticles();
  for (size_t i = 0; i < soa.size(); ++i) {
    const auto& pos = soa.positions[i];
    const auto& vel = soa.velocities[i];
    EXPECT_TRUE(std::isfinite(pos.x) && std::isfinite(pos.y) && std::isfinite(pos.z));
    EXPECT_TRUE(std::isfinite(vel.x) && std::isfinite(vel.y) && std::isfinite(vel.z));
  }
}

TEST(FlameSolverTest, BuoyancyLiftsAverageHeightOverTime)
{
  FlameFluid fluid;
  fluid.setEffectLength(0.15f);
  fluid.setDensity(1.0f);
  fluid.setPressureCoe(20.0f);
  fluid.setVicosityCoe(0.001f);
  fluid.setBuoyancyCoe(6.0f);
  fluid.setThermalExpansion(0.05f);
  fluid.setVorticityEps(0.0f);
  fluid.setCurlNoiseStrength(0.0f); // isolate the buoyancy trend from decorative noise
  fluid.setLifeMax(50.0f);          // no deaths within this test's duration
  fluid.setMaxParticles(200);

  FlameFluid::Emitter e;
  e.center = Vector3df(0.0f, 0.0f, 0.0f);
  e.radius = 0.05f;
  e.rate = 300.0f;
  fluid.addEmitter(e);

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(0.15f);

  const float dt = 0.01f;

  // Phase A: spawn a batch, then stop emitting so the tracked population is fixed.
  for (int step = 0; step < 30; ++step) {
    solver.simulate(dt);
  }
  fluid.clearEmitters();
  ASSERT_GT(fluid.getNumParticles(), 0);
  const float avgY0 = averageY(fluid);

  // Phase B: let buoyancy act on the same (fixed) population.
  for (int step = 0; step < 150; ++step) {
    solver.simulate(dt);
  }
  const float avgY1 = averageY(fluid);

  EXPECT_GT(avgY1, avgY0);
}

TEST(FlameSolverTest, FreshlyIgnitedParticlesAreHotterThanAmbient)
{
  FlameFluid fluid;
  fluid.setEffectLength(0.15f);
  fluid.setAmbientTemperature(300.0f);
  fluid.setIgnitionTemperature(1200.0f);
  fluid.setBurnRate(0.5f);
  fluid.setHeatRelease(2000.0f);
  fluid.setCoolRate(1.0f);
  fluid.setLifeMax(50.0f);
  fluid.setMaxParticles(200);

  FlameFluid::Emitter e;
  e.center = Vector3df(0.0f, 0.0f, 0.0f);
  e.radius = 0.05f;
  e.rate = 300.0f;
  fluid.addEmitter(e);

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(0.15f);

  for (int step = 0; step < 10; ++step) {
    solver.simulate(0.01f);
  }

  ASSERT_GT(fluid.getNumParticles(), 0);
  EXPECT_GT(averageTemperature(fluid), fluid.getAmbientTemperature());
}

// ---- Vorticity confinement / curl noise: loose smoke test -------------------

TEST(FlameSolverTest, VorticityAndCurlNoiseChangeVelocitySpread)
{
  FlameFluid offFluid;
  offFluid.setEffectLength(0.15f);
  offFluid.setVorticityEps(0.0f);
  offFluid.setCurlNoiseStrength(0.0f);
  offFluid.setLifeMax(50.0f);
  buildShearRing(offFluid);

  FlameFluid onFluid;
  onFluid.setEffectLength(0.15f);
  onFluid.setVorticityEps(5.0f);
  onFluid.setCurlNoiseStrength(2.0f);
  onFluid.setCurlNoiseFrequency(0.5f);
  onFluid.setLifeMax(50.0f);
  buildShearRing(onFluid);

  FlameSolver offSolver;
  offSolver.add(&offFluid);
  offSolver.setEffectLength(0.15f);

  FlameSolver onSolver;
  onSolver.add(&onFluid);
  onSolver.setEffectLength(0.15f);

  const float dt = 0.01f;
  for (int step = 0; step < 50; ++step) {
    offSolver.simulate(dt);
    onSolver.simulate(dt);
  }

  const float spreadOff = velocityVarianceProxy(offFluid);
  const float spreadOn = velocityVarianceProxy(onFluid);

  EXPECT_GT(std::abs(spreadOn - spreadOff), 1.0e-4f);
}
