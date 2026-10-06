#include "pch.h"

#include "../Physics/FlameFluid.h"
#include "../Physics/FlameParticle.h"
#include "../Physics/FlameSolver.h"
#include "../Physics/FlameStats.h"
#include "../PhysicsView/FlameWorld.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {
void seedClosedSphere(FlameFluid& fluid, FlameSolver& solver)
{
  fluid.fixedCarriers=true; fluid.setDensity(1); fluid.setEffectLength(0.12f);
  fluid.setPressureCoe(20); fluid.setCoolRate(0); fluid.setBurnRate(0);
  fluid.setCurlNoiseStrength(0); fluid.setVorticityEps(0);
  fluid.setThermalDiffusivity(0.005f); fluid.setFuelDiffusivity(0.005f);
  fluid.setOxygenDiffusivity(0.005f); fluid.setSootDiffusivity(0.005f);
  fluid.setLifeMax(0.001f);
  solver.add(&fluid); solver.setEffectLength(0.12f); solver.setBoundarySphere({0,0,0},0.3f);
  for(int z=-4;z<=4;++z) for(int y=-4;y<=4;++y) for(int x=-4;x<=4;++x) {
    const Vector3df pos(x*0.06f,y*0.06f,z*0.06f);
    if(getLength(pos)>0.27f) continue;
    fluid.createParticle(pos,0.03f);
    const auto i=fluid.getParticles().size()-1;
    FlameParticle p(fluid.getParticles(),i,&fluid); p.setAir(true);
    fluid.initialOxygenMass+=p.getMass(); fluid.initialHeat+=p.getMass()*p.getTemperature();
  }
}
}

TEST(FlameSolverTest, FixedSphereRetainsColdCarriersAndDisablesSources)
{
  FlameFluid fluid; FlameSolver solver; seedClosedSphere(fluid,solver);
  FlameFluid::Emitter emitter; emitter.rate=1000; emitter.airRate=1000;
  emitter.pilotTemperature=2000; emitter.radius=1; emitter.pilotHeight=1;
  fluid.addEmitter(emitter);
  const int count=fluid.getNumParticles();
  fluid.updateEmitters(1); fluid.applyPilots(); fluid.removeDead();
  EXPECT_EQ(count,fluid.getNumParticles());
  for(int i=0;i<60;++i) solver.simulate(1.0f/60);
  const auto st=computeFlameStats(fluid,nullptr,&solver);
  EXPECT_EQ(count,st.count); EXPECT_EQ(0,st.nanCount);
  EXPECT_NEAR(300,st.allAvgT,1e-3); EXPECT_LT(st.maxSpeed,0.03f); // Small lattice pressure relaxation.
  EXPECT_LT(st.maxWallPenetration,1e-6f);
}

TEST(FlameSolverTest, SphereProjectionKeepsTangentialVelocity)
{
  FlameFluid fluid; fluid.fixedCarriers=true; fluid.setPressureCoe(0); fluid.setBurnRate(0);
  fluid.setCoolRate(0); fluid.setCurlNoiseStrength(0); fluid.setVorticityEps(0);
  fluid.createParticle({1.2f,0,0},0.05f);
  fluid.getParticles().velocities[0]={1,0.4f,0};
  FlameSolver solver; solver.add(&fluid); solver.setGravity({0,0,0});
  ASSERT_TRUE(solver.setBoundarySphere({0,0,0},1));
  solver.simulate(0.001f);
  EXPECT_LE(getLength(fluid.getParticles().positions[0]),0.950001f);
  EXPECT_GT(fluid.getParticles().velocities[0].y,0.39f);
  EXPECT_GT(fluid.maxAttemptedWallPenetration,0.2f);
  EXPECT_GT(fluid.boundaryEnergyLoss,0);
  EXPECT_FALSE(solver.setBoundarySphere({0,0,0},-1));
  EXPECT_EQ(1.0f,solver.getBoundarySphere()->getRadius());
}

TEST(FlameSolverTest, ClosedSphereHeatAndChemicalDiffusionConserveTotals)
{
  FlameFluid fluid; FlameSolver solver; seedClosedSphere(fluid,solver);
  solver.setGravity({0,0,0});
  auto& gas=fluid.getParticles();
  fluid.initialHeat=0; fluid.initialOxygenMass=0;
  for(size_t i=0;i<gas.size();++i) {
    FlameParticle p(gas,i,&fluid);
    p.setTemperature(i%2?300.0f:700.0f); p.setFuel(i%2?0.0f:1.0f); p.setOxygen(i%2?1.0f:0.0f);
    fluid.initialHeat+=p.getMass()*p.getTemperature();
    fluid.initialFuelMass+=p.getMass()*p.getFuel(); fluid.initialOxygenMass+=p.getMass()*p.getOxygen();
  }
  for(int i=0;i<60;++i) solver.simulate(1.0f/60);
  const auto st=computeFlameStats(fluid,nullptr,&solver);
  EXPECT_NEAR(0,st.fuelBalanceError,1e-6); EXPECT_NEAR(0,st.oxygenBalanceError,1e-6);
  EXPECT_NEAR(0,st.heatBalanceError,1e-4);
  EXPECT_LT(st.maxT,700); EXPECT_GT(st.maxT,300);
}

TEST(FlameSolverTest, WallCoolingAndFiniteHeatingCloseHeatBudget)
{
  FlameFluid fluid; FlameSolver solver; seedClosedSphere(fluid,solver);
  solver.setGravity({0,0,0});
  FlameSolver::ThermalBoundary t; t.sourceCenter={0,-0.15f,0}; t.sourceRadius=0.12f;
  t.sourcePower=1; t.sourceDuration=0.2f; t.wallRate=2; t.wallThickness=0.12f;
  ASSERT_TRUE(solver.setThermalBoundary(t));
  for(int i=0;i<60;++i) solver.simulate(1.0f/60);
  const auto st=computeFlameStats(fluid,nullptr,&solver);
  EXPECT_NEAR(0.2,st.sourceHeat,1e-5); EXPECT_GT(st.wallHeat,0);
  EXPECT_NEAR(0,st.heatBalanceError,1e-4);
  t.wallRate=-1; EXPECT_FALSE(solver.setThermalBoundary(t));
}

TEST(FlameSolverTest, ClosedSphereBuoyancyDrivesFlowWithoutNoise)
{
  float kinetic[2]={};
  for(int trial=0;trial<2;++trial) {
    FlameFluid fluid; FlameSolver solver; seedClosedSphere(fluid,solver);
    if(!trial) solver.setGravity({0,0,0});
    FlameSolver::ThermalBoundary t; t.sourceCenter={0,-0.15f,0}; t.sourceRadius=0.12f;
    t.sourcePower=4; t.wallRate=2; t.wallThickness=0.12f;
    solver.setThermalBoundary(t);
    double core=0,outer=0,covariance=0;
    for(int i=0;i<240;++i) {
      solver.simulate(1.0f/60);
      if(i>=120) { const auto st=computeFlameStats(fluid,nullptr,&solver);
        core+=st.coreVelocityY; outer+=st.outerVelocityY; covariance+=st.thermalVelocityCovariance; }
    }
    const auto st=computeFlameStats(fluid,nullptr,&solver);
    kinetic[trial]=static_cast<float>(st.kineticEnergy);
    std::printf("sphere trial=%d core=%g outer=%g covariance=%g maxSpeed=%g densityError=%g\n",
      trial,core/120,outer/120,covariance/120,st.maxSpeed,st.densityError);
    EXPECT_EQ(0,st.nanCount); EXPECT_LT(st.maxWallPenetration,1e-6f);
    if(trial) { EXPECT_GT(core,0); EXPECT_LT(outer,0); EXPECT_GT(covariance,0); EXPECT_LT(st.speedCapFraction,0.01f); }
  }
  EXPECT_GT(kinetic[1],kinetic[0]+1e-6f);
}

TEST(FlameSolverTest, FiniteReactionRetainsParticlesAndClosesFuelBudget)
{
  FlameFluid fluid; FlameSolver solver; seedClosedSphere(fluid,solver);
  solver.setGravity({0,0,0}); fluid.setBurnRate(10);
  auto& gas=fluid.getParticles(); fluid.initialHeat=0; fluid.initialOxygenMass=0;
  for(size_t i=0;i<gas.size();++i) {
    FlameParticle p(gas,i,&fluid); p.setTemperature(1600); p.setFuel(0.2f); p.setOxygen(0.8f); p.setAir(false);
    fluid.initialHeat+=p.getMass()*p.getTemperature(); fluid.initialFuelMass+=p.getMass()*p.getFuel();
    fluid.initialOxygenMass+=p.getMass()*p.getOxygen();
  }
  const int count=fluid.getNumParticles();
  for(int i=0;i<120;++i) solver.simulate(1.0f/60);
  const auto st=computeFlameStats(fluid,nullptr,&solver);
  EXPECT_EQ(count,st.count); EXPECT_GT(st.burnedFuel,fluid.initialFuelMass*0.99);
  EXPECT_NEAR(0,st.fuelBalanceError,1e-6); EXPECT_NEAR(0,st.oxygenBalanceError,1e-6);
  EXPECT_NEAR(0,st.heatBalanceError,1e-4); EXPECT_LT(st.burningFraction,0.001f);
}

TEST(FlameWorld, SpherePresetIgnitesAndRejectsIncompatibleSolidRelease)
{
  Phantom::FlameWorld world;
  ASSERT_TRUE(world.sphericalPreset(true,0.3f,0.06f));
  const int count=world.fluid().getNumParticles();
  EXPECT_FALSE(world.sphericalPreset(false,-1,0.06f));
  EXPECT_EQ(count,world.fluid().getNumParticles());
  EXPECT_EQ(0u,world.addBody(CombustibleBody::Shape::Sphere,{0,0,0},{0.1f,0.1f,0.1f},0.01));
  for(int i=0;i<60;++i) world.stepOnce();
  const auto st=computeFlameStats(world.fluid(),nullptr,&world.solver());
  std::printf("sphere preset maxT=%g burned=%g initial=%g heatError=%g\n",st.maxT,st.burnedFuel,world.fluid().initialFuelMass,st.heatBalanceError);
  EXPECT_EQ(count,st.count); EXPECT_GT(st.burnedFuel,world.fluid().initialFuelMass*0.01);
  EXPECT_NEAR(0,st.fuelBalanceError,1e-6); EXPECT_NEAR(0,st.heatBalanceError,1e-4);
  world.stopSource(); EXPECT_EQ(0,world.solver().getThermalBoundary().sourcePower);
  world.reset(); EXPECT_FALSE(world.fluid().fixedCarriers); EXPECT_FALSE(world.solver().getBoundarySphere());
}

TEST(FlameWorld, SphereConvectionSurvivesTimeStepAndResolutionChanges)
{
  float temperatures[3]={};
  for(int trial=0;trial<3;++trial) {
    Phantom::FlameWorld world;
    ASSERT_TRUE(world.sphericalPreset(false,0.3f,trial==2?0.05f:0.06f));
    const float dt=trial==1?1.0f/120:1.0f/60;
    ASSERT_TRUE(world.setTimeStep(dt));
    auto thermal=world.solver().getThermalBoundary(); thermal.sourcePower=4;
    world.solver().setThermalBoundary(thermal);
    double core=0,outer=0,covariance=0; int samples=0;
    for(int i=0;i<(trial==1?240:120);++i) {
      world.stepOnce();
      if(world.getSimTime()>1) { const auto st=computeFlameStats(world.fluid(),nullptr,&world.solver());
        core+=st.coreVelocityY; outer+=st.outerVelocityY; covariance+=st.thermalVelocityCovariance; ++samples; }
    }
    const auto st=computeFlameStats(world.fluid(),nullptr,&world.solver()); temperatures[trial]=st.allAvgT;
    std::printf("sphere convergence trial=%d T=%g core=%g outer=%g cov=%g\n",trial,st.allAvgT,core/samples,outer/samples,covariance/samples);
    EXPECT_EQ(0,st.nanCount); EXPECT_GT(core,0); EXPECT_LT(outer,0); EXPECT_GT(covariance,0);
    EXPECT_NEAR(0,st.heatBalanceError,1e-4);
  }
  // Time-step refinement should keep the global thermal state within 5%.
  EXPECT_NEAR(temperatures[0],temperatures[1],temperatures[0]*0.05f);
  EXPECT_NEAR(temperatures[0],temperatures[2],temperatures[0]*0.1f);
}

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
  // First-order, temperature-independent law (the default is Physical, whose
  // own behavior is covered by the Phase 2 tests below).
  fluid.setCombustionModel(FlameFluid::CombustionModel::Legacy);
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
  fluid.setCombustionModel(FlameFluid::CombustionModel::Legacy);
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

// ---- Phase 1 (docs/todo/PLAN_flame_sph_pbvr_improvement.md): dt independence --

namespace
{
struct PlumeSummary {
  float avgY = 0.0f;
  float rmsSpeed = 0.0f;
  int count = 0;
};

// Emitter-fed plume with every decorative force on (curl noise, vorticity
// confinement, buoyancy), run for `seconds` at step dt. No air carriers and
// no secondary particles, so the emitter's RNG draw sequence -- one draw set
// per spawned particle -- is identical at any dt and only the spawn *times*
// shift by less than one step.
PlumeSummary runPlume(const float dt, const float seconds)
{
  FlameFluid fluid;
  fluid.setEffectLength(0.12f);
  fluid.setPressureCoe(20.0f);
  fluid.setMaxParticles(600);
  fluid.setLifeMax(50.0f);

  FlameFluid::Emitter e;
  e.radius = 0.12f;
  e.rate = 250.0f;
  fluid.addEmitter(e);

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(0.12f);
  solver.setBoundary(Box3df(Vector3df(-1.5f, -0.05f, -1.5f), Vector3df(1.5f, 4.0f, 1.5f)), dt);

  const int steps = static_cast<int>(std::lround(seconds / dt));
  for (int i = 0; i < steps; ++i) {
    solver.simulate(dt);
  }

  PlumeSummary s;
  s.count = fluid.getNumParticles();
  double sumV2 = 0.0;
  for (const auto& v : fluid.getParticles().velocities) {
    sumV2 += getLengthSquared(v);
  }
  s.avgY = averageY(fluid);
  s.rmsSpeed = static_cast<float>(std::sqrt(sumV2 / std::max(1, s.count)));
  return s;
}
}

TEST(FlameSolverTest, PlumeHeightAndSpeedDoNotDependOnTimeStep)
{
  // Before plan A1 the curl noise was a per-step velocity kick with no dt, so
  // halving dt doubled its effective acceleration; this pins that it is now a
  // proper acceleration. SPH at two step sizes is never bit-identical, so the
  // tolerance is loose (~15%), but the pre-fix drift was far outside it.
  const PlumeSummary coarse = runPlume(1.0f / 60.0f, 1.5f);
  const PlumeSummary fine = runPlume(1.0f / 120.0f, 1.5f);

  ASSERT_GT(coarse.count, 100);
  EXPECT_NEAR(static_cast<float>(fine.count), static_cast<float>(coarse.count), 0.02f * coarse.count + 2.0f);
  EXPECT_NEAR(fine.avgY, coarse.avgY, 0.15f * coarse.avgY);
  EXPECT_NEAR(fine.rmsSpeed, coarse.rmsSpeed, 0.15f * coarse.rmsSpeed);
}

TEST(FlameSolverTest, CurlNoiseRespectsTheSpeedCap)
{
  // Plan A1: the noise used to be added after forwardTime()'s maxSpeed clamp.
  FlameFluid fluid;
  fluid.setEffectLength(0.12f);
  fluid.setCurlNoiseStrength(500.0f); // absurdly strong on purpose
  fluid.setMaxSpeed(1.0f);
  fluid.setLifeMax(50.0f);
  buildShearRing(fluid);

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(0.12f);
  for (int step = 0; step < 20; ++step) {
    solver.simulate(1.0f / 60.0f);
  }
  for (const auto& v : fluid.getParticles().velocities) {
    EXPECT_LE(getLength(v), fluid.getMaxSpeed() * (1.0f + 1.0e-4f));
  }
}

TEST(FlameFluidTest, SecondarySwirlRotationPreservesSpeed)
{
  // Plan A4: v += (omega x v)*s*dt grew |v| by sqrt(1+(|omega| s dt)^2) per step.
  const Vector3df omega(0.0f, 8.0f, 3.0f);
  Vector3df v(1.0f, 0.2f, -0.5f);
  const float speed0 = getLength(v);
  for (int i = 0; i < 600; ++i) {
    v = FlameFluid::rotateBySwirl(v, omega, 1.5f, 1.0f / 60.0f);
  }
  EXPECT_NEAR(getLength(v), speed0, 1.0e-4f);

  // First-order agreement with the explicit cross-product form for a small step.
  const Vector3df v0(1.0f, 0.2f, -0.5f);
  const float dt = 1.0e-4f;
  const Vector3df exact = FlameFluid::rotateBySwirl(v0, omega, 1.5f, dt);
  const Vector3df explicitEuler = v0 + glm::cross(omega, v0) * 1.5f * dt;
  EXPECT_NEAR(getLength(exact - explicitEuler), 0.0f, 1.0e-5f);
}

TEST(FlameFluidTest, SecondarySwirlDoesNotAccelerateSmoke)
{
  // End-to-end: smoke's own velocity jitter is tiny (0.045/s^2), so with the
  // exact rotation the population's mean speed must not creep upward over time.
  FlameFluid fluid;
  fluid.setSmokeCountPerPrimary(20.0f);
  fluid.setSmokeRiseSpeed(0.3f);
  fluid.setSmokeLifeMax(100.0f);
  fluid.setSecondarySwirlStrength(5.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle p(fluid.getParticles(), 0, &fluid);
  p.setSoot(1.0f);
  fluid.getParticles().vorticities[0] = Vector3df(0.0f, 30.0f, 10.0f);

  const Vector3df noGravity(0.0f, 0.0f, 0.0f);
  fluid.updateSecondaryParticles(1.0f / 60.0f, noGravity);
  fluid.setSmokeCountPerPrimary(0.0f); // freeze the population
  auto meanSpeed = [&fluid]() {
    double sum = 0.0;
    for (const auto& sp : fluid.getSecondaryParticles()) sum += getLength(sp.velocity);
    return static_cast<float>(sum / fluid.getSecondaryParticles().size());
  };
  ASSERT_GT(fluid.getSecondaryParticles().size(), 0U);
  const float before = meanSpeed();
  for (int i = 0; i < 120; ++i) {
    fluid.updateSecondaryParticles(1.0f / 60.0f, noGravity);
  }
  EXPECT_LT(meanSpeed(), before * 1.05f);
}

// ---- Phase 2: physical combustion model --------------------------------------

namespace
{
// Physical model with everything but combustion + diffusion switched off, so
// only the reaction/diffusion bookkeeping moves scalars around.
void configureQuietPhysical(FlameFluid& fluid)
{
  fluid.setCombustionModel(FlameFluid::CombustionModel::Physical);
  fluid.setEffectLength(0.12f);
  fluid.setPressureCoe(0.0f);
  fluid.setBuoyancyCoe(0.0f);
  fluid.setVorticityEps(0.0f);
  fluid.setCurlNoiseStrength(0.0f);
  fluid.setLifeMax(1000.0f);
}

// n x n x n lattice at the given spacing, origin at `corner`; returns first index.
size_t addBlock(FlameFluid& fluid, const Vector3df& corner, const int n, const float spacing,
                const float temperature, const float fuel, const float oxygen, const float soot)
{
  const size_t first = fluid.getParticles().size();
  for (int x = 0; x < n; ++x) {
    for (int y = 0; y < n; ++y) {
      for (int z = 0; z < n; ++z) {
        fluid.createParticle(corner + Vector3df(x * spacing, y * spacing, z * spacing), 0.02f);
        FlameParticle p(fluid.getParticles(), fluid.getParticles().size() - 1, &fluid);
        p.setTemperature(temperature);
        p.setFuel(fuel);
        p.setOxygen(oxygen);
        p.setSoot(soot);
      }
    }
  }
  return first;
}
}

TEST(FlameParticleTest, PhysicalColdPremixedFuelDoesNotBurnOnItsOwn)
{
  FlameFluid fluid;
  configureQuietPhysical(fluid);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle p(fluid.getParticles(), 0, &fluid);
  p.setFuel(1.0f);
  p.setOxygen(1.0f);
  p.setTemperature(fluid.getAmbientTemperature());

  for (int i = 0; i < 600; ++i) {
    p.react(1.0f / 60.0f);
  }
  EXPECT_NEAR(p.getFuel(), 1.0f, kTol);
  EXPECT_NEAR(p.getOxygen(), 1.0f, kTol);
  EXPECT_NEAR(p.getTemperature(), fluid.getAmbientTemperature(), 1.0e-3f);
}

TEST(FlameParticleTest, PhysicalHotFuelWithoutOxygenDoesNotBurn)
{
  FlameFluid fluid;
  configureQuietPhysical(fluid);
  fluid.setCoolRate(0.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle p(fluid.getParticles(), 0, &fluid);
  p.setFuel(1.0f);
  p.setOxygen(0.0f);
  p.setTemperature(2000.0f);

  for (int i = 0; i < 600; ++i) {
    p.react(1.0f / 60.0f);
  }
  EXPECT_NEAR(p.getFuel(), 1.0f, kTol);
  EXPECT_NEAR(p.getSoot(), 0.0f, kTol);
  EXPECT_NEAR(p.getTemperature(), 2000.0f, 1.0e-2f);
}

TEST(FlameParticleTest, PhysicalHotPremixedFuelBurnsAndConsumesOxygen)
{
  FlameFluid fluid;
  configureQuietPhysical(fluid);
  fluid.setOxygenPerFuel(1.0f);
  // No cooling: alone, a lean particle otherwise drops back below ignition
  // and self-extinguishes (correct, but not what this test is about).
  fluid.setCoolRate(0.0f);
  fluid.createParticle(Vector3df(0.0f, 0.0f, 0.0f), 0.02f);
  FlameParticle p(fluid.getParticles(), 0, &fluid);
  p.setFuel(0.5f);
  p.setOxygen(1.0f);
  p.setTemperature(1500.0f);

  for (int i = 0; i < 600; ++i) {
    p.react(1.0f / 60.0f);
    ASSERT_GE(p.getFuel(), 0.0f);
    ASSERT_GE(p.getOxygen(), 0.0f);
  }
  EXPECT_LT(p.getFuel(), 0.05f);
  // Stoichiometric bookkeeping: oxygen fell by exactly what the fuel did.
  EXPECT_NEAR(1.0f - p.getOxygen(), 0.5f - p.getFuel(), 1.0e-4f);
  EXPECT_GT(p.getSoot(), 0.0f);
}

TEST(FlameFluidTest, TemperatureFactorIsAnIgnitionThreshold)
{
  FlameFluid fluid;
  fluid.setCombustionModel(FlameFluid::CombustionModel::Physical);
  fluid.setIgnitionTemperature(1200.0f);
  fluid.setIgnitionWidth(150.0f);

  EXPECT_NEAR(fluid.computeTemperatureFactor(300.0f), 0.0f, kTol);
  EXPECT_NEAR(fluid.computeTemperatureFactor(1050.0f), 0.0f, kTol);
  EXPECT_NEAR(fluid.computeTemperatureFactor(1200.0f), 0.5f, kTol);
  EXPECT_NEAR(fluid.computeTemperatureFactor(1350.0f), 1.0f, kTol);
  EXPECT_NEAR(fluid.computeTemperatureFactor(2500.0f), 1.0f, kTol);

  fluid.setReactionRateModel(FlameFluid::ReactionRateModel::Arrhenius);
  EXPECT_NEAR(fluid.computeTemperatureFactor(1200.0f), 1.0f, 1.0e-4f); // normalized at T_ign
  EXPECT_LT(fluid.computeTemperatureFactor(600.0f), 0.01f);
  float prev = 0.0f;
  for (float t = 400.0f; t <= 3000.0f; t += 100.0f) {
    const float k = fluid.computeTemperatureFactor(t);
    EXPECT_GE(k, prev);
    EXPECT_LE(k, 8.0f + kTol); // capped gain
    prev = k;
  }

  // Legacy ignores temperature (and oxygen) entirely.
  fluid.setCombustionModel(FlameFluid::CombustionModel::Legacy);
  EXPECT_NEAR(fluid.computeReactionRate(300.0f, 0.5f, 0.0f), fluid.getBurnRate() * 0.5f, kTol);
}

TEST(FlameSolverTest, PhysicalFlameSpreadsFromHotNeighborsToColdMixture)
{
  // Flame spread: a hot burned-gas block touching a cold premixed block. The
  // cold block cannot ignite by itself (PhysicalColdPremixedFuelDoesNotBurnOnItsOwn);
  // only heat diffused across the interface can light it.
  FlameFluid fluid;
  configureQuietPhysical(fluid);
  fluid.setCoolRate(0.0f);
  const float spacing = 0.04f;
  addBlock(fluid, Vector3df(0.0f, 0.0f, 0.0f), 4, spacing, 2200.0f, 0.0f, 0.5f, 0.0f);
  const size_t coldFirst = addBlock(fluid, Vector3df(4 * spacing, 0.0f, 0.0f), 4, spacing, 300.0f, 1.0f, 1.0f, 0.0f);
  const size_t coldCount = fluid.getParticles().size() - coldFirst;

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(fluid.getEffectLength());
  for (int step = 0; step < 180; ++step) {
    solver.simulate(1.0f / 60.0f);
  }

  const auto& soa = fluid.getParticles();
  ASSERT_EQ(soa.size(), coldFirst + coldCount);
  float coldFuel = 0.0f;
  float coldMaxT = 0.0f;
  for (size_t i = coldFirst; i < soa.size(); ++i) {
    coldFuel += soa.fuels[i];
    coldMaxT = std::max(coldMaxT, soa.temperatures[i]);
  }
  coldFuel /= static_cast<float>(coldCount);
  EXPECT_LT(coldFuel, 0.8f);                           // part of it burned
  EXPECT_GT(coldMaxT, fluid.getIgnitionTemperature()); // and it got past ignition

  // Control: without diffusion nothing crosses the interface.
  FlameFluid isolated;
  configureQuietPhysical(isolated);
  isolated.setCoolRate(0.0f);
  isolated.setThermalDiffusivity(0.0f);
  isolated.setFuelDiffusivity(0.0f);
  isolated.setOxygenDiffusivity(0.0f);
  addBlock(isolated, Vector3df(0.0f, 0.0f, 0.0f), 4, spacing, 2200.0f, 0.0f, 0.5f, 0.0f);
  const size_t isoCold = addBlock(isolated, Vector3df(4 * spacing, 0.0f, 0.0f), 4, spacing, 300.0f, 1.0f, 1.0f, 0.0f);
  FlameSolver isoSolver;
  isoSolver.add(&isolated);
  isoSolver.setEffectLength(isolated.getEffectLength());
  for (int step = 0; step < 180; ++step) {
    isoSolver.simulate(1.0f / 60.0f);
  }
  for (size_t i = isoCold; i < isolated.getParticles().size(); ++i) {
    EXPECT_NEAR(isolated.getParticles().fuels[i], 1.0f, kTol);
  }
}

TEST(FlameSolverTest, ScalarDiffusionConservesMassWeightedTotals)
{
  // No reaction (burnRate 0), no cooling: only the diffusion pass moves T/f/o/s,
  // and it is antisymmetric per pair, so sum_i m_i A_i must not change.
  FlameFluid fluid;
  configureQuietPhysical(fluid);
  fluid.setBurnRate(0.0f);
  fluid.setCoolRate(0.0f);
  fluid.setSootDiffusivity(0.02f);
  addBlock(fluid, Vector3df(0.0f, 0.0f, 0.0f), 4, 0.04f, 1800.0f, 0.2f, 0.9f, 0.6f);
  addBlock(fluid, Vector3df(0.16f, 0.0f, 0.0f), 4, 0.04f, 400.0f, 0.9f, 0.1f, 0.05f);

  const auto totals = [&fluid]() {
    std::array<double, 4> t{};
    const auto& soa = fluid.getParticles();
    for (size_t i = 0; i < soa.size(); ++i) {
      const double m = FlameParticle(fluid.getParticles(), i, &fluid).getMass();
      t[0] += m * soa.temperatures[i];
      t[1] += m * soa.fuels[i];
      t[2] += m * soa.oxygens[i];
      t[3] += m * soa.soots[i];
    }
    return t;
  };
  const auto before = totals();

  FlameSolver solver;
  solver.add(&fluid);
  solver.setEffectLength(fluid.getEffectLength());
  for (int step = 0; step < 120; ++step) {
    solver.simulate(1.0f / 60.0f);
  }
  ASSERT_EQ(fluid.getNumParticles(), 128);
  const auto after = totals();
  for (int k = 0; k < 4; ++k) {
    EXPECT_NEAR(after[k], before[k], 1.0e-4 * std::abs(before[k])) << "scalar " << k;
  }

  // ...and it actually mixed: the two blocks' temperatures moved toward each other.
  const auto& soa = fluid.getParticles();
  float hotMean = 0.0f, coldMean = 0.0f;
  for (int i = 0; i < 64; ++i) { hotMean += soa.temperatures[i]; coldMean += soa.temperatures[64 + i]; }
  EXPECT_LT(hotMean / 64.0f, 1800.0f - 1.0f);
  EXPECT_GT(coldMean / 64.0f, 400.0f + 1.0f);
}
