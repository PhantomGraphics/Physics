#include "pch.h"

#include "../Physics/CloudSolver.h"

#include <cmath>

using namespace Phantom::Physics;
using Phantom::Math::Vector3dd;

namespace
{
struct Scene {
  CloudSolver solver;
  CloudParticleSoA soa;
  CloudWaterLedger ledger;
  double time = 0.0;

  Scene()
  {
    solver.params.domainMin = Vector3dd(0.0);
    solver.params.domainMax = Vector3dd(500.0, 500.0, 1000.0);
    solver.params.spacing = 62.5;
    CloudOps::fillEnvironmentLattice(soa, solver.thermo, solver.environment, solver.params.domainMin,
                                     solver.params.domainMax, solver.params.spacing, ledger);
    solver.initialize(soa);
  }

  void run(double seconds, const std::vector<CloudSourceParams>& sources = {}, double dtMax = 0.5)
  {
    const double end = time + seconds;
    while (time < end - 1.0e-12) {
      const double dt = std::min({ solver.stableTimeStep(soa), dtMax, end - time });
      ASSERT_TRUE(solver.step(soa, sources, time, dt, ledger));
      time += dt;
    }
  }
};

double maxSpeed(const CloudParticleSoA& s)
{
  double m = 0.0;
  for (const auto& v : s.velocities) m = std::max(m, std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z));
  return m;
}
}

TEST(CloudSolverTest, RestLatticeStaysAtRest)
{
    SKIP_IN_DEBUG_SLOW();
  Scene s;
  ASSERT_EQ(s.soa.size(), 8u * 8u * 16u);
  s.run(60.0);
  const CloudStats st = CloudOps::computeStats(s.soa, s.ledger);
  EXPECT_EQ(st.nonFiniteCount, 0u);
  EXPECT_EQ(st.negativeCount, 0u);
  EXPECT_EQ(st.cloudyParticles, 0u);
  EXPECT_LT(maxSpeed(s.soa), 1.0e-6);   // env-balanced state is an exact equilibrium
  EXPECT_LT(st.balanceError, 1.0e-12);
}

TEST(CloudSolverTest, StableTimeStepIsPositiveAndBounded)
{
  Scene s;
  const double dt = s.solver.stableTimeStep(s.soa);
  EXPECT_GT(dt, 0.0);
  EXPECT_LE(dt, 0.25 * s.solver.params.effectLength() / s.solver.params.soundSpeed + 1.0e-12);
}

TEST(CloudSolverTest, HeatedAirRisesAndClosedWaterIsConserved)
{
    SKIP_IN_DEBUG_SLOW();
  Scene s;
  CloudSourceParams src;
  src.center = Vector3dd(250.0, 250.0, 150.0);
  src.radius = 150.0;
  src.heatingRate = 0.2;
  src.endTime = 30.0;
  s.run(90.0, { src });
  const CloudStats st = CloudOps::computeStats(s.soa, s.ledger);
  EXPECT_EQ(st.nonFiniteCount, 0u);
  EXPECT_EQ(st.negativeCount, 0u);
  EXPECT_LT(st.balanceError, 1.0e-10);
  // Warm parcels accelerate upward (buoyancy is the only driver of vertical motion).
  double maxUp = 0.0;
  for (const auto& v : s.soa.velocities) maxUp = std::max(maxUp, v.z);
  EXPECT_GT(maxUp, 0.5);
  for (const auto& p : s.soa.positions) {
    EXPECT_GE(p.z, 0.0);
    EXPECT_LE(p.z, 1000.0);
  }
  EXPECT_LT(s.solver.diagnostics().maxDensityRatio, 2.0);   // weakly compressible: no collapse
}

TEST(CloudSolverTest, MoistSourceFormsCloudInsideClosedDomain)
{
    SKIP_IN_DEBUG_SLOW();
  Scene s;
  CloudSourceParams src;
  src.center = Vector3dd(250.0, 250.0, 300.0);
  src.radius = 150.0;
  src.vaporRate = 3000.0;
  src.heatingRate = 0.1;
  src.endTime = 30.0;
  s.run(30.0, { src });
  const CloudStats st = CloudOps::computeStats(s.soa, s.ledger);
  EXPECT_GT(st.totalCloudWater, 0.0);          // qc from zero
  EXPECT_GT(st.cloudyParticles, 0u);
  EXPECT_LT(st.balanceError, 1.0e-10);
  EXPECT_EQ(s.solver.diagnostics().adjustFailures, 0u);
}

TEST(CloudSolverTest, SameSetupIsReproducible)
{
    SKIP_IN_DEBUG_SLOW();
  auto run = [] {
    Scene s;
    CloudSourceParams src;
    src.center = Vector3dd(250.0, 250.0, 300.0);
    src.radius = 150.0;
    src.vaporRate = 3000.0;
    src.heatingRate = 0.2;
    src.endTime = 20.0;
    s.run(40.0, { src });
    return CloudOps::computeStats(s.soa, s.ledger);
  };
  const CloudStats a = run(), b = run();
  EXPECT_EQ(a.totalCloudWater, b.totalCloudWater);
  EXPECT_EQ(a.cloudTop, b.cloudTop);
}

TEST(CloudSolverTest, EmptyOrZeroDtIsRejected)
{
  CloudSolver solver;
  CloudParticleSoA soa;
  CloudWaterLedger ledger;
  EXPECT_FALSE(solver.step(soa, {}, 0.0, 0.1, ledger));
  soa.add(Vector3dd(1.0), Vector3dd(0.0), 1.0, 290.0, 0.01, 0.0);
  EXPECT_FALSE(solver.step(soa, {}, 0.0, 0.0, ledger));
}
