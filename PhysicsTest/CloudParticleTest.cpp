#include "pch.h"

#include "../Physics/CloudParticle.h"
#include "../Physics/CloudThermodynamics.h"

#include <cmath>
#include <cstdint>
#include <random>

using namespace Phantom::Physics;
using Phantom::Math::Vector3dd;
namespace CT = Phantom::Physics::CloudThermodynamics;

namespace
{
const CloudThermoParams kTh;

double totalOf(const CloudParticleSoA& s, const std::vector<double>& q)
{
  double sum = 0.0;
  for (size_t i = 0; i < s.size(); ++i) sum += s.mDry[i] * q[i];
  return sum;
}

CloudParticleSoA makeLattice(CloudWaterLedger& ledger, double spacing = 20.0)
{
  CloudEnvironmentParams env;
  CloudParticleSoA soa;
  CloudOps::fillEnvironmentLattice(soa, kTh, env, Vector3dd(0.0), Vector3dd(200.0, 200.0, 400.0), spacing, ledger);
  return soa;
}
}

TEST(CloudParticleTest, LatticeMatchesEnvironmentAndLedger)
{
  CloudWaterLedger ledger;
  CloudParticleSoA soa = makeLattice(ledger);
  ASSERT_EQ(soa.size(), 10u * 10u * 20u);
  for (size_t i = 0; i < soa.size(); ++i) EXPECT_EQ(soa.ids[i], i);
  const CloudStats s = CloudOps::computeStats(soa, ledger);
  EXPECT_EQ(s.nonFiniteCount, 0u);
  EXPECT_EQ(s.negativeCount, 0u);
  EXPECT_EQ(s.cloudyParticles, 0u);
  EXPECT_LT(s.balanceError, 1.0e-14);
  EXPECT_GT(s.totalVapor, 0.0);
  // Ideal-gas dry density at the surface layer ~ p/(Rd T).
  EXPECT_NEAR(soa.mDry[0] / (20.0 * 20.0 * 20.0), 1.0e5 / (kTh.gasConstantDry * 300.0), 0.05);
}

TEST(CloudParticleTest, MixPairConservesAndStaysBounded)
{
  CloudParticleSoA soa;
  soa.add(Vector3dd(0.0), Vector3dd(0.0), 2.0, 300.0, 0.010, 0.002);
  soa.add(Vector3dd(1.0), Vector3dd(0.0), 5.0, 280.0, 0.002, 0.0);
  const double w0 = totalOf(soa, soa.qv) + totalOf(soa, soa.qc);
  const double h0 = totalOf(soa, soa.temperatures);
  CloudOps::mixPair(soa, 0, 1, 0.7);
  EXPECT_NEAR(totalOf(soa, soa.qv) + totalOf(soa, soa.qc), w0, 1.0e-15);
  EXPECT_NEAR(totalOf(soa, soa.temperatures), h0, 1.0e-12);
  for (size_t i = 0; i < 2; ++i) {
    EXPECT_GE(soa.qv[i], 0.002);
    EXPECT_LE(soa.qv[i], 0.010);
    EXPECT_GE(soa.qc[i], 0.0);
    EXPECT_LE(soa.qc[i], 0.002);
  }
  CloudOps::mixPair(soa, 0, 1, 1.0);  // full equalization
  EXPECT_NEAR(soa.qv[0], soa.qv[1], 1.0e-15);
  EXPECT_NEAR(soa.temperatures[0], soa.temperatures[1], 1.0e-12);
  const double before = soa.qv[0];
  CloudOps::mixPair(soa, 0, 0, 1.0);   // self / non-positive fraction ignored
  CloudOps::mixPair(soa, 0, 1, 0.0);
  EXPECT_EQ(soa.qv[0], before);
}

TEST(CloudParticleTest, ClosedMixingConservesWaterOver1000Steps)
{
  CloudWaterLedger ledger;
  CloudParticleSoA soa = makeLattice(ledger);
  // Perturb a blob so there is something to mix (including cloud water).
  for (size_t i = 0; i < soa.size(); ++i) {
    if (i % 7 == 0) { soa.qv[i] *= 1.8; soa.qc[i] = 1.0e-3; soa.temperatures[i] += 2.0; }
  }
  ledger.initialWater = totalOf(soa, soa.qv) + totalOf(soa, soa.qc);
  std::mt19937 rng(12345);
  std::uniform_int_distribution<size_t> pick(0, soa.size() - 1);
  std::uniform_real_distribution<double> frac(0.0, 0.5);
  for (int step = 0; step < 1000; ++step) {
    for (int k = 0; k < 500; ++k) CloudOps::mixPair(soa, pick(rng), pick(rng), frac(rng));
    CloudOps::saturationAdjustAll(soa, kTh, CloudEnvironmentParams());
  }
  const CloudStats s = CloudOps::computeStats(soa, ledger);
  EXPECT_EQ(s.nonFiniteCount, 0u);
  EXPECT_EQ(s.negativeCount, 0u);
  EXPECT_LT(s.balanceError, 1.0e-4);
  EXPECT_LT(s.balanceError, 1.0e-10);  // double precision reference actually achieves this
}

TEST(CloudParticleTest, SourceSuppliesVaporAndIsLedgered)
{
  CloudWaterLedger ledger;
  CloudParticleSoA soa = makeLattice(ledger);
  CloudSourceParams src;
  src.center = Vector3dd(100.0, 100.0, 40.0);
  src.radius = 60.0;
  src.vaporRate = 10.0;
  src.heatingRate = 0.5;
  src.startTime = 1.0;
  src.endTime = 3.0;
  const double before = totalOf(soa, soa.qv);
  CloudOps::applySources(soa, { src }, 0.0, 0.5, ledger);   // before window
  EXPECT_EQ(ledger.suppliedWater, 0.0);
  CloudOps::applySources(soa, { src }, 0.5, 1.0, ledger);   // half overlaps: active 0.5 s
  EXPECT_NEAR(ledger.suppliedWater, 5.0, 1.0e-12);
  EXPECT_NEAR(totalOf(soa, soa.qv) - before, 5.0, 1.0e-9);
  CloudOps::applySources(soa, { src }, 2.0, 5.0, ledger);   // active 1 s (2..3)
  EXPECT_NEAR(ledger.suppliedWater, 15.0, 1.0e-12);
  CloudOps::applySources(soa, { src }, 4.0, 1.0, ledger);   // after window
  EXPECT_NEAR(ledger.suppliedWater, 15.0, 1.0e-12);
  const CloudStats s = CloudOps::computeStats(soa, ledger);
  EXPECT_LT(s.balanceError, 1.0e-12);
}

TEST(CloudParticleTest, SourceWithNoParticlesInReachSuppliesNothing)
{
  CloudWaterLedger ledger;
  CloudParticleSoA soa = makeLattice(ledger);
  CloudSourceParams src;
  src.center = Vector3dd(5000.0);
  src.vaporRate = 10.0;
  CloudOps::applySources(soa, { src }, 0.0, 1.0, ledger);
  EXPECT_EQ(ledger.suppliedWater, 0.0);
  src.center = Vector3dd(100.0, 100.0, 40.0);
  src.enabled = false;
  CloudOps::applySources(soa, { src }, 0.0, 1.0, ledger);
  EXPECT_EQ(ledger.suppliedWater, 0.0);
}

TEST(CloudParticleTest, HeatedMoistSourceFormsCloudThenDriesOut)
{
  CloudWaterLedger ledger;
  CloudParticleSoA soa = makeLattice(ledger);
  CloudEnvironmentParams env;
  CloudSourceParams src;
  src.center = Vector3dd(100.0, 100.0, 300.0);
  src.radius = 50.0;
  src.vaporRate = 1000.0;
  src.endTime = 5.0;
  CloudOps::applySources(soa, { src }, 0.0, 5.0, ledger);
  CloudOps::saturationAdjustAll(soa, kTh, env);
  CloudStats formed = CloudOps::computeStats(soa, ledger);
  EXPECT_GT(formed.totalCloudWater, 0.0);            // formation from qc = 0
  EXPECT_GT(formed.cloudyParticles, 0u);
  EXPECT_GT(formed.cloudTop, 0.0);
  EXPECT_LT(formed.balanceError, 1.0e-12);

  // Dry, warm out-of-cloud air mixes in: cloud water falls back to vapour, total water kept.
  for (size_t i = 0; i < soa.size(); ++i) {
    if (soa.qc[i] <= 0.0) { soa.qv[i] *= 0.1; soa.temperatures[i] += 3.0; }
  }
  ledger.initialWater = totalOf(soa, soa.qv) + totalOf(soa, soa.qc) - ledger.suppliedWater;
  std::mt19937 rng(7);
  std::uniform_int_distribution<size_t> pick(0, soa.size() - 1);
  for (int step = 0; step < 4000 && CloudOps::computeStats(soa, ledger).totalCloudWater > 1.0e-3; ++step) {
    for (int k = 0; k < 2000; ++k) CloudOps::mixPair(soa, pick(rng), pick(rng), 0.5);
    CloudOps::saturationAdjustAll(soa, kTh, env);
  }
  const CloudStats gone = CloudOps::computeStats(soa, ledger);
  EXPECT_LT(gone.totalCloudWater, formed.totalCloudWater);   // dissipation
  EXPECT_EQ(gone.negativeCount, 0u);
  EXPECT_LT(gone.balanceError, 1.0e-10);                     // no water lost by evaporation
}

TEST(CloudParticleTest, StatsDetectNonFiniteAndNegative)
{
  CloudWaterLedger ledger;
  CloudParticleSoA soa;
  soa.add(Vector3dd(0.0), Vector3dd(0.0), 1.0, 290.0, 0.01, 0.0);
  soa.add(Vector3dd(0.0), Vector3dd(0.0), 1.0, 290.0, -0.01, 0.0);
  soa.add(Vector3dd(std::nan(""), 0.0, 0.0), Vector3dd(0.0), 1.0, 290.0, 0.01, 0.0);
  const CloudStats s = CloudOps::computeStats(soa, ledger);
  EXPECT_EQ(s.nonFiniteCount, 1u);
  EXPECT_EQ(s.negativeCount, 1u);
}

TEST(CloudParticleTest, SaturationAdjustAllCountsFailures)
{
  CloudParticleSoA soa;
  soa.add(Vector3dd(0.0), Vector3dd(0.0), 1.0, 290.0, 0.01, 0.0);
  soa.add(Vector3dd(0.0), Vector3dd(0.0), 1.0, 290.0, std::nan(""), 0.0);
  CloudStats stats;
  EXPECT_EQ(CloudOps::saturationAdjustAll(soa, kTh, CloudEnvironmentParams(), &stats), 1u);
  EXPECT_EQ(stats.adjustFailures, 1u);
}
