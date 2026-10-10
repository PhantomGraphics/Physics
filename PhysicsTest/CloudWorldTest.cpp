#include "pch.h"

#include "../PhysicsView/CloudWorld.h"

#include <cmath>

using namespace Phantom;
using Phantom::Physics::CloudSourceParams;
using Phantom::Math::Vector3dd;

namespace
{
void smallDomain(CloudWorld& w)
{
  w.config().solver.domainMax = Vector3dd(500.0, 500.0, 1000.0);
  w.config().solver.spacing = 62.5;
  w.reset();
}

CloudSourceParams moistSource(Vector3dd c, double endTime = 30.0)
{
  CloudSourceParams s;
  s.center = c;
  s.radius = 150.0;
  s.vaporRate = 3000.0;
  s.heatingRate = 0.1;
  s.endTime = endTime;
  return s;
}
}

TEST(CloudWorldTest, ResetIsReproducibleAndSeedDependent)
{
  CloudWorld a, b, c;
  smallDomain(a); smallDomain(b); smallDomain(c);
  b.reset(1);
  c.reset(2);
  ASSERT_EQ(a.particles().size(), b.particles().size());
  bool sameAB = true, sameAC = true;
  for (size_t i = 0; i < a.particles().size(); ++i) {
    if (a.particles().positions[i] != b.particles().positions[i]) sameAB = false;
    if (a.particles().positions[i] != c.particles().positions[i]) sameAC = false;
  }
  EXPECT_TRUE(sameAB);
  EXPECT_FALSE(sameAC);
}

TEST(CloudWorldTest, PausedWorldDoesNotAdvanceAndStepOnceDoes)
{
  CloudWorld w;
  smallDomain(w);
  w.addSource(moistSource(Vector3dd(250.0, 250.0, 300.0)));
  w.update(1.0);
  EXPECT_EQ(w.simTime(), 0.0);
  EXPECT_EQ(w.stepCount(), 0u);
  w.stepOnce();
  EXPECT_GT(w.simTime(), 0.0);
  EXPECT_EQ(w.stepCount(), 1u);
  w.setRunning(true);
  const double t = w.simTime();
  w.update(0.5);
  EXPECT_GT(w.simTime(), t);
}

TEST(CloudWorldTest, OverloadIsCountedNotHidden)
{
  CloudWorld w;
  smallDomain(w);
  w.config().maxSubstepsPerFrame = 1;
  w.config().timeScale = 100.0;
  w.setRunning(true);
  w.update(1.0);   // wants 100 sim seconds in 1 substep
  EXPECT_EQ(w.overloadedFrames(), 1u);
  EXPECT_GT(w.droppedSimTime(), 90.0);
  EXPECT_EQ(w.stepCount(), 1u);
}

TEST(CloudWorldTest, FormGrowEvaporateReform)
{
    SKIP_IN_DEBUG_SLOW();
  CloudWorld w;
  smallDomain(w);
  const size_t s0 = w.addSource(moistSource(Vector3dd(150.0, 250.0, 300.0)));
  (void)s0;
  w.advance(30.0);
  const Physics::CloudStats formed = w.stats();
  EXPECT_GT(formed.totalCloudWater, 0.0);
  EXPECT_LT(formed.balanceError, 1.0e-10);

  // Dry the air: existing cloud must evaporate back into vapour (no loss).
  const double totalBefore = formed.totalWater;
  w.setRelativeHumidity(0.15);
  w.advance(60.0);
  const Physics::CloudStats dry = w.stats();
  EXPECT_LT(dry.totalCloudWater, 0.1 * formed.totalCloudWater);
  EXPECT_LT(dry.balanceError, 1.0e-10);            // drying was recorded as an external removal
  EXPECT_LT(dry.totalWater, totalBefore);          // ...so the total did drop, and the ledger says why
  EXPECT_LT(w.ledger().suppliedWater, formed.totalWater);

  // Re-form at another place from the remaining air state.
  CloudSourceParams again = moistSource(Vector3dd(350.0, 250.0, 300.0), 1.0e30);
  again.startTime = w.simTime();
  again.endTime = again.startTime + 30.0;
  again.vaporRate = 6000.0;
  w.addSource(again);
  w.advance(30.0);
  const Physics::CloudStats re = w.stats();
  EXPECT_GT(re.totalCloudWater, dry.totalCloudWater);
  EXPECT_EQ(re.nonFiniteCount, 0u);
  EXPECT_EQ(re.negativeCount, 0u);
  EXPECT_LT(re.balanceError, 1.0e-10);
}

TEST(CloudWorldTest, HumidityChangeIsLedgered)
{
  CloudWorld w;
  smallDomain(w);
  const double before = w.stats().totalWater;
  w.setRelativeHumidity(0.9);
  const Physics::CloudStats s = w.stats();
  EXPECT_GT(s.totalWater, before);
  EXPECT_LT(s.balanceError, 1.0e-12);
}

TEST(CloudWorldTest, TimeStepHalvingKeepsResultsClose)
{
    SKIP_IN_DEBUG_SLOW();
  auto run = [](double fixedDt) {
    CloudWorld w;
    smallDomain(w);
    w.config().fixedTimeStep = fixedDt;
    w.addSource(moistSource(Vector3dd(250.0, 250.0, 300.0), 40.0));
    w.advance(60.0);
    return w.stats();
  };
  const Physics::CloudStats a = run(0.2), b = run(0.1);
  ASSERT_GT(a.totalCloudWater, 0.0);
  // Plan target: max cloud water / cloud top within 10% for dt vs dt/2.
  EXPECT_NEAR(a.totalCloudWater, b.totalCloudWater, 0.10 * b.totalCloudWater);
  EXPECT_NEAR(a.cloudTop, b.cloudTop, 0.10 * std::max(b.cloudTop, 1.0) + 62.5);
}

TEST(CloudWorldTest, ParticleSpacingSensitivityIsRecorded)
{
    SKIP_IN_DEBUG_SLOW();
  // Plan Phase 2: record how formation changes with particle spacing (values are logged, not tuned to).
  auto run = [](double spacing) {
    CloudWorld w;
    w.config().solver.domainMax = Vector3dd(500.0, 500.0, 1000.0);
    w.config().solver.spacing = spacing;
    w.reset();
    w.addSource(moistSource(Vector3dd(250.0, 250.0, 300.0), 40.0));
    double formTime = -1.0;
    for (int i = 0; i < 60; ++i) {
      w.advance(1.0);
      if (formTime < 0.0 && w.stats().totalCloudWater > 0.0) formTime = w.simTime();
    }
    const Physics::CloudStats s = w.stats();
    std::printf("[spacing %.2f] particles=%zu formTime=%.1f s cloudWater=%.4g kg top=%.0f m\n",
                spacing, s.particleCount, formTime, s.totalCloudWater, s.cloudTop);
    return std::make_pair(formTime, s);
  };
  const auto coarse = run(83.3333333);
  const auto medium = run(62.5);
  const auto fine = run(50.0);
  // Root cause of an earlier 0 / 1072 / 2268 kg spread: mixing was a per-pair rate (1/s), so its
  // effective diffusivity scaled with spacing^2. It is now a physical diffusivity [m^2/s]
  // (Cleary-Monaghan Laplacian). Measured 2026-09-30: form 18 / 16 / 15 s, cloud water at
  // 60 s 5170 / 4891 / 5387 kg for spacing 83.3 / 62.5 / 50 m.
  for (const auto* r : { &coarse, &medium, &fine }) {
    EXPECT_GT(r->first, 0.0);
    EXPECT_LT(r->second.balanceError, 1.0e-10);
    EXPECT_EQ(r->second.nonFiniteCount, 0u);
    EXPECT_NEAR(r->second.totalCloudWater, fine.second.totalCloudWater, 0.20 * fine.second.totalCloudWater);
    EXPECT_NEAR(r->first, fine.first, 4.0);
  }
}

TEST(CloudWorldTest, SourceListEditing)
{
  CloudWorld w;
  EXPECT_EQ(w.addSource(CloudSourceParams()), 0u);
  EXPECT_EQ(w.addSource(CloudSourceParams()), 1u);
  CloudSourceParams s; s.heatingRate = 3.0;
  EXPECT_TRUE(w.setSource(1, s));
  EXPECT_FALSE(w.setSource(5, s));
  EXPECT_TRUE(w.removeSource(0));
  ASSERT_EQ(w.sources().size(), 1u);
  EXPECT_EQ(w.sources()[0].heatingRate, 3.0);
  w.reset();
  EXPECT_EQ(w.sources().size(), 1u);   // survives reset
  EXPECT_FALSE(w.removeSource(3));
}

TEST(CloudWorldTest, CsvRowMatchesHeaderColumnCount)
{
  CloudWorld w;
  auto commas = [](const std::string& s) { return std::count(s.begin(), s.end(), ','); };
  EXPECT_EQ(commas(CloudWorld::csvHeader()), commas(w.csvRow()));
}
