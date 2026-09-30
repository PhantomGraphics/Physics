#include "pch.h"

#include "../Physics/CloudThermodynamics.h"

#include <cmath>
#include <limits>

using namespace Phantom::Physics;
namespace CT = Phantom::Physics::CloudThermodynamics;

namespace
{
const CloudThermoParams kTh;

double rel(double a, double b) { return std::abs(a - b) / std::max(std::abs(b), 1.0e-30); }

// Conserved by isobaric saturation adjustment: cp*T + L*qv.
double enthalpy(double t, double qv) { return kTh.cpDry * t + kTh.latentHeat * qv; }
}

TEST(CloudThermodynamicsTest, SaturationVaporPressureReference)
{
  EXPECT_NEAR(CT::saturationVaporPressure(273.15), 611.2, 1.0e-9);
  // ~2339 Pa at 20 C, ~4246 Pa at 30 C (Bolton within 0.5%).
  EXPECT_NEAR(CT::saturationVaporPressure(293.15), 2339.0, 2339.0 * 0.005);
  EXPECT_NEAR(CT::saturationVaporPressure(303.15), 4246.0, 4246.0 * 0.005);
  double prev = 0.0;
  for (double t = 250.0; t < 320.0; t += 1.0) {
    const double es = CT::saturationVaporPressure(t);
    EXPECT_GT(es, prev);
    prev = es;
  }
}

TEST(CloudThermodynamicsTest, SaturationDerivativeMatchesFiniteDifference)
{
  for (double t : { 260.0, 280.0, 300.0 }) {
    const double h = 1.0e-3;
    const double fd = (CT::saturationVaporPressure(t + h) - CT::saturationVaporPressure(t - h)) / (2.0 * h);
    EXPECT_NEAR(CT::saturationVaporPressureDerivative(t), fd, std::abs(fd) * 1.0e-6);
  }
}

TEST(CloudThermodynamicsTest, EnvironmentProfile)
{
  CloudEnvironmentParams env;
  EXPECT_NEAR(CT::environmentTemperature(env, 0.0), 300.0, 1.0e-12);
  EXPECT_NEAR(CT::environmentTemperature(env, 1000.0), 300.0 - 6.5, 1.0e-12);
  EXPECT_NEAR(CT::environmentPressure(kTh, env, 0.0), 1.0e5, 1.0e-6);
  // Hydrostatic consistency: dp/dz = -rho g.
  const double z = 500.0, h = 0.5;
  const double dpdz = (CT::environmentPressure(kTh, env, z + h) - CT::environmentPressure(kTh, env, z - h)) / (2.0 * h);
  const double rho = CT::environmentPressure(kTh, env, z) / (kTh.gasConstantDry * CT::environmentTemperature(env, z));
  EXPECT_NEAR(dpdz, -rho * kTh.gravity, std::abs(dpdz) * 1.0e-4);
  // Environment vapour = RH * saturation.
  const double qenv = CT::environmentVaporMixingRatio(kTh, env, z);
  const double qs = CT::saturationMixingRatio(kTh, CT::environmentTemperature(env, z), CT::environmentPressure(kTh, env, z));
  EXPECT_NEAR(qenv, env.relativeHumidity * qs, 1.0e-15);
}

TEST(CloudThermodynamicsTest, UnsaturatedAirIsUnchanged)
{
  const double p = 9.0e4, t = 290.0;
  const double qs = CT::saturationMixingRatio(kTh, t, p);
  const CloudSaturationResult r = CT::saturationAdjust(kTh, t, 0.5 * qs, 0.0, p);
  EXPECT_TRUE(r.converged);
  EXPECT_EQ(r.condensed, 0.0);
  EXPECT_EQ(r.temperature, t);
  EXPECT_EQ(r.qv, 0.5 * qs);
  EXPECT_EQ(r.qc, 0.0);
}

TEST(CloudThermodynamicsTest, SupersaturatedAirCondensesAndWarms)
{
  const double p = 9.0e4, t = 285.0;
  const double qs = CT::saturationMixingRatio(kTh, t, p);
  const double qv = 1.3 * qs;
  const CloudSaturationResult r = CT::saturationAdjust(kTh, t, qv, 0.0, p);
  ASSERT_TRUE(r.converged);
  EXPECT_GT(r.condensed, 0.0);
  EXPECT_GT(r.temperature, t);                       // latent heating
  EXPECT_GT(r.qc, 0.0);
  EXPECT_GE(r.qv, 0.0);
  EXPECT_LT(rel(r.qv + r.qc, qv), 1.0e-6);           // total water
  EXPECT_LT(rel(enthalpy(r.temperature, r.qv), enthalpy(t, qv)), 1.0e-5);  // latent heat balance
  // Result sits on the saturation curve.
  EXPECT_NEAR(r.qv, CT::saturationMixingRatio(kTh, r.temperature, p), 1.0e-9);
}

TEST(CloudThermodynamicsTest, SubsaturatedCloudEvaporatesAndCools)
{
  const double p = 9.0e4, t = 285.0;
  const double qs = CT::saturationMixingRatio(kTh, t, p);
  const double qv = 0.6 * qs, qc = 4.0e-3;
  const CloudSaturationResult r = CT::saturationAdjust(kTh, t, qv, qc, p);
  ASSERT_TRUE(r.converged);
  EXPECT_LT(r.condensed, 0.0);
  EXPECT_LT(r.temperature, t);                       // evaporative cooling
  EXPECT_GE(r.qc, 0.0);
  EXPECT_GT(r.qv, qv);
  EXPECT_LT(rel(r.qv + r.qc, qv + qc), 1.0e-6);
  EXPECT_LT(rel(enthalpy(r.temperature, r.qv), enthalpy(t, qv)), 1.0e-5);
}

TEST(CloudThermodynamicsTest, EvaporationNeverExceedsCloudWater)
{
  const double p = 9.0e4, t = 285.0;
  const double qc = 1.0e-6;  // tiny; very dry air would like to evaporate far more
  const CloudSaturationResult r = CT::saturationAdjust(kTh, t, 0.0, qc, p);
  ASSERT_TRUE(r.converged);
  EXPECT_EQ(r.qc, 0.0);
  EXPECT_NEAR(r.qv, qc, 1.0e-18);
  EXPECT_LT(r.temperature, t);
}

TEST(CloudThermodynamicsTest, ZeroWaterIsANoOp)
{
  const CloudSaturationResult r = CT::saturationAdjust(kTh, 280.0, 0.0, 0.0, 8.0e4);
  EXPECT_TRUE(r.converged);
  EXPECT_EQ(r.qv, 0.0);
  EXPECT_EQ(r.qc, 0.0);
  EXPECT_EQ(r.temperature, 280.0);
}

TEST(CloudThermodynamicsTest, ExtremeSupersaturationStaysValid)
{
  const CloudSaturationResult r = CT::saturationAdjust(kTh, 260.0, 0.05, 0.0, 7.0e4);
  ASSERT_TRUE(r.converged);
  EXPECT_GE(r.qv, 0.0);
  EXPECT_GE(r.qc, 0.0);
  EXPECT_LT(rel(r.qv + r.qc, 0.05), 1.0e-9);
  EXPECT_LT(rel(enthalpy(r.temperature, r.qv), enthalpy(260.0, 0.05)), 1.0e-5);
}

TEST(CloudThermodynamicsTest, InvalidInputLeavesStateUnchanged)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  struct Case { double t, qv, qc, p; };
  const Case cases[] = {
    { nan, 0.01, 0.0, 9.0e4 }, { 290.0, nan, 0.0, 9.0e4 }, { 290.0, 0.01, inf, 9.0e4 },
    { 290.0, -0.01, 0.0, 9.0e4 }, { 290.0, 0.01, -1.0e-3, 9.0e4 },
    { 290.0, 0.01, 0.0, 0.0 }, { -5.0, 0.01, 0.0, 9.0e4 },
  };
  for (const Case& c : cases) {
    const CloudSaturationResult r = CT::saturationAdjust(kTh, c.t, c.qv, c.qc, c.p);
    EXPECT_FALSE(r.converged);
    EXPECT_EQ(r.condensed, 0.0);
    if (std::isfinite(c.t)) EXPECT_EQ(r.temperature, c.t);
  }
}

TEST(CloudThermodynamicsTest, LiftThenDescendFormsAndEvaporatesCloud)
{
  CloudEnvironmentParams env;
  env.relativeHumidity = 0.9;
  // Parcel starts slightly warm and moist at the surface, is lifted 3 km in 10 m
  // steps (dry-adiabatic cooling + saturation adjustment), then returned.
  double z = 0.0;
  double t = CT::environmentTemperature(env, z) + 1.0;
  double qv = CT::environmentVaporMixingRatio(kTh, env, z) + 2.0e-3;
  double qc = 0.0;
  const double totalWater = qv + qc;
  const double t0 = t, qv0 = qv;
  double pPrev = CT::environmentPressure(kTh, env, z);

  bool everCloudy = false;
  double maxQc = 0.0;
  auto move = [&](double dz) {
    z += dz;
    const double p = CT::environmentPressure(kTh, env, z);
    t = CT::adiabaticTemperature(kTh, t, pPrev, p);
    pPrev = p;
    const CloudSaturationResult r = CT::saturationAdjust(kTh, t, qv, qc, p);
    ASSERT_TRUE(r.converged);
    t = r.temperature; qv = r.qv; qc = r.qc;
    EXPECT_GE(qv, 0.0);
    EXPECT_GE(qc, 0.0);
    EXPECT_LT(rel(qv + qc, totalWater), 1.0e-12);
    if (qc > 1.0e-6) everCloudy = true;
    maxQc = std::max(maxQc, qc);
  };

  double baseZ = -1.0;
  for (int i = 0; i < 300; ++i) {
    move(10.0);
    if (baseZ < 0.0 && qc > 1.0e-6) baseZ = z;
  }
  EXPECT_TRUE(everCloudy);
  EXPECT_GT(baseZ, 0.0);
  EXPECT_LT(baseZ, 3000.0);                          // condensation onset (cloud base) inside the lift
  EXPECT_GT(maxQc, 1.0e-3);                          // meaningful cloud water at the top
  const double qcTop = qc;
  for (int i = 0; i < 300; ++i) move(-10.0);
  EXPECT_LT(qc, qcTop);                              // evaporation on descent
  EXPECT_LT(qc, 1.0e-6);                             // returned to (near) clear air
  EXPECT_NEAR(t, t0, 0.1);                           // reversible: returns to the starting temperature
  EXPECT_NEAR(qv, qv0, 1.0e-4);
}

TEST(CloudThermodynamicsTest, LargeLiftStepStaysValid)
{
  CloudEnvironmentParams env;
  double t = CT::environmentTemperature(env, 0.0) + 1.0;
  double qv = 0.014;
  const double p0 = CT::environmentPressure(kTh, env, 0.0);
  const double p1 = CT::environmentPressure(kTh, env, 4000.0);
  t = CT::adiabaticTemperature(kTh, t, p0, p1);
  const CloudSaturationResult r = CT::saturationAdjust(kTh, t, qv, 0.0, p1);
  ASSERT_TRUE(r.converged);
  EXPECT_GT(r.qc, 0.0);
  EXPECT_GE(r.qv, 0.0);
  EXPECT_LT(rel(r.qv + r.qc, qv), 1.0e-12);
}

TEST(CloudThermodynamicsTest, WarmMoistParcelIsBuoyant)
{
  const double tEnv = 290.0;
  EXPECT_GT(CT::buoyancy(kTh, 291.0, 0.005, 0.0, tEnv, 0.005), 0.0);
  EXPECT_LT(CT::buoyancy(kTh, 290.0, 0.005, 5.0e-3, tEnv, 0.005), 0.0);  // water loading
  EXPECT_NEAR(CT::buoyancy(kTh, tEnv, 0.005, 0.0, tEnv, 0.005), 0.0, 1.0e-15);
}
