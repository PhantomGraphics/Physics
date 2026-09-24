#include "pch.h"

#include "../PhysicsView/FlameBlackbody.h"

using namespace Phantom;

TEST(FlameBlackbodyTest, ChromaticityHasUnitLuminance)
{
  for (float t = 800.0f; t <= 8000.0f; t += 400.0f) {
    const glm::vec3 c = FlameBlackbody::chromaticity(t);
    EXPECT_NEAR(FlameBlackbody::luminance(c), 1.0f, 1.0e-4f) << "T=" << t;
    EXPECT_GE(c.r, 0.0f);
    EXPECT_GE(c.g, 0.0f);
    EXPECT_GE(c.b, 0.0f);
  }
}

TEST(FlameBlackbodyTest, FlameTemperaturesAreRedToYellowAndBlueRisesWithTemperature)
{
  // Candle-flame range: red dominates, blue is weakest.
  for (float t : { 1200.0f, 1600.0f, 2000.0f, 2500.0f }) {
    const glm::vec3 c = FlameBlackbody::chromaticity(t);
    EXPECT_GT(c.r, c.g) << "T=" << t;
    EXPECT_GT(c.g, c.b) << "T=" << t;
  }
  // Hotter = whiter: green/red increases strictly; blue/red never decreases
  // (below ~2000 K blue is outside the sRGB gamut and clips to exactly 0).
  float prevBR = -1.0f, prevGR = -1.0f;
  for (float t = 1000.0f; t <= 6000.0f; t += 250.0f) {
    const glm::vec3 c = FlameBlackbody::chromaticity(t);
    EXPECT_GE(c.b / c.r, prevBR) << "T=" << t;
    EXPECT_GT(c.g / c.r, prevGR) << "T=" << t;
    prevBR = c.b / c.r;
    prevGR = c.g / c.r;
  }
  EXPECT_GT(FlameBlackbody::chromaticity(3000.0f).b, 0.0f);
}

TEST(FlameBlackbodyTest, D65LikeTemperatureIsNearlyWhite)
{
  // A 6500 K blackbody sits close to (not exactly on) the D65 white point.
  const glm::vec3 c = FlameBlackbody::chromaticity(6500.0f);
  EXPECT_NEAR(c.r, 1.0f, 0.12f);
  EXPECT_NEAR(c.g, 1.0f, 0.12f);
  EXPECT_NEAR(c.b, 1.0f, 0.12f);
}

TEST(FlameBlackbodyTest, RelativeRadianceFollowsStefanBoltzmann)
{
  EXPECT_NEAR(FlameBlackbody::relativeRadiance(300.0f, 300.0f, 2000.0f), 0.0f, 1.0e-6f);
  EXPECT_NEAR(FlameBlackbody::relativeRadiance(2000.0f, 300.0f, 2000.0f), 1.0f, 1.0e-5f);
  EXPECT_NEAR(FlameBlackbody::relativeRadiance(200.0f, 300.0f, 2000.0f), 0.0f, 1.0e-6f); // clamped
  // Above the reference it exceeds 1 (HDR): (4000^4 - 300^4) / (2000^4 - 300^4) ~ 16.
  EXPECT_NEAR(FlameBlackbody::relativeRadiance(4000.0f, 300.0f, 2000.0f), 16.0f, 0.05f);
  EXPECT_NEAR(FlameBlackbody::relativeRadiance(1000.0f, 300.0f, 300.0f), 0.0f, 1.0e-6f); // degenerate
}

TEST(FlameBlackbodyTest, LutMatchesDirectEvaluation)
{
  const auto lut = FlameBlackbody::makeLut();
  ASSERT_EQ(lut.size(), static_cast<size_t>(FlameBlackbody::kLutSize));
  const glm::vec3 first = FlameBlackbody::chromaticity(FlameBlackbody::kLutMinT);
  const glm::vec3 last = FlameBlackbody::chromaticity(FlameBlackbody::kLutMaxT);
  EXPECT_NEAR(lut.front().r, first.r, 1.0e-6f);
  EXPECT_NEAR(lut.back().b, last.b, 1.0e-6f);
  EXPECT_NEAR(lut.front().w, 1.0f, 1.0e-6f);
}

TEST(FlameBlackbodyTest, FullAdaptationMapsTheWhiteTemperatureToNeutral)
{
  for (float w : { 1200.0f, 1800.0f, 2500.0f }) {
    const glm::vec3 c = FlameBlackbody::chromaticity(w, w, 1.0f);
    EXPECT_NEAR(c.r, 1.0f, 0.02f) << "white=" << w;
    EXPECT_NEAR(c.g, 1.0f, 0.02f) << "white=" << w;
    EXPECT_NEAR(c.b, 1.0f, 0.02f) << "white=" << w;
  }
}

TEST(FlameBlackbodyTest, AdaptationKeepsHueOrderAndDegreeInterpolates)
{
  // Adapted to 2000 K: cooler gas is still warm (r > g > b), hotter gas turns bluish.
  const glm::vec3 cool = FlameBlackbody::chromaticity(1200.0f, 2000.0f, 1.0f);
  EXPECT_GT(cool.r, cool.g);
  EXPECT_GT(cool.g, cool.b);
  const glm::vec3 hot = FlameBlackbody::chromaticity(3500.0f, 2000.0f, 1.0f);
  EXPECT_GT(hot.b, hot.r);

  // Degree 0 (or no white) is the unadapted colour; partial adaptation lies in between.
  const glm::vec3 none = FlameBlackbody::chromaticity(1500.0f);
  const glm::vec3 d0 = FlameBlackbody::chromaticity(1500.0f, 2000.0f, 0.0f);
  EXPECT_NEAR(glm::length(none - d0), 0.0f, 1.0e-5f);
  const glm::vec3 half = FlameBlackbody::chromaticity(1500.0f, 1500.0f, 0.5f);
  const glm::vec3 full = FlameBlackbody::chromaticity(1500.0f, 1500.0f, 1.0f);
  EXPECT_LT(half.r / half.g, none.r / none.g);
  EXPECT_GT(half.r / half.g, full.r / full.g);
}
