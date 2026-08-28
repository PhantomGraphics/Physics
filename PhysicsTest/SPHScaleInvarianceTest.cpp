#include "pch.h"

#include "../Physics/WCSPHFluid.h"
#include "../Physics/WCSPHParticle.h"
#include "../Physics/WCSPHSolver.h"

#include <cmath>
#include <limits>

using namespace Phantom::Math;
using namespace Phantom::Physics;

// docs/todo/PLAN_sph_scale_invariance.md Phase 8: a dedicated end-to-end
// check that the scale-invariance work from Phases 1-7 actually composes --
// running the *same* relative scenario (a small particle block settling
// under gravity inside a box) at two different length scales should produce
// comparably small relative density error, provided the scale-dependent
// parameters that the plan made auto-derivable (WCSPH's pressureCoe via
// WCSPHFluid::setPressureCoeFromScale(), Phase 1) or documented as needing
// proportional rescaling (effectLength, timestep; Physics/CLAUDE.md's "SPH
// の長さ単位..." convention note, Phase 7) are in fact rescaled together.
// Gravity is kept fixed across scales -- matching Phase 0's own measurement
// methodology (section 4.1: "stiffness・restDensity・重力は固定したまま") --
// since this models the same real-world gravity acting on a physically
// smaller fluid volume, not a change of length unit.

namespace
{
bool isFinite3(const Vector3df& v)
{
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Runs the pool-settling scenario at `radiusScale` (1.0 = the codebase's
// conventional default scale) and returns the mean relative density error
// across all particles after settling, or +inf if any particle ever went
// non-finite.
float runPoolAndGetMeanRelativeDensityError(const float radiusScale)
{
  const float radius = 0.05f * radiusScale;
  const float restDensity = 1000.0f;
  const float gravityY = -9.8f;
  // 4.0 * radius (== 2 * particle spacing) is this codebase's documented
  // support radius -- the value every preset uses via
  // SPHFluidItemProps::effect_length_factor, chosen because the lattice
  // density sum then lands on rest density (presets.py: "h = 2*spacing,
  // 近傍約33個", bulk lattice = 1009.8 for restDensity 1000).
  //
  // This used to read 2.25f * radius, which is only 1.125 * spacing. At that
  // support a particle's *self* density term alone is 1.10 * restDensity
  // (and the full lattice 1.16 *), so "relative density error" was dominated
  // by that discretization offset rather than by the compression this test
  // means to measure -- and it happened to offset the opposite error of walls
  // contributing no density at all, letting the assertion below pass for the
  // wrong reason. With the support corrected, the mean error is a real
  // measurement: it is 0.80 without WCSPHSolver::addBoundaryDensity()
  // and under the bound with it.
  const float effectLength = 4.0f * radius;
  const float dt = 0.005f * radiusScale;

  WCSPHFluid fluid;
  fluid.setDensity(restDensity);
  fluid.setVicosityCoe(0.05f);
  fluid.setEffectLength(effectLength);
  fluid.setStatic(false);
  // Phase 1: derive pressureCoe from the scene's scale (effectLength) instead
  // of reusing a fixed numeric pressureCoe across radiusScale values. The
  // 1960.0f default reproduces the same value the retired gravity/target-
  // density-error-ratio-based derivation gave at gravity=9.8, ratio=0.01.
  fluid.setPressureCoeFromScale();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        fluid.createParticle(
            Vector3df((i - 1) * radius * 2.0f, radius * 6.0f + j * radius * 2.0f, (k - 1) * radius * 2.0f),
            radius);
      }
    }
  }

  WCSPHSolver solver;
  solver.add(&fluid);
  solver.setExternalForce(Vector3df(0.0f, gravityY, 0.0f));
  solver.setTimeStep(dt);
  solver.setEffectLength(effectLength);
  const float half = radius * 10.0f;
  solver.setBoundary(Box3df(Vector3df(-half, -half, -half), Vector3df(half, half, half)), dt);

  for (int step = 0; step < 200; ++step) {
    solver.simulate(dt, 1);
    for (size_t i = 0; i < fluid.getParticles().size(); ++i) {
      if (!isFinite3(fluid.getParticles().positions[i]) || !isFinite3(fluid.getParticles().velocities[i])) {
        return std::numeric_limits<float>::infinity();
      }
    }
  }

  float sumRelativeError = 0.0f;
  for (size_t i = 0; i < fluid.getParticles().size(); ++i) {
    WCSPHParticle p(fluid.getParticles(), i, &fluid);
    sumRelativeError += std::fabs(p.getDensity() - restDensity) / restDensity;
  }
  return sumRelativeError / static_cast<float>(fluid.getParticles().size());
}
}

TEST(SPHScaleInvarianceTest, WCSPH_PoolStability_RelativeDensityErrorStaysBoundedAcrossScales)
{
  const float errAtDefaultScale = runPoolAndGetMeanRelativeDensityError(1.0f);
  const float errAtShrunkScale = runPoolAndGetMeanRelativeDensityError(0.1f);

  ASSERT_TRUE(std::isfinite(errAtDefaultScale)) << "default scale (radius=0.05) exploded";
  ASSERT_TRUE(std::isfinite(errAtShrunkScale)) << "shrunk scale (radius=0.005) exploded";

  // Neither scale should show wildly different relative compression -- both
  // must land in the same "reasonably incompressible" ballpark despite the
  // absolute pressureCoe differing by ~10x between the two runs (Phase 0
  // measured stiffness held fixed instead would have made the shrunk scale's
  // pressure acceleration diverge to ~10x gravity).
  EXPECT_LT(errAtDefaultScale, 0.5f);
  EXPECT_LT(errAtShrunkScale, 0.5f);
}
