#include "pch.h"

#include "../Physics/SPHVolumeConverter.h"
#include "CGLib/Volume/Volume/SparseVolumeTree/SparseVolume.h"

#include <cmath>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Volume;
using namespace Phantom::Physics;

// ---- Helpers ----------------------------------------------------------------

namespace
{
// World position of the voxel that contains the given world coordinate.
Coord worldToIndex(const SparseVolumef& sv, float x, float y, float z)
{
    return sv.worldToIndex(Vector3df(x, y, z));
}
} // namespace

// ---- Empty input ------------------------------------------------------------

TEST(SPHVolumeConverterTest, IsotropicEmptyInputReturnsNull)
{
    SPHVolumeConverter conv;
    auto sv = conv.buildIsotoropic(std::vector<Vector3dd>{}, 0.5f, 0.25f);
    EXPECT_EQ(sv, nullptr);
}

// ---- Single particle --------------------------------------------------------

TEST(SPHVolumeConverterTest, IsotropicSingleParticleCenterVoxelPositive)
{
    // One particle at the origin with radius 1.0, voxel size 0.5.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildIsotoropic(positions, 1.0f, 0.5f);

    ASSERT_NE(sv, nullptr);
    // The voxel at the particle center must accumulate a positive kernel value.
    EXPECT_GT(sv->getValue(Coord(0, 0, 0)), 0.0f);
}

TEST(SPHVolumeConverterTest, IsotropicSingleParticleActivatesVoxels)
{
    // With radius=1.0 and cellLength=0.5, voxels within distance < 1.0 become
    // active (distance == 1.0 gives W_cubic = 0 at the support boundary).
    // Voxels within L-inf halfCells=2 and L2 dist < 1.0:
    //   (0,0,0): d=0,          active
    //   (±1,0,0) etc: d=0.5,   active   -> 6 voxels
    //   (±1,±1,0) etc: d≈0.71, active   -> 12 voxels
    //   (±1,±1,±1): d≈0.87,    active   -> 8 voxels
    // Total = 27 voxels.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildIsotoropic(positions, 1.0f, 0.5f);

    ASSERT_NE(sv, nullptr);
    EXPECT_EQ(sv->getActiveVoxelCount(), 27);
}

TEST(SPHVolumeConverterTest, IsotropicVoxelBeyondRadiusIsBackground)
{
    // Voxel at index (3, 0, 0) has world position (1.5, 0, 0).
    // Distance from the origin particle = 1.5 > particleRadius=1.0 -> background.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildIsotoropic(positions, 1.0f, 0.5f);

    ASSERT_NE(sv, nullptr);
    EXPECT_EQ(sv->getValue(Coord(3, 0, 0)), sv->getBackground());
}

// ---- Decay with distance ----------------------------------------------------

TEST(SPHVolumeConverterTest, IsotropicValueDecaysWithDistance)
{
    // The cubic-spline kernel is monotonically decreasing.
    // The voxel at (1, 0, 0) (dist=0.5) must be strictly less than the center.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildIsotoropic(positions, 1.0f, 0.5f);

    ASSERT_NE(sv, nullptr);
    const float vCenter = sv->getValue(Coord(0, 0, 0));   // dist = 0.0
    const float vNear   = sv->getValue(Coord(1, 0, 0));   // dist = 0.5
    EXPECT_GT(vCenter, vNear);
    EXPECT_GT(vNear,   0.0f);
}

// ---- Multiple particles accumulate ------------------------------------------

TEST(SPHVolumeConverterTest, IsotropicTwoParticlesAccumulateAtOverlap)
{
    // Two particles: one at origin, one at (0.5, 0, 0).
    // The second particle's center voxel (1, 0, 0) [world (0.5,0,0)] lies
    // inside the first particle's support (dist=0.5 < 1.0), so contributions
    // from both particles stack.
    // Compare with a single-particle run: the overlap voxel value must be higher.
    const float radius     = 1.0f;
    const float cellLength = 0.5f;

    SPHVolumeConverter convTwo;
    auto svTwo = convTwo.buildIsotoropic(
        std::vector<Vector3dd>{{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, radius, cellLength);
    ASSERT_NE(svTwo, nullptr);

    SPHVolumeConverter convOne;
    auto svOne = convOne.buildIsotoropic(std::vector<Vector3dd>{{0.0, 0.0, 0.0}}, radius, cellLength);
    ASSERT_NE(svOne, nullptr);

    // The voxel at (1, 0, 0) sits at the center of the second particle and
    // within the support of the first -> must receive contributions from both.
    const Coord overlapVoxel(1, 0, 0);
    EXPECT_GT(svTwo->getValue(overlapVoxel), svOne->getValue(overlapVoxel));
}

// ---- Converter can be reused ------------------------------------------------

TEST(SPHVolumeConverterTest, IsotropicReusedConverterGivesFreshResult)
{
    // Calling buildIsotoropic twice on the same instance must not accumulate
    // state from the first call.
    const std::vector<Vector3dd> posA = {{0.0, 0.0, 0.0}};
    const std::vector<Vector3dd> posB = {{0.0, 0.0, 0.0}};

    SPHVolumeConverter conv;
    auto svA = conv.buildIsotoropic(posA, 1.0f, 0.5f);
    auto svB = conv.buildIsotoropic(posB, 1.0f, 0.5f);

    ASSERT_NE(svA, nullptr);
    ASSERT_NE(svB, nullptr);

    // Both runs used an identical single particle, so the center voxel values
    // must match (no stale density accumulation from the first call).
    EXPECT_FLOAT_EQ(svA->getValue(Coord(0, 0, 0)),
                    svB->getValue(Coord(0, 0, 0)));
}

// ============================================================================
// Anisotropic kernel tests
// ============================================================================
//
// For a single particle in isolation the neighbourhood count is 1 (self only),
// which falls below the 25-neighbour threshold in calculateAnisotoropicMatrix.
// The fallback sets scaleMatrix = 0.5 * I, yielding G = 2 * I.  G is
// dimensionless: |Gv| is a world-space distance compared against the kernel
// support h = particleRadius.  All geometric expectations below are derived
// from this known G.
//
// Effective world-space support with G = 2*I:
//   |Gv| = 2*d < h  ⟹  d < h/2
// ============================================================================

// ---- Empty input ------------------------------------------------------------

TEST(SPHVolumeConverterTest, AnisotropicEmptyInputReturnsNull)
{
    SPHVolumeConverter conv;
    auto sv = conv.buildAnisotoropic(std::vector<Vector3dd>{}, 0.5f, 0.25f);
    EXPECT_EQ(sv, nullptr);
}

// ---- Single particle --------------------------------------------------------

TEST(SPHVolumeConverterTest, AnisotropicSingleParticleCenterVoxelPositive)
{
    // G = 2*I at origin: the kernel is symmetric but compressed to d < h/2.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildAnisotoropic(positions, 1.0f, 0.25f);

    ASSERT_NE(sv, nullptr);
    EXPECT_GT(sv->getValue(Coord(0, 0, 0)), 0.0f);
}

TEST(SPHVolumeConverterTest, AnisotropicSingleParticleActivatesVoxels)
{
    // With G = 2*I/h (h=1, cellLength=0.25), the effective world-space support
    // is d < 0.5.  Active voxels (same count as isotropic with half the radius):
    //   (0,0,0):d=0, (±1,0,0)…:d=0.25, (±1,±1,0)…:d≈0.35, (±1,±1,±1):d≈0.43
    //   Total = 1 + 6 + 12 + 8 = 27  (d=0.5 gives W=0, skipped).
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildAnisotoropic(positions, 1.0f, 0.25f);

    ASSERT_NE(sv, nullptr);
    EXPECT_EQ(sv->getActiveVoxelCount(), 27);
}

// ---- Background voxel -------------------------------------------------------

TEST(SPHVolumeConverterTest, AnisotropicVoxelBeyondSupportIsBackground)
{
    // Coord(3,0,0) → world (0.75,0,0); d=0.75 > h/2=0.5 → W_cubic(|Gv|)=0.
    // The w<=0 guard prevents this voxel from being written → background.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildAnisotoropic(positions, 1.0f, 0.25f);

    ASSERT_NE(sv, nullptr);
    EXPECT_EQ(sv->getValue(Coord(3, 0, 0)), sv->getBackground());
}

// ---- Decay with transformed distance ----------------------------------------

TEST(SPHVolumeConverterTest, AnisotropicValueDecaysWithDistance)
{
    // cellLength=0.1 gives several active voxels inside d<0.5.
    // Center voxel (|Gv|=0) must exceed the adjacent voxel (|Gv|=0.2).
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};
    SPHVolumeConverter conv;
    auto sv = conv.buildAnisotoropic(positions, 1.0f, 0.1f);

    ASSERT_NE(sv, nullptr);
    const float vCenter = sv->getValue(Coord(0, 0, 0));  // |Gv| = 0
    const float vNear   = sv->getValue(Coord(1, 0, 0));  // |Gv| = 0.2
    EXPECT_GT(vCenter, vNear);
    EXPECT_GT(vNear,   0.0f);
}

// ---- Anisotropic differs from isotropic -------------------------------------

TEST(SPHVolumeConverterTest, AnisotropicCenterVoxelDiffersFromIsotropic)
{
    // G = 2*I multiplies by |det(G)| = 2³ = 8, so the centre-voxel
    // contribution changes.  Verify the two modes produce distinct values.
    const std::vector<Vector3dd> positions = {{0.0, 0.0, 0.0}};

    SPHVolumeConverter convIso;
    auto svIso  = convIso.buildIsotoropic(positions, 1.0f, 0.25f);

    SPHVolumeConverter convAniso;
    auto svAniso = convAniso.buildAnisotoropic(positions, 1.0f, 0.25f);

    ASSERT_NE(svIso,   nullptr);
    ASSERT_NE(svAniso, nullptr);

    EXPECT_NE(svIso->getValue(Coord(0, 0, 0)),
              svAniso->getValue(Coord(0, 0, 0)));
}

// ---- Converter can be reused ------------------------------------------------

TEST(SPHVolumeConverterTest, AnisotropicReusedConverterGivesFreshResult)
{
    // Calling buildAnisotoropic twice must not accumulate particle state.
    const std::vector<Vector3dd> pos = {{0.0, 0.0, 0.0}};

    SPHVolumeConverter conv;
    auto svA = conv.buildAnisotoropic(pos, 1.0f, 0.25f);
    auto svB = conv.buildAnisotoropic(pos, 1.0f, 0.25f);

    ASSERT_NE(svA, nullptr);
    ASSERT_NE(svB, nullptr);

    EXPECT_FLOAT_EQ(svA->getValue(Coord(0, 0, 0)),
                    svB->getValue(Coord(0, 0, 0)));
}

// ---- Neighbourhood-driven anisotropy (>= 25 neighbours) ---------------------

namespace
{
// n^3 lattice with the given spacing, centred on `center`.
std::vector<Vector3df> makeLattice(int n, float spacing, const Vector3df& center)
{
    std::vector<Vector3df> pts;
    const float off = (n - 1) * 0.5f;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            for (int k = 0; k < n; ++k)
                pts.emplace_back(center + Vector3df(i - off, j - off, k - off) * spacing);
    return pts;
}

struct VolumeSummary
{
    int   activeCount = 0;
    float sum = 0.0f;
    float maxValue = 0.0f;
};

VolumeSummary summarize(const SparseVolumef& sv)
{
    VolumeSummary s;
    s.activeCount = sv.getActiveVoxelCount();
    sv.forEachActive([&](const Coord&, const Vector3df&, const float& v) {
        s.sum += v;
        s.maxValue = std::max(s.maxValue, v);
    });
    return s;
}
} // namespace

TEST(SPHVolumeConverterTest, AnisotropicLatticeIsDeterministic)
{
    // calculateAnisotropy() used to move each particle to its smoothed centre
    // inside the OpenMP loop while other threads read it as a neighbour, so the
    // same input produced a different volume on every run.
    const auto pts = makeLattice(10, 0.02f, Vector3df(-0.03f, 0.05f, -0.08f));

    SPHVolumeConverter convA, convB;
    auto svA = convA.buildAnisotoropic(pts, 0.04f, 0.01f);
    auto svB = convB.buildAnisotoropic(pts, 0.04f, 0.01f);
    ASSERT_NE(svA, nullptr);
    ASSERT_NE(svB, nullptr);

    ASSERT_EQ(svA->getActiveVoxelCount(), svB->getActiveVoxelCount());
    int mismatches = 0;
    svA->forEachActive([&](const Coord& c, const Vector3df&, const float& v) {
        if (svB->getValue(c) != v)
            ++mismatches;
    });
    EXPECT_EQ(mismatches, 0);
}

TEST(SPHVolumeConverterTest, AnisotropicUniformLatticeMatchesIsotropicScale)
{
    // Inside a uniform lattice the weighted covariance is isotropic, so the
    // normalized stretch is 1 on every axis and G = I: the anisotropic volume
    // must cover the same region with the same magnitude as the isotropic one.
    // Before the fix the unnormalized covariance (~spacing²) made |Gv| ~1e4x
    // too large -- the kernel collapsed to ~2% of the isotropic voxel count and
    // det(G) blew individual voxels up to ~1e14.
    const auto pts = makeLattice(10, 0.02f, Vector3df(0.0f, 0.0f, 0.0f));

    SPHVolumeConverter convIso, convAniso;
    auto svIso   = convIso.buildIsotoropic(pts, 0.04f, 0.01f);
    auto svAniso = convAniso.buildAnisotoropic(pts, 0.04f, 0.01f);
    ASSERT_NE(svIso, nullptr);
    ASSERT_NE(svAniso, nullptr);

    const auto iso   = summarize(*svIso);
    const auto aniso = summarize(*svAniso);
    EXPECT_GT(aniso.activeCount, iso.activeCount / 2);
    EXPECT_LT(aniso.activeCount, iso.activeCount * 2);
    // det(G) * W(|Gv|) integrates to 1 for any G, so the total volume fraction
    // is conserved up to voxel sampling (the old G gave ~1e14 here).
    EXPECT_NEAR(aniso.sum, iso.sum, iso.sum * 0.25f);

    // Deep inside the lattice G = I, so the value matches the isotropic one.
    // (The per-voxel maximum is not compared: corner/edge particles have < 25
    // neighbours and use the compact k_n = 0.5 kernel, which peaks 8x higher.)
    const float vIso   = svIso->getValue(Coord(0, 0, 0));
    const float vAniso = svAniso->getValue(Coord(0, 0, 0));
    EXPECT_GT(vIso, 0.0f);
    EXPECT_NEAR(vAniso, vIso, vIso * 0.1f);
}

TEST(SPHVolumeConverterTest, AnisotropicIsScaleInvariant)
{
    // G must be dimensionless: scaling the whole scene (positions, radius and
    // voxel size) by the same factor must reproduce the same voxel pattern.
    // The old G carried a 1/searchRadius factor plus the covariance's
    // spacing² units, so the kernel shape depended on the scene's scale.
    constexpr float s = 100.0f;
    const auto small = makeLattice(8, 0.02f, Vector3df(0.0f, 0.0f, 0.0f));
    const auto large = makeLattice(8, 0.02f * s, Vector3df(0.0f, 0.0f, 0.0f));

    SPHVolumeConverter convS, convL;
    auto svS = convS.buildAnisotoropic(small, 0.04f,     0.01f);
    auto svL = convL.buildAnisotoropic(large, 0.04f * s, 0.01f * s);
    ASSERT_NE(svS, nullptr);
    ASSERT_NE(svL, nullptr);

    const auto a = summarize(*svS);
    const auto b = summarize(*svL);
    // Voxel counts can differ by a handful of support-boundary voxels from
    // float rounding. Each voxel value W*m/rho is a dimensionless volume
    // fraction (W ~ 1/s³, m ~ s³, rho invariant), so values match directly.
    EXPECT_NEAR(static_cast<float>(b.activeCount), static_cast<float>(a.activeCount), a.activeCount * 0.01f);
    EXPECT_NEAR(b.sum, a.sum, a.sum * 1.0e-3f);
    EXPECT_NEAR(b.maxValue, a.maxValue, a.maxValue * 1.0e-3f);
}

// ============================================================================
// float overload tests
// ============================================================================

// ---- Empty input ------------------------------------------------------------

TEST(SPHVolumeConverterTest, IsotropicFloatEmptyInputReturnsNull)
{
    SPHVolumeConverter conv;
    EXPECT_EQ(conv.buildIsotoropic(std::vector<Vector3df>{}, 0.5f, 0.25f), nullptr);
}

TEST(SPHVolumeConverterTest, AnisotropicFloatEmptyInputReturnsNull)
{
    SPHVolumeConverter conv;
    EXPECT_EQ(conv.buildAnisotoropic(std::vector<Vector3df>{}, 0.5f, 0.25f), nullptr);
}

// ---- float and double overloads produce identical results -------------------

TEST(SPHVolumeConverterTest, IsotropicFloatAndDoubleGiveSameResult)
{
    // A single particle at the origin: both overloads must agree on voxel
    // values and active voxel count (float cast is exact for zero and small
    // integers, so bit-exact equality is expected here).
    const std::vector<Vector3df> posf = { {0.0f, 0.0f, 0.0f} };
    const std::vector<Vector3dd> posd = { {0.0,  0.0,  0.0 } };

    SPHVolumeConverter convF, convD;
    auto svF = convF.buildIsotoropic(posf, 1.0f, 0.5f);
    auto svD = convD.buildIsotoropic(posd, 1.0f, 0.5f);

    ASSERT_NE(svF, nullptr);
    ASSERT_NE(svD, nullptr);
    EXPECT_EQ(svF->getActiveVoxelCount(), svD->getActiveVoxelCount());
    EXPECT_FLOAT_EQ(svF->getValue(Coord(0, 0, 0)), svD->getValue(Coord(0, 0, 0)));
    EXPECT_FLOAT_EQ(svF->getValue(Coord(1, 0, 0)), svD->getValue(Coord(1, 0, 0)));
}

TEST(SPHVolumeConverterTest, AnisotropicFloatAndDoubleGiveSameResult)
{
    const std::vector<Vector3df> posf = { {0.0f, 0.0f, 0.0f} };
    const std::vector<Vector3dd> posd = { {0.0,  0.0,  0.0 } };

    SPHVolumeConverter convF, convD;
    auto svF = convF.buildAnisotoropic(posf, 1.0f, 0.25f);
    auto svD = convD.buildAnisotoropic(posd, 1.0f, 0.25f);

    ASSERT_NE(svF, nullptr);
    ASSERT_NE(svD, nullptr);
    EXPECT_EQ(svF->getActiveVoxelCount(), svD->getActiveVoxelCount());
    EXPECT_FLOAT_EQ(svF->getValue(Coord(0, 0, 0)), svD->getValue(Coord(0, 0, 0)));
}
