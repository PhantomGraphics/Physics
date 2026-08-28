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
// The fallback sets scaleMatrix = 0.5 * I, yielding G = 2 * I / h (with h =
// particleRadius).  All geometric expectations below are derived from this
// known G.
//
// Effective world-space support with G = 2*I/h:
//   |Gv| = 2*d/h < h  ⟹  d < h²/2 = h/2  (when h = 1)
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
    // G = 2*I/h multiplies by |det(G)| = (2/h)³ = 8, so the centre-voxel
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
