#include "pch.h"

#include "../Physics/AnisotropicKernel.h"
#include "../Physics/SPHSurfaceParticle.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace
{
// Cubic lattice of (2n+1)^3 points spaced `h` apart, centred on the origin.
std::vector<Vector3df> lattice(int n, float h, Vector3df offset = Vector3df(0.0f))
{
    std::vector<Vector3df> points;
    for (int z = -n; z <= n; ++z)
        for (int y = -n; y <= n; ++y)
            for (int x = -n; x <= n; ++x)
                points.push_back(offset + h * Vector3df(float(x), float(y), float(z)));
    return points;
}

// Single-layer square sheet in the XZ plane.
std::vector<Vector3df> sheet(int n, float h)
{
    std::vector<Vector3df> points;
    for (int z = -n; z <= n; ++z)
        for (int x = -n; x <= n; ++x)
            points.push_back(h * Vector3df(float(x), 0.0f, float(z)));
    return points;
}

int indexOf(const std::vector<Vector3df>& points, const Vector3df& p)
{
    for (int i = 0; i < static_cast<int>(points.size()); ++i)
        if (glm::length(points[i] - p) < 1.0e-6f) return i;
    return -1;
}

// T T^T is independent of the sign/order ambiguity of the eigenvectors.
Matrix3df gram(const Matrix3df& axes) { return axes * glm::transpose(axes); }
} // namespace

TEST(AnisotropicKernelTest, EmptyInputGivesEmptyResult)
{
    AnisotropyResult result;
    computeAnisotropy({}, AnisotropyParams{}, result);
    EXPECT_TRUE(result.centers.empty());
    EXPECT_TRUE(result.axes.empty());
    EXPECT_EQ(result.anisotropicCount, 0u);
}

TEST(AnisotropicKernelTest, IsolatedParticleIsSphereOfIsolatedScale)
{
    AnisotropyParams params;
    params.searchRadius = 1.0f;
    AnisotropyResult result;
    computeAnisotropy({ Vector3df(1.0f, 2.0f, 3.0f) }, params, result);

    ASSERT_EQ(result.axes.size(), 1u);
    const Matrix3df g = gram(result.axes[0]);
    const float kn2 = params.isolatedScale * params.isolatedScale;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_NEAR(g[c][r], r == c ? kn2 : 0.0f, 1.0e-5f);
    // Alone, the weighted mean is the particle itself.
    EXPECT_NEAR(glm::length(result.centers[0] - Vector3df(1.0f, 2.0f, 3.0f)), 0.0f, 1.0e-6f);
    EXPECT_EQ(result.anisotropicCount, 0u);
}

TEST(AnisotropicKernelTest, UniformLatticeInteriorIsUnitSphere)
{
    const float h = 0.1f;
    const auto points = lattice(4, h);
    AnisotropyParams params;
    params.searchRadius = 2.5f * h;
    AnisotropyResult result;
    computeAnisotropy(points, params, result);

    const int centre = indexOf(points, Vector3df(0.0f));
    ASSERT_GE(centre, 0);
    const Matrix3df g = gram(result.axes[centre]);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            EXPECT_NEAR(g[c][r], r == c ? 1.0f : 0.0f, 1.0e-4f);
    EXPECT_NEAR(glm::length(result.centers[centre]), 0.0f, 1.0e-5f);
}

TEST(AnisotropicKernelTest, SheetIsFlattenedAlongItsNormal)
{
    const float h = 0.1f;
    const auto points = sheet(6, h);
    AnisotropyParams params;
    params.searchRadius = 3.5f * h; // enough in-plane neighbours to trust the PCA
    AnisotropyResult result;
    computeAnisotropy(points, params, result);

    const int centre = indexOf(points, Vector3df(0.0f));
    ASSERT_GE(centre, 0);
    const Matrix3df g = gram(result.axes[centre]);
    const float normalStretch = std::sqrt(g[1][1]);  // along Y (sheet normal)
    const float inPlaneStretch = std::sqrt(g[0][0]); // along X
    // The zero-variance normal is clamped to sigma_max / k_r.
    EXPECT_NEAR(inPlaneStretch / normalStretch, params.maxRatio, 1.0e-3f);
    EXPECT_NEAR(std::sqrt(g[2][2]), inPlaneStretch, 1.0e-4f);
    // Unit product (volume preserving).
    EXPECT_NEAR(inPlaneStretch * inPlaneStretch * normalStretch, 1.0f, 1.0e-4f);
    EXPECT_GT(result.meanStretchRatio, 1.0f);
}

TEST(AnisotropicKernelTest, IsScaleInvariant)
{
    const auto small = sheet(5, 0.01f);
    std::vector<Vector3df> large;
    for (const auto& p : small) large.push_back(p * 100.0f);

    AnisotropyParams ps;
    ps.searchRadius = 0.035f;
    AnisotropyParams pl = ps;
    pl.searchRadius = 3.5f;

    AnisotropyResult rs, rl;
    computeAnisotropy(small, ps, rs);
    computeAnisotropy(large, pl, rl);
    ASSERT_EQ(rs.axes.size(), rl.axes.size());
    for (size_t i = 0; i < rs.axes.size(); ++i) {
        const Matrix3df gs = gram(rs.axes[i]);
        const Matrix3df gl = gram(rl.axes[i]);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                EXPECT_NEAR(gs[c][r], gl[c][r], 1.0e-3f);
        EXPECT_NEAR(glm::length(rs.centers[i] * 100.0f - rl.centers[i]), 0.0f, 1.0e-3f);
    }
}

TEST(AnisotropicKernelTest, IsDeterministic)
{
    // Jittered lattice so the PCA is non-trivial.
    auto points = lattice(5, 0.1f);
    for (size_t i = 0; i < points.size(); ++i)
        points[i] += 0.02f * Vector3df(std::sin(float(i) * 1.7f), std::cos(float(i) * 2.3f), std::sin(float(i) * 0.9f));
    AnisotropyParams params;
    params.searchRadius = 0.25f;

    AnisotropyResult a, b;
    computeAnisotropy(points, params, a);
    computeAnisotropy(points, params, b);
    ASSERT_EQ(a.axes.size(), b.axes.size());
    for (size_t i = 0; i < a.axes.size(); ++i) {
        EXPECT_EQ(a.centers[i], b.centers[i]);
        EXPECT_EQ(a.axes[i], b.axes[i]);
    }
    EXPECT_EQ(a.meanStretchRatio, b.meanStretchRatio);
}

TEST(AnisotropicKernelTest, SmoothingZeroKeepsPositions)
{
    const auto points = sheet(4, 0.1f);
    AnisotropyParams params;
    params.searchRadius = 0.35f;
    params.smoothing = 0.0f;
    AnisotropyResult result;
    computeAnisotropy(points, params, result);
    for (size_t i = 0; i < points.size(); ++i)
        EXPECT_EQ(result.centers[i], points[i]);
}

TEST(AnisotropicKernelTest, MatchesSPHSurfaceParticleMatrix)
{
    // The volume converter's per-particle matrix G must be the inverse of the
    // ellipsoid gram: G = R diag(1/sigma) R^T, T T^T = R diag(sigma^2) R^T.
    auto points = sheet(6, 0.1f);
    for (size_t i = 0; i < points.size(); ++i)
        points[i].y += 0.01f * std::sin(float(i) * 1.3f);
    const float radius = 0.35f;
    AnisotropyParams params;
    params.searchRadius = radius;
    AnisotropyResult result;
    computeAnisotropy(points, params, result);

    for (int i : { 0, static_cast<int>(points.size() / 2), static_cast<int>(points.size() - 1) }) {
        std::vector<Vector3df> neighbors;
        for (const auto& q : points)
            if (glm::length(q - points[i]) < radius) neighbors.push_back(q);

        SPHSurfaceParticle particle(points[i], radius);
        particle.calculateAnisotoropicMatrix(neighbors, radius);
        const Matrix3dd G = particle.getMatrix();
        const Matrix3dd T = Matrix3dd(result.axes[i]);
        // G = R diag(1/sigma) R^T and T = R diag(sigma), so G * T = R is
        // orthonormal exactly when both come from the same frame.
        const Matrix3dd GT = G * T;
        const Matrix3dd shouldBeI = glm::transpose(GT) * GT;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                EXPECT_NEAR(shouldBeI[c][r], r == c ? 1.0 : 0.0, 1.0e-3) << "particle " << i;
    }
}
