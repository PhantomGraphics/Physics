#include "pch.h"
#include <set>
#include <utility>
#include "../Physics/SoftMesh.h"

using namespace Phantom::Physics;

// ------------------------------------------------- generateChain -------------

TEST(SoftMeshTest, GenerateChain_ParticleCount) {
    SoftMesh mesh;
    mesh.generateChain(10, 2.f);
    EXPECT_EQ(static_cast<int>(mesh.particles.size()), 10);
}

TEST(SoftMeshTest, GenerateChain_EdgeCount) {
    SoftMesh mesh;
    mesh.generateChain(10, 2.f);
    EXPECT_EQ(static_cast<int>(mesh.edges.size()), 9);
}

TEST(SoftMeshTest, GenerateChain_NumStructuralEdges) {
    SoftMesh mesh;
    mesh.generateChain(10, 2.f);
    EXPECT_EQ(mesh.numStructuralEdges, static_cast<int>(mesh.edges.size()));
}

TEST(SoftMeshTest, GenerateChain_RestLengthCorrect) {
    SoftMesh mesh;
    mesh.generateChain(5, 4.f);    // segLen = 1.0
    for (const auto& e : mesh.edges)
        EXPECT_NEAR(e.restLength, 1.f, 1e-5f);
}

TEST(SoftMeshTest, GenerateChain_InitialPositionsDescending) {
    SoftMesh mesh;
    mesh.generateChain(4, 3.f);
    for (int i = 0; i < 3; ++i)
        EXPECT_LT(mesh.particles.positions[i + 1].y, mesh.particles.positions[i].y);
}

// ------------------------------------------------- generateGrid  -------------

TEST(SoftMeshTest, GenerateGrid_ParticleCount) {
    SoftMesh mesh;
    mesh.generateGrid(5, 6, 2.f, 2.f);
    EXPECT_EQ(static_cast<int>(mesh.particles.size()), 5 * 6);
}

TEST(SoftMeshTest, GenerateGrid_NumStructuralEdges) {
    int rows = 5, cols = 6;
    SoftMesh mesh;
    mesh.generateGrid(rows, cols, 2.f, 2.f);
    int expected = rows * (cols - 1) + (rows - 1) * cols;
    EXPECT_EQ(mesh.numStructuralEdges, expected);
}

TEST(SoftMeshTest, GenerateGrid_TotalEdgesIncludeDiagonals) {
    int rows = 5, cols = 6;
    SoftMesh mesh;
    mesh.generateGrid(rows, cols, 2.f, 2.f);
    int structural = rows * (cols - 1) + (rows - 1) * cols;
    int diagonals  = (rows - 1) * (cols - 1) * 2;
    EXPECT_EQ(static_cast<int>(mesh.edges.size()), structural + diagonals);
}

TEST(SoftMeshTest, GenerateGrid_FaceCount) {
    int rows = 5, cols = 6;
    SoftMesh mesh;
    mesh.generateGrid(rows, cols, 2.f, 2.f);
    EXPECT_EQ(static_cast<int>(mesh.faces.size()), (rows - 1) * (cols - 1) * 2);
}

// ------------------------------------------------- generateTetBox ------------

TEST(SoftMeshTest, GenerateTetBox_ParticleCount) {
    int nx = 2, ny = 2, nz = 2;
    SoftMesh mesh;
    mesh.generateTetBox(nx, ny, nz, 1.f, 1.f, 1.f);
    EXPECT_EQ(static_cast<int>(mesh.particles.size()), (nx + 1) * (ny + 1) * (nz + 1));
}

TEST(SoftMeshTest, GenerateTetBox_TetraCount) {
    int nx = 2, ny = 2, nz = 2;
    SoftMesh mesh;
    mesh.generateTetBox(nx, ny, nz, 1.f, 1.f, 1.f);
    EXPECT_EQ(static_cast<int>(mesh.tetrahedra.size()), nx * ny * nz * 5);
}

TEST(SoftMeshTest, GenerateTetBox_RestVolumeNonZero) {
    // Signed volume — negative values are valid for opposite-winding tetras
    SoftMesh mesh;
    mesh.generateTetBox(2, 2, 2, 1.f, 1.f, 1.f);
    for (const auto& t : mesh.tetrahedra)
        EXPECT_GT(std::abs(t.restVolume), 0.f);
}

TEST(SoftMeshTest, GenerateTetBox_EdgesDeduplicated) {
    // 2x2x2 セル = 3x3x3 頂点の完全グラフではなく、実際に使われる辺のみ
    SoftMesh mesh;
    mesh.generateTetBox(2, 2, 2, 1.f, 1.f, 1.f);
    EXPECT_GT(static_cast<int>(mesh.edges.size()), 0);

    std::set<std::pair<int, int>> unique;
    for (const auto& e : mesh.edges) {
        auto key = e.a < e.b ? std::make_pair(e.a, e.b) : std::make_pair(e.b, e.a);
        EXPECT_TRUE(unique.insert(key).second) << "duplicate edge found";
    }
}

TEST(SoftMeshTest, GenerateTetBox_NumStructuralEdgesMatchesEdges) {
    SoftMesh mesh;
    mesh.generateTetBox(2, 2, 2, 1.f, 1.f, 1.f);
    EXPECT_EQ(mesh.numStructuralEdges, static_cast<int>(mesh.edges.size()));
}
