#pragma once

#include <vector>
#include "SoftParticle.h"

namespace Phantom {
namespace Physics {

struct SoftEdge {
    int   a, b;
    float restLength;
};

struct SoftFace {
    int a, b, c;
};

struct SoftTetra {
    int   a, b, c, d;
    float restVolume;
};

class SoftMesh {
public:
    // 1D: n 個のセグメントから成るチェーン（RopeBody 用）
    void generateChain(int n, float length);

    // 2D: rows×cols のグリッド（ClothBody 用）
    void generateGrid(int rows, int cols, float width, float height);

    // 3D: nx×ny×nz の直方体テトラ分割（JellyBody 用）
    //     各六面体セルを 5 テトラに分解
    void generateTetBox(int nx, int ny, int nz, float w, float h, float d);

    SoftParticleSoA           particles;
    std::vector<SoftEdge>     edges;              // structural + bending edges
    std::vector<SoftFace>     faces;              // triangles
    std::vector<SoftTetra>    tetrahedra;         // volume constraint tetras
    int                       numStructuralEdges = 0; // edges[0..n) = structural
};

} // namespace Physics
} // namespace Phantom
