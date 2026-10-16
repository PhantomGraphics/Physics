#include "SoftMesh.h"

#include <set>
#include <utility>

namespace Phantom {
namespace Physics {

void SoftMesh::generateChain(int n, float length) {
    particles.clear();
    edges.clear();
    faces.clear();
    tetrahedra.clear();

    float segLen = (n > 1) ? length / (n - 1) : 0.f;
    for (int i = 0; i < n; ++i) {
        SoftParticle p;
        p.position    = { 0.f, -i * segLen, 0.f };
        p.predicted   = p.position;
        p.inverseMass = 1.f;
        particles.push_back(p);
    }
    for (int i = 0; i < n - 1; ++i)
        edges.push_back({ i, i + 1, segLen });

    numStructuralEdges = static_cast<int>(edges.size());
}

void SoftMesh::generateGrid(int rows, int cols, float width, float height) {
    particles.clear();
    edges.clear();
    faces.clear();
    tetrahedra.clear();

    float dx = (cols > 1) ? width  / (cols - 1) : 0.f;
    float dz = (rows > 1) ? height / (rows - 1) : 0.f;

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            SoftParticle p;
            p.position    = { c * dx - width * 0.5f, 0.f, r * dz - height * 0.5f };
            p.predicted   = p.position;
            p.inverseMass = 1.f;
            particles.push_back(p);
        }
    }

    auto idx = [cols](int r, int c) { return r * cols + c; };

    // 横・縦の structural エッジ
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols - 1; ++c) {
            int a = idx(r, c), b = idx(r, c + 1);
            edges.push_back({ a, b, glm::length(particles.positions[b] - particles.positions[a]) });
        }
    for (int r = 0; r < rows - 1; ++r)
        for (int c = 0; c < cols; ++c) {
            int a = idx(r, c), b = idx(r + 1, c);
            edges.push_back({ a, b, glm::length(particles.positions[b] - particles.positions[a]) });
        }

    numStructuralEdges = static_cast<int>(edges.size());

    // 曲げ用対角エッジ
    for (int r = 0; r < rows - 1; ++r)
        for (int c = 0; c < cols - 1; ++c) {
            int a = idx(r, c),     b = idx(r + 1, c + 1);
            int c1 = idx(r, c + 1), d = idx(r + 1, c);
            edges.push_back({ a, b, glm::length(particles.positions[b]  - particles.positions[a]) });
            edges.push_back({ c1, d, glm::length(particles.positions[d] - particles.positions[c1]) });
        }

    // 三角形フェース
    for (int r = 0; r < rows - 1; ++r)
        for (int c = 0; c < cols - 1; ++c) {
            int tl = idx(r, c), tr = idx(r, c + 1);
            int bl = idx(r + 1, c), br = idx(r + 1, c + 1);
            faces.push_back({ tl, tr, bl });
            faces.push_back({ tr, br, bl });
        }
}

void SoftMesh::generateTetBox(int nx, int ny, int nz, float w, float h, float d) {
    particles.clear();
    edges.clear();
    faces.clear();
    tetrahedra.clear();

    float dx = (nx > 0) ? w / nx : w;
    float dy = (ny > 0) ? h / ny : h;
    float dz = (nz > 0) ? d / nz : d;

    auto idx3 = [&](int i, int j, int k) {
        return i * (ny + 1) * (nz + 1) + j * (nz + 1) + k;
    };

    for (int i = 0; i <= nx; ++i)
        for (int j = 0; j <= ny; ++j)
            for (int k = 0; k <= nz; ++k) {
                SoftParticle p;
                p.position    = { i * dx - w * 0.5f, j * dy, k * dz - d * 0.5f };
                p.predicted   = p.position;
                p.inverseMass = 1.f;
                particles.push_back(p);
            }

    // 各六面体セルを 5 テトラに分割
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < ny; ++j)
            for (int k = 0; k < nz; ++k) {
                int v[8] = {
                    idx3(i,   j,   k),   idx3(i+1, j,   k),
                    idx3(i+1, j,   k+1), idx3(i,   j,   k+1),
                    idx3(i,   j+1, k),   idx3(i+1, j+1, k),
                    idx3(i+1, j+1, k+1), idx3(i,   j+1, k+1)
                };

                // 5-tetra decomposition (consistent handedness)
                int tetras[5][4] = {
                    { v[0], v[1], v[3], v[4] },
                    { v[1], v[2], v[3], v[6] },
                    { v[4], v[5], v[6], v[1] },
                    { v[4], v[6], v[7], v[3] },
                    { v[1], v[3], v[4], v[6] }
                };

                for (auto& t : tetras) {
                    const auto& pa = particles.positions[t[0]];
                    const auto& pb = particles.positions[t[1]];
                    const auto& pc = particles.positions[t[2]];
                    const auto& pd = particles.positions[t[3]];
                    float vol = glm::dot(pb - pa, glm::cross(pc - pa, pd - pa)) / 6.f;
                    tetrahedra.push_back({ t[0], t[1], t[2], t[3], vol });
                }
            }

    // 全テトラの辺を重複なく抽出（JellyBody の DistanceConstraint / ワイヤ描画用）
    std::set<std::pair<int, int>> edgeSet;
    auto addEdge = [&](int i, int j) {
        edgeSet.insert(i < j ? std::make_pair(i, j) : std::make_pair(j, i));
    };
    for (const auto& t : tetrahedra) {
        addEdge(t.a, t.b); addEdge(t.a, t.c); addEdge(t.a, t.d);
        addEdge(t.b, t.c); addEdge(t.b, t.d); addEdge(t.c, t.d);
    }
    for (const auto& e : edgeSet) {
        float len = glm::length(particles.positions[e.second] - particles.positions[e.first]);
        edges.push_back({ e.first, e.second, len });
    }
    numStructuralEdges = static_cast<int>(edges.size());
}

} // namespace Physics
} // namespace Phantom
