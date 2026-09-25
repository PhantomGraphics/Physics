#include "pch.h"
#include "ConvexHullShape.h"
#include "ContactGeometry.h"

#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

#include <unordered_set>

namespace Phantom {
namespace Physics {

namespace {

using V = Math::Vector3df;

struct HullTri {
    int   v[3];
    V     n;
    float d = 0.f;
    bool  alive = true;
};

HullTri makeTri(const std::vector<V>& p, int a, int b, int c, const V& interior) {
    HullTri t{ { a, b, c } };
    V n = glm::cross(p[b] - p[a], p[c] - p[a]);
    const float len = glm::length(n);
    t.n = (len > 1e-30f) ? n / len : V(0.f, 1.f, 0.f);
    t.d = glm::dot(t.n, p[a]);
    if (glm::dot(t.n, interior) - t.d > 0.f) { // keep every normal pointing away from the interior
        std::swap(t.v[1], t.v[2]);
        t.n = -t.n;
        t.d = -t.d;
    }
    return t;
}

// Incremental 3D hull. `eps` is the visibility tolerance in the points' units.
bool incrementalHull(const std::vector<V>& pts, float eps, std::vector<HullTri>& out) {
    const int n = static_cast<int>(pts.size());
    if (n < 4) return false;

    // Initial tetrahedron from the farthest pair of axis extremes.
    int ext[6] = { 0, 0, 0, 0, 0, 0 };
    for (int i = 1; i < n; ++i)
        for (int ax = 0; ax < 3; ++ax) {
            if (pts[i][ax] < pts[ext[ax * 2]][ax])     ext[ax * 2] = i;
            if (pts[i][ax] > pts[ext[ax * 2 + 1]][ax]) ext[ax * 2 + 1] = i;
        }
    int i0 = ext[0], i1 = ext[1];
    float best = -1.f;
    for (int a = 0; a < 6; ++a)
        for (int b = a + 1; b < 6; ++b) {
            const float d = glm::length(pts[ext[a]] - pts[ext[b]]);
            if (d > best) { best = d; i0 = ext[a]; i1 = ext[b]; }
        }
    if (best <= eps) return false;

    int i2 = -1;
    best = eps;
    const V axis = glm::normalize(pts[i1] - pts[i0]);
    for (int i = 0; i < n; ++i) {
        const V r = pts[i] - pts[i0];
        const float d = glm::length(r - axis * glm::dot(r, axis));
        if (d > best) { best = d; i2 = i; }
    }
    if (i2 < 0) return false;

    int i3 = -1;
    best = eps;
    const V pn = glm::normalize(glm::cross(pts[i1] - pts[i0], pts[i2] - pts[i0]));
    for (int i = 0; i < n; ++i) {
        const float d = std::abs(glm::dot(pn, pts[i] - pts[i0]));
        if (d > best) { best = d; i3 = i; }
    }
    if (i3 < 0) return false;

    const V interior = (pts[i0] + pts[i1] + pts[i2] + pts[i3]) * 0.25f;
    std::vector<HullTri> tris = {
        makeTri(pts, i0, i1, i2, interior), makeTri(pts, i0, i1, i3, interior),
        makeTri(pts, i0, i2, i3, interior), makeTri(pts, i1, i2, i3, interior),
    };

    auto key = [](int a, int b) {
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) | static_cast<uint32_t>(b);
    };
    std::unordered_set<uint64_t> edgeSet;
    std::vector<std::pair<int, int>> visibleEdges;
    for (int k = 0; k < n; ++k) {
        if (k == i0 || k == i1 || k == i2 || k == i3) continue;
        edgeSet.clear();
        visibleEdges.clear();
        for (auto& t : tris) {
            if (!t.alive || glm::dot(t.n, pts[k]) - t.d <= eps) continue;
            t.alive = false;
            for (int e = 0; e < 3; ++e) {
                const int a = t.v[e], b = t.v[(e + 1) % 3];
                edgeSet.insert(key(a, b));
                visibleEdges.emplace_back(a, b);
            }
        }
        if (visibleEdges.empty()) continue;
        // Horizon = visible edges whose twin belongs to a face that stays.
        for (const auto& [a, b] : visibleEdges)
            if (!edgeSet.count(key(b, a)))
                tris.push_back(makeTri(pts, a, b, k, interior));
        if (tris.size() > 64 && tris.size() % 64 == 0)
            tris.erase(std::remove_if(tris.begin(), tris.end(),
                                      [](const HullTri& t) { return !t.alive; }), tris.end());
    }
    tris.erase(std::remove_if(tris.begin(), tris.end(),
                              [](const HullTri& t) { return !t.alive; }), tris.end());
    out = std::move(tris);
    return out.size() >= 4;
}

// Points on a sphere spread by the golden angle.
std::vector<V> fibonacciDirections(int count) {
    std::vector<V> dirs;
    dirs.reserve(count);
    const float golden = 3.14159265f * (3.f - std::sqrt(5.f));
    for (int i = 0; i < count; ++i) {
        const float y = 1.f - 2.f * (static_cast<float>(i) + 0.5f) / static_cast<float>(count);
        const float r = std::sqrt(std::max(0.f, 1.f - y * y));
        const float phi = golden * static_cast<float>(i);
        dirs.emplace_back(r * std::cos(phi), y, r * std::sin(phi));
    }
    return dirs;
}

// The distinct support points of `pts` along `dirCount` directions.
std::vector<V> supportPoints(const std::vector<V>& pts, int dirCount) {
    std::vector<int> picked;
    for (const V& dir : fibonacciDirections(dirCount)) {
        int bestI = 0;
        float bestD = glm::dot(dir, pts[0]);
        for (int i = 1; i < static_cast<int>(pts.size()); ++i) {
            const float d = glm::dot(dir, pts[i]);
            if (d > bestD) { bestD = d; bestI = i; }
        }
        picked.push_back(bestI);
    }
    std::sort(picked.begin(), picked.end());
    picked.erase(std::unique(picked.begin(), picked.end()), picked.end());
    std::vector<V> out;
    for (int i : picked) out.push_back(pts[i]);
    return out;
}

} // namespace

bool ConvexHullShape::build(const std::vector<Math::Vector3df>& inputPoints, int maxVertices) {
    vertices.clear();
    faces.clear();
    triangles.clear();
    edges.clear();
    edgeDirs.clear();
    centerOffset = V(0.f);
    volume = 0.f;
    unitInertia = Math::Matrix3df(0.f);
    if (inputPoints.size() < 4) return false;
    maxVertices = std::max(maxVertices, 8);

    V mn = inputPoints[0], mx = inputPoints[0];
    for (const V& p : inputPoints) { mn = glm::min(mn, p); mx = glm::max(mx, p); }
    const float scale = std::max(glm::length(mx - mn), 1e-6f);
    const float eps = scale * 1e-5f;

    // Very large inputs: their support points bound the same hull closely and
    // keep the incremental build cheap.
    std::vector<V> pts = inputPoints.size() > 20000 ? supportPoints(inputPoints, 1024) : inputPoints;

    std::vector<HullTri> tris;
    if (!incrementalHull(pts, eps, tris)) return false;

    auto collectUsed = [&](const std::vector<HullTri>& ts) {
        std::vector<int> used;
        for (const auto& t : ts) used.insert(used.end(), t.v, t.v + 3);
        std::sort(used.begin(), used.end());
        used.erase(std::unique(used.begin(), used.end()), used.end());
        return used;
    };
    std::vector<int> used = collectUsed(tris);
    if (static_cast<int>(used.size()) > maxVertices) {
        std::vector<V> hullPts;
        for (int i : used) hullPts.push_back(pts[i]);
        std::vector<V> reduced = supportPoints(hullPts, maxVertices);
        std::vector<HullTri> reducedTris;
        if (incrementalHull(reduced, eps, reducedTris)) {
            pts = std::move(reduced);
            tris = std::move(reducedTris);
            used = collectUsed(tris);
        }
    }

    std::vector<int> remap(pts.size(), -1);
    for (int i : used) {
        remap[i] = static_cast<int>(vertices.size());
        vertices.push_back(pts[i]);
    }
    for (const auto& t : tris)
        triangles.push_back({ remap[t.v[0]], remap[t.v[1]], remap[t.v[2]] });

    // Mass properties from the tetrahedra (origin, a, b, c) of the outward-wound surface.
    float vol = 0.f;
    V com(0.f);
    Math::Matrix3df cov(0.f);
    for (const auto& t : triangles) {
        const V& a = vertices[t[0]];
        const V& b = vertices[t[1]];
        const V& c = vertices[t[2]];
        const float det = glm::dot(a, glm::cross(b, c));
        vol += det / 6.f;
        com += (a + b + c) * (det / 24.f);
        const V s = a + b + c;
        cov += (glm::outerProduct(a, a) + glm::outerProduct(b, b) + glm::outerProduct(c, c)
                + glm::outerProduct(s, s)) * (det / 120.f);
    }
    if (vol <= eps * eps * eps) {
        vertices.clear();
        triangles.clear();
        return false;
    }
    com /= vol;
    cov -= glm::outerProduct(com, com) * vol;
    const float tr = cov[0][0] + cov[1][1] + cov[2][2];
    unitInertia = (Math::Matrix3df(tr) - cov) / vol;
    volume = vol;
    centerOffset = com;
    for (V& v : vertices) v -= com;

    // Merge coplanar surface triangles into polygon faces (a convex polytope has
    // at most one face per plane, so grouping by plane is enough).
    std::vector<V> groupNormalSum;
    std::vector<std::vector<int>> groupVerts;
    for (const auto& t : triangles) {
        const V& a = vertices[t[0]];
        V n = glm::cross(vertices[t[1]] - a, vertices[t[2]] - a);
        const float area2 = glm::length(n);
        if (area2 < 1e-30f) continue;
        const V unit = n / area2;
        const float d = glm::dot(unit, a);
        int g = 0;
        for (; g < static_cast<int>(faces.size()); ++g)
            if (glm::dot(faces[g].normal, unit) > 0.99995f && std::abs(faces[g].d - d) < eps * 10.f)
                break;
        if (g == static_cast<int>(faces.size())) {
            faces.push_back({ unit, d, {} });
            groupNormalSum.push_back(V(0.f));
            groupVerts.emplace_back();
        }
        groupNormalSum[g] += n;
        groupVerts[g].insert(groupVerts[g].end(), t.begin(), t.end());
    }
    for (int g = 0; g < static_cast<int>(faces.size()); ++g) {
        Face& f = faces[g];
        f.normal = glm::normalize(groupNormalSum[g]);
        auto& vs = groupVerts[g];
        std::sort(vs.begin(), vs.end());
        vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
        V centroid(0.f);
        for (int i : vs) centroid += vertices[i];
        centroid /= static_cast<float>(vs.size());
        f.d = glm::dot(f.normal, centroid);
        V u = vertices[vs[0]] - centroid;
        u -= f.normal * glm::dot(u, f.normal);
        u = (glm::length(u) > 1e-20f) ? glm::normalize(u) : V(1.f, 0.f, 0.f);
        const V w = glm::cross(f.normal, u);
        std::sort(vs.begin(), vs.end(), [&](int ia, int ib) {
            const V ra = vertices[ia] - centroid, rb = vertices[ib] - centroid;
            return std::atan2(glm::dot(w, ra), glm::dot(u, ra))
                 < std::atan2(glm::dot(w, rb), glm::dot(u, rb));
        });
        f.loop = vs;
    }

    for (const Face& f : faces)
        for (size_t i = 0; i < f.loop.size(); ++i) {
            int a = f.loop[i], b = f.loop[(i + 1) % f.loop.size()];
            if (a > b) std::swap(a, b);
            if (std::find(edges.begin(), edges.end(), std::make_pair(a, b)) != edges.end()) continue;
            edges.emplace_back(a, b);
            const V dir = vertices[b] - vertices[a];
            const float len = glm::length(dir);
            if (len < 1e-20f) continue;
            const V unit = dir / len;
            const bool parallel = std::any_of(edgeDirs.begin(), edgeDirs.end(),
                [&](const V& e) { return std::abs(glm::dot(e, unit)) > 0.9999f; });
            if (!parallel) edgeDirs.push_back(unit);
        }
    return true;
}

Math::Box3df ConvexHullShape::getAABB(const Math::Vector3df& pos,
                                      const Math::Quaternion& orient) const {
    if (vertices.empty()) return Math::Box3df(pos, pos);
    const Math::Matrix3df R = glm::mat3_cast(orient);
    V mn = pos + R * vertices[0], mx = mn;
    for (const V& v : vertices) {
        const V w = pos + R * v;
        mn = glm::min(mn, w);
        mx = glm::max(mx, w);
    }
    return Math::Box3df(mn, mx);
}

float ConvexHullShape::closestLocal(const Math::Vector3df& p,
                                    Math::Vector3df& surfacePoint,
                                    Math::Vector3df& normal) const {
    float maxSep = -std::numeric_limits<float>::max();
    const Face* nearest = nullptr;
    for (const Face& f : faces) {
        const float s = glm::dot(f.normal, p) - f.d;
        if (s > maxSep) { maxSep = s; nearest = &f; }
    }
    if (!nearest) {
        surfacePoint = p;
        normal = V(0.f, 1.f, 0.f);
        return 0.f;
    }
    if (maxSep <= 0.f) {
        normal = nearest->normal;
        surfacePoint = p - normal * maxSep;
        return maxSep;
    }
    float best = std::numeric_limits<float>::max();
    for (const auto& t : triangles) {
        const V q = closestPointOnTriangle(p, vertices[t[0]], vertices[t[1]], vertices[t[2]]);
        const V d = p - q;
        const float d2 = glm::dot(d, d);
        if (d2 < best) { best = d2; surfacePoint = q; }
    }
    const float dist = std::sqrt(best);
    normal = (dist > 1e-8f) ? (p - surfacePoint) / dist : nearest->normal;
    return dist;
}

float ConvexHullShape::getSignedDistance(const Math::Vector3df& worldPoint,
                                         const Math::Vector3df& pos,
                                         const Math::Quaternion& orient) const {
    V q, n;
    return closestLocal(glm::inverse(orient) * (worldPoint - pos), q, n);
}

Math::Vector3df ConvexHullShape::getSurfaceNormal(const Math::Vector3df& worldPoint,
                                                  const Math::Vector3df& pos,
                                                  const Math::Quaternion& orient) const {
    V q, n;
    closestLocal(glm::inverse(orient) * (worldPoint - pos), q, n);
    return orient * n;
}

} // namespace Physics
} // namespace Phantom
