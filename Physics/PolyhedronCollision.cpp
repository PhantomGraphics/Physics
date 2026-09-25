#include "pch.h"
#include "PolyhedronCollision.h"
#include "ContactGeometry.h"
#include "ConvexHullShape.h"

#include "CGLib/Math/Matrix3d.h"
#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

namespace Phantom {
namespace Physics {

namespace {

using V = Math::Vector3df;

// Reverse `loop` if its winding disagrees with `normal` (Newell normal).
void orientLoop(const std::vector<V>& verts, std::vector<int>& loop, const V& normal) {
    V newell(0.f);
    for (size_t i = 0; i < loop.size(); ++i) {
        const V& p = verts[loop[i]];
        const V& q = verts[loop[(i + 1) % loop.size()]];
        newell += glm::cross(p, q);
    }
    if (glm::dot(newell, normal) < 0.f) std::reverse(loop.begin(), loop.end());
}

void projectInterval(const std::vector<V>& verts, const V& axis, float& mn, float& mx) {
    mn = mx = glm::dot(axis, verts[0]);
    for (size_t i = 1; i < verts.size(); ++i) {
        const float d = glm::dot(axis, verts[i]);
        mn = std::min(mn, d);
        mx = std::max(mx, d);
    }
}

struct FaceQuery {
    float sep = -std::numeric_limits<float>::max();
    int   face = -1;
};

// Deepest penetration of `other` below each of `ref`'s faces; the best face is
// the one with the largest (least negative) separation.
FaceQuery queryFaces(const WorldPolyhedron& ref, const WorldPolyhedron& other) {
    FaceQuery q;
    for (int f = 0; f < static_cast<int>(ref.faces.size()); ++f) {
        const auto& face = ref.faces[f];
        float s = std::numeric_limits<float>::max();
        for (const V& v : other.verts) s = std::min(s, glm::dot(face.normal, v) - face.d);
        if (s > q.sep) { q.sep = s; q.face = f; }
        if (s > 0.f) return q;
    }
    return q;
}

struct EdgeQuery {
    float sep = -std::numeric_limits<float>::max();
    V     axis = V(0.f); // points from B toward A
    int   dirA = -1, dirB = -1;
};

EdgeQuery queryEdges(const WorldPolyhedron& a, const WorldPolyhedron& b) {
    EdgeQuery q;
    for (int i = 0; i < static_cast<int>(a.edgeDirs.size()); ++i)
        for (int j = 0; j < static_cast<int>(b.edgeDirs.size()); ++j) {
            V axis = glm::cross(a.edgeDirs[i], b.edgeDirs[j]);
            const float len = glm::length(axis);
            if (len < 1e-4f) continue; // parallel edges: covered by the face axes
            axis /= len;
            float minA, maxA, minB, maxB;
            projectInterval(a.verts, axis, minA, maxA);
            projectInterval(b.verts, axis, minB, maxB);
            // Both orientations: a flat polyhedron (triangle) has no meaningful center to orient by.
            const float sPos = minA - maxB; // A above B along +axis
            const float sNeg = minB - maxA; // A above B along -axis
            const float s = std::max(sPos, sNeg);
            if (s > q.sep) {
                q.sep = s;
                q.axis = (sPos >= sNeg) ? axis : -axis;
                q.dirA = i;
                q.dirB = j;
            }
            if (s > 0.f) return q;
        }
    return q;
}

// Sutherland-Hodgman: keep the part of `poly` with dot(n, p) <= d.
void clipPolygon(std::vector<V>& poly, const V& n, float d) {
    if (poly.empty()) return;
    std::vector<V> out;
    out.reserve(poly.size() + 2);
    for (size_t i = 0; i < poly.size(); ++i) {
        const V& p = poly[i];
        const V& q = poly[(i + 1) % poly.size()];
        const float dp = glm::dot(n, p) - d, dq = glm::dot(n, q) - d;
        if (dp <= 0.f) out.push_back(p);
        if ((dp < 0.f && dq > 0.f) || (dp > 0.f && dq < 0.f))
            out.push_back(p + (q - p) * (dp / (dp - dq)));
    }
    poly.swap(out);
}

// Face contact: `ref` owns the reference face; `normalSign` turns its outward
// normal into the B->A contact normal (-1 when ref is A, +1 when ref is B).
bool faceContact(const WorldPolyhedron& ref, int refFace, const WorldPolyhedron& inc,
                 float normalSign, std::vector<ContactPoint>& out) {
    const auto& rf = ref.faces[refFace];
    int incFace = 0;
    float minDot = std::numeric_limits<float>::max();
    for (int f = 0; f < static_cast<int>(inc.faces.size()); ++f) {
        const float d = glm::dot(inc.faces[f].normal, rf.normal);
        if (d < minDot) { minDot = d; incFace = f; }
    }

    std::vector<V> poly;
    for (int i : inc.faces[incFace].loop) poly.push_back(inc.verts[i]);
    const size_t n = rf.loop.size();
    for (size_t i = 0; i < n && !poly.empty(); ++i) {
        const V& p = ref.verts[rf.loop[i]];
        const V& q = ref.verts[rf.loop[(i + 1) % n]];
        V side = glm::cross(q - p, rf.normal);
        const float len = glm::length(side);
        if (len < 1e-12f) continue;
        side /= len;
        clipPolygon(poly, side, glm::dot(side, p));
    }

    std::vector<ContactPoint> found;
    for (const V& p : poly) {
        const float depth = glm::dot(rf.normal, p) - rf.d;
        if (depth > 0.f) continue;
        ContactPoint cp;
        cp.normal = rf.normal * normalSign;
        cp.penetration = -depth;
        cp.position = p - rf.normal * (depth * 0.5f);
        found.push_back(cp);
    }
    if (found.empty()) {
        // Numerical corner case (clipping removed everything): the incident
        // body's deepest vertex below the reference plane.
        float deepest = 0.f;
        const V* best = nullptr;
        for (const V& v : inc.verts) {
            const float depth = glm::dot(rf.normal, v) - rf.d;
            if (depth < deepest) { deepest = depth; best = &v; }
        }
        if (!best) return false;
        ContactPoint cp;
        cp.normal = rf.normal * normalSign;
        cp.penetration = -deepest;
        cp.position = *best - rf.normal * (deepest * 0.5f);
        found.push_back(cp);
    }
    reduceContacts(found, rf.normal);
    out.insert(out.end(), found.begin(), found.end());
    return true;
}

// The edge of `poly` running along `dir` that lies furthest along `towards`.
int supportEdge(const WorldPolyhedron& poly, const V& dir, const V& towards) {
    int best = -1;
    float bestD = -std::numeric_limits<float>::max();
    for (int e = 0; e < static_cast<int>(poly.edges.size()); ++e) {
        const V& p = poly.verts[poly.edges[e].first];
        const V& q = poly.verts[poly.edges[e].second];
        const V ed = q - p;
        const float len = glm::length(ed);
        if (len < 1e-12f || std::abs(glm::dot(ed / len, dir)) < 0.999f) continue;
        const float d = glm::dot(towards, (p + q) * 0.5f);
        if (d > bestD) { bestD = d; best = e; }
    }
    return best;
}

} // namespace

WorldPolyhedron WorldPolyhedron::fromBox(const BoxShape& box, const Math::Vector3df& pos,
                                         const Math::Quaternion& orient) {
    WorldPolyhedron p;
    const Math::Matrix3df R = glm::mat3_cast(orient);
    const V& h = box.halfExtents;
    for (int i = 0; i < 8; ++i)
        p.verts.push_back(pos + R[0] * ((i & 1) ? h.x : -h.x)
                              + R[1] * ((i & 2) ? h.y : -h.y)
                              + R[2] * ((i & 4) ? h.z : -h.z));
    // Corner index bits: x=1, y=2, z=4.
    const int loops[6][4] = { { 0, 2, 6, 4 }, { 1, 3, 7, 5 }, { 0, 1, 5, 4 },
                              { 2, 3, 7, 6 }, { 0, 1, 3, 2 }, { 4, 5, 7, 6 } };
    for (int f = 0; f < 6; ++f) {
        const V n = R[f / 2] * ((f % 2) ? 1.f : -1.f);
        Face face{ n, 0.f, { loops[f][0], loops[f][1], loops[f][2], loops[f][3] } };
        face.d = glm::dot(n, p.verts[face.loop[0]]);
        orientLoop(p.verts, face.loop, n);
        p.faces.push_back(face);
    }
    for (int i = 0; i < 8; ++i)
        for (int bit : { 1, 2, 4 })
            if (!(i & bit)) p.edges.emplace_back(i, i | bit);
    p.edgeDirs = { R[0], R[1], R[2] };
    p.center = pos;
    return p;
}

WorldPolyhedron WorldPolyhedron::fromHull(const ConvexHullShape& hull, const Math::Vector3df& pos,
                                          const Math::Quaternion& orient) {
    WorldPolyhedron p;
    const Math::Matrix3df R = glm::mat3_cast(orient);
    p.verts.reserve(hull.getVertices().size());
    for (const V& v : hull.getVertices()) p.verts.push_back(pos + R * v);
    p.faces.reserve(hull.getFaces().size());
    for (const auto& f : hull.getFaces()) {
        const V n = R * f.normal;
        p.faces.push_back({ n, f.d + glm::dot(n, pos), f.loop });
    }
    p.edges = hull.getEdges();
    for (const V& e : hull.getEdgeDirections()) p.edgeDirs.push_back(R * e);
    p.center = pos;
    return p;
}

WorldPolyhedron WorldPolyhedron::fromTriangle(const Math::Vector3df& a, const Math::Vector3df& b,
                                              const Math::Vector3df& c) {
    WorldPolyhedron p;
    p.verts = { a, b, c };
    const V n = glm::normalize(glm::cross(b - a, c - a));
    p.faces.push_back({ n, glm::dot(n, a), { 0, 1, 2 } });
    p.faces.push_back({ -n, -glm::dot(n, a), { 0, 2, 1 } });
    p.edges = { { 0, 1 }, { 1, 2 }, { 2, 0 } };
    for (const auto& [i, j] : p.edges) p.edgeDirs.push_back(glm::normalize(p.verts[j] - p.verts[i]));
    p.center = (a + b + c) / 3.f;
    return p;
}

bool collidePolyhedra(const WorldPolyhedron& a, const WorldPolyhedron& b,
                      std::vector<ContactPoint>& out) {
    if (a.verts.empty() || b.verts.empty()) return false;
    const FaceQuery fa = queryFaces(a, b);
    if (fa.sep > 0.f) return false;
    const FaceQuery fb = queryFaces(b, a);
    if (fb.sep > 0.f) return false;
    const EdgeQuery eq = queryEdges(a, b);
    if (eq.sep > 0.f) return false;

    // Prefer face contacts (stable, multi-point); take B's face or an edge
    // pair only when it is clearly shallower.
    constexpr float kRel = 0.98f, kAbs = 1e-3f;
    const bool useB = fb.sep > kRel * fa.sep + kAbs;
    const float faceSep = useB ? fb.sep : fa.sep;
    if (eq.dirA >= 0 && eq.sep > kRel * faceSep + kAbs) {
        const int ea = supportEdge(a, a.edgeDirs[eq.dirA], -eq.axis);
        const int eb = supportEdge(b, b.edgeDirs[eq.dirB], eq.axis);
        if (ea >= 0 && eb >= 0) {
            V ca, cb;
            closestPointsSegmentSegment(a.verts[a.edges[ea].first], a.verts[a.edges[ea].second],
                                        b.verts[b.edges[eb].first], b.verts[b.edges[eb].second],
                                        ca, cb);
            ContactPoint cp;
            cp.normal = eq.axis;
            cp.penetration = -eq.sep;
            cp.position = (ca + cb) * 0.5f;
            out.push_back(cp);
            return true;
        }
    }
    return useB ? faceContact(b, fb.face, a, 1.f, out)
                : faceContact(a, fa.face, b, -1.f, out);
}

void reduceContacts(std::vector<ContactPoint>& contacts, const Math::Vector3df& normal) {
    if (contacts.size() <= 4) return;
    auto at = [&](size_t i) { return contacts[i].position; };
    size_t i0 = 0;
    for (size_t i = 1; i < contacts.size(); ++i)
        if (contacts[i].penetration > contacts[i0].penetration) i0 = i;
    size_t i1 = i0;
    float best = -1.f;
    for (size_t i = 0; i < contacts.size(); ++i) {
        const V d = at(i) - at(i0);
        const float d2 = glm::dot(d, d);
        if (d2 > best) { best = d2; i1 = i; }
    }
    size_t i2 = i0, i3 = i0;
    float maxArea = 0.f, minArea = 0.f;
    for (size_t i = 0; i < contacts.size(); ++i) {
        const float area = glm::dot(glm::cross(at(i1) - at(i0), at(i) - at(i0)), normal);
        if (area > maxArea) { maxArea = area; i2 = i; }
        if (area < minArea) { minArea = area; i3 = i; }
    }
    std::vector<size_t> keep = { i0, i1, i2, i3 };
    std::sort(keep.begin(), keep.end());
    keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
    std::vector<ContactPoint> reduced;
    for (size_t i : keep) reduced.push_back(contacts[i]);
    contacts.swap(reduced);
}

} // namespace Physics
} // namespace Phantom
