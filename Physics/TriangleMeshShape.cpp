#include "pch.h"
#include "TriangleMeshShape.h"
#include "ContactGeometry.h"

#include "CGLib/ThirdParty/glm-0.9.9.8/glm/gtc/quaternion.hpp"

namespace Phantom {
namespace Physics {

namespace {
constexpr int kLeafSize = 4;
}

bool TriangleMeshShape::build(const std::vector<Math::Vector3df>& vertices,
                              const std::vector<uint32_t>& indices) {
    verts_ = vertices;
    tris_.clear();
    triBoxes_.clear();
    order_.clear();
    nodes_.clear();
    const size_t nv = verts_.size();
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const uint32_t a = indices[i], b = indices[i + 1], c = indices[i + 2];
        if (a >= nv || b >= nv || c >= nv) continue;
        const Math::Vector3df n = glm::cross(verts_[b] - verts_[a], verts_[c] - verts_[a]);
        if (glm::dot(n, n) < 1e-24f) continue;
        tris_.push_back({ a, b, c });
        Math::Box3df box(verts_[a]);
        box.add(verts_[b]);
        box.add(verts_[c]);
        triBoxes_.push_back(box);
    }
    if (tris_.empty()) return false;

    order_.resize(tris_.size());
    for (int i = 0; i < static_cast<int>(order_.size()); ++i) order_[i] = i;
    nodes_.reserve(2 * tris_.size() / kLeafSize + 1);
    buildNode(0, static_cast<int>(tris_.size()), 0);
    localBounds_ = nodes_[0].box;
    return true;
}

int TriangleMeshShape::buildNode(int first, int count, int depth) {
    const int index = static_cast<int>(nodes_.size());
    nodes_.emplace_back();
    Math::Box3df box = triBoxes_[order_[first]];
    Math::Box3df centers(triBoxes_[order_[first]].getCenter());
    for (int i = first; i < first + count; ++i) {
        box.add(triBoxes_[order_[i]]);
        centers.add(triBoxes_[order_[i]].getCenter());
    }
    nodes_[index].box = box;
    if (count <= kLeafSize || depth > 40) {
        nodes_[index].first = first;
        nodes_[index].count = count;
        return index;
    }

    // Median split along the longest axis of the triangle centers.
    const Math::Vector3df ext = centers.getLength();
    const int axis = (ext.x >= ext.y && ext.x >= ext.z) ? 0 : (ext.y >= ext.z ? 1 : 2);
    const int mid = first + count / 2;
    std::nth_element(order_.begin() + first, order_.begin() + mid, order_.begin() + first + count,
        [&](int a, int b) {
            return triBoxes_[a].getCenter()[axis] < triBoxes_[b].getCenter()[axis];
        });
    const int left = buildNode(first, mid - first, depth + 1);
    const int right = buildNode(mid, first + count - mid, depth + 1);
    nodes_[index].left = left;
    nodes_[index].right = right;
    return index;
}

void TriangleMeshShape::queryTriangles(const Math::Box3df& localBox, std::vector<int>& out) const {
    out.clear();
    if (nodes_.empty()) return;
    int stack[128];
    int top = 0;
    stack[top++] = 0;
    while (top > 0) {
        const Node& node = nodes_[stack[--top]];
        if (!node.box.intersects(localBox)) continue;
        if (node.left < 0) {
            for (int i = node.first; i < node.first + node.count; ++i)
                if (triBoxes_[order_[i]].intersects(localBox)) out.push_back(order_[i]);
        } else if (top + 2 <= 128) {
            stack[top++] = node.left;
            stack[top++] = node.right;
        }
    }
}

Math::Box3df TriangleMeshShape::getAABB(const Math::Vector3df& pos,
                                        const Math::Quaternion& orient) const {
    if (nodes_.empty()) return Math::Box3df(pos, pos);
    const Math::Vector3df mn = localBounds_.getMin(), mx = localBounds_.getMax();
    Math::Box3df box(pos + orient * mn);
    for (int i = 1; i < 8; ++i) {
        const Math::Vector3df corner((i & 1) ? mx.x : mn.x, (i & 2) ? mx.y : mn.y, (i & 4) ? mx.z : mn.z);
        box.add(pos + orient * corner);
    }
    return box;
}

int TriangleMeshShape::nearestTriangle(const Math::Vector3df& p, Math::Vector3df& closest) const {
    // Brute force: only used by the SDF-style ICollisionShape queries, never by
    // the rigid-body narrow phase.
    int best = -1;
    float bestD2 = std::numeric_limits<float>::max();
    for (int i = 0; i < static_cast<int>(tris_.size()); ++i) {
        const auto& t = tris_[i];
        const Math::Vector3df q = closestPointOnTriangle(p, verts_[t[0]], verts_[t[1]], verts_[t[2]]);
        const Math::Vector3df d = p - q;
        const float d2 = glm::dot(d, d);
        if (d2 < bestD2) { bestD2 = d2; best = i; closest = q; }
    }
    return best;
}

float TriangleMeshShape::getSignedDistance(const Math::Vector3df& worldPoint,
                                           const Math::Vector3df& pos,
                                           const Math::Quaternion& orient) const {
    const Math::Vector3df local = glm::inverse(orient) * (worldPoint - pos);
    Math::Vector3df q;
    if (nearestTriangle(local, q) < 0) return std::numeric_limits<float>::max();
    return glm::length(local - q);
}

Math::Vector3df TriangleMeshShape::getSurfaceNormal(const Math::Vector3df& worldPoint,
                                                    const Math::Vector3df& pos,
                                                    const Math::Quaternion& orient) const {
    const Math::Vector3df local = glm::inverse(orient) * (worldPoint - pos);
    Math::Vector3df q;
    const int t = nearestTriangle(local, q);
    if (t < 0) return Math::Vector3df(0.f, 1.f, 0.f);
    Math::Vector3df d = local - q;
    const float len = glm::length(d);
    if (len > 1e-8f) return orient * (d / len);
    const auto& tri = tris_[t];
    const Math::Vector3df n = glm::normalize(
        glm::cross(verts_[tri[1]] - verts_[tri[0]], verts_[tri[2]] - verts_[tri[0]]));
    return orient * n;
}

} // namespace Physics
} // namespace Phantom
