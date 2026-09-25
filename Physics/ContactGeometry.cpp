#include "pch.h"
#include "ContactGeometry.h"

namespace Phantom {
namespace Physics {

Math::Vector3df closestPointOnTriangle(const Math::Vector3df& p,
                                       const Math::Vector3df& a,
                                       const Math::Vector3df& b,
                                       const Math::Vector3df& c)
{
    const Math::Vector3df ab = b - a, ac = c - a, ap = p - a;
    const float d1 = glm::dot(ab, ap), d2 = glm::dot(ac, ap);
    if (d1 <= 0.f && d2 <= 0.f) return a;

    const Math::Vector3df bp = p - b;
    const float d3 = glm::dot(ab, bp), d4 = glm::dot(ac, bp);
    if (d3 >= 0.f && d4 <= d3) return b;

    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
        const float denom = d1 - d3;
        return (std::abs(denom) > 1e-20f) ? a + ab * (d1 / denom) : a;
    }

    const Math::Vector3df cp = p - c;
    const float d5 = glm::dot(ab, cp), d6 = glm::dot(ac, cp);
    if (d6 >= 0.f && d5 <= d6) return c;

    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
        const float denom = d2 - d6;
        return (std::abs(denom) > 1e-20f) ? a + ac * (d2 / denom) : a;
    }

    const float va = d3 * d6 - d5 * d4;
    if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
        const float denom = (d4 - d3) + (d5 - d6);
        return (std::abs(denom) > 1e-20f) ? b + (c - b) * ((d4 - d3) / denom) : b;
    }

    const float sum = va + vb + vc;
    if (std::abs(sum) < 1e-20f) return a; // degenerate triangle
    const float v = vb / sum, w = vc / sum;
    return a + ab * v + ac * w;
}

void closestPointsSegmentSegment(const Math::Vector3df& p1, const Math::Vector3df& q1,
                                 const Math::Vector3df& p2, const Math::Vector3df& q2,
                                 Math::Vector3df& c1, Math::Vector3df& c2)
{
    const Math::Vector3df d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
    const float a = glm::dot(d1, d1), e = glm::dot(d2, d2), f = glm::dot(d2, r);
    constexpr float eps = 1e-12f;
    float s = 0.f, t = 0.f;
    if (a <= eps && e <= eps) {
        s = t = 0.f;
    } else if (a <= eps) {
        t = std::clamp(f / e, 0.f, 1.f);
    } else {
        const float c = glm::dot(d1, r);
        if (e <= eps) {
            s = std::clamp(-c / a, 0.f, 1.f);
        } else {
            const float b = glm::dot(d1, d2);
            const float denom = a * e - b * b;
            s = (denom > eps) ? std::clamp((b * f - c * e) / denom, 0.f, 1.f) : 0.f;
            t = (b * s + f) / e;
            if (t < 0.f)      { t = 0.f; s = std::clamp(-c / a, 0.f, 1.f); }
            else if (t > 1.f) { t = 1.f; s = std::clamp((b - c) / a, 0.f, 1.f); }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

void closestPointsSegmentTriangle(const Math::Vector3df& p, const Math::Vector3df& q,
                                  const Math::Vector3df& a, const Math::Vector3df& b,
                                  const Math::Vector3df& c,
                                  Math::Vector3df& onSeg, Math::Vector3df& onTri)
{
    // Piercing: the segment crosses the triangle's plane inside the triangle.
    const Math::Vector3df n = glm::cross(b - a, c - a);
    const float dp = glm::dot(n, p - a), dq = glm::dot(n, q - a);
    if ((dp <= 0.f && dq >= 0.f) || (dp >= 0.f && dq <= 0.f)) {
        const float denom = dp - dq;
        if (std::abs(denom) > 1e-20f) {
            const Math::Vector3df x = p + (q - p) * (dp / denom);
            // Inside test via same-side barycentric signs.
            const float s0 = glm::dot(n, glm::cross(b - a, x - a));
            const float s1 = glm::dot(n, glm::cross(c - b, x - b));
            const float s2 = glm::dot(n, glm::cross(a - c, x - c));
            if (s0 >= 0.f && s1 >= 0.f && s2 >= 0.f) {
                onSeg = onTri = x;
                return;
            }
        }
    }

    // Otherwise the minimum is at a segment end vs the triangle, or a segment
    // vs one of the triangle's edges.
    float best = std::numeric_limits<float>::max();
    auto consider = [&](const Math::Vector3df& s, const Math::Vector3df& t) {
        const Math::Vector3df d = s - t;
        const float d2 = glm::dot(d, d);
        if (d2 < best) { best = d2; onSeg = s; onTri = t; }
    };
    consider(p, closestPointOnTriangle(p, a, b, c));
    consider(q, closestPointOnTriangle(q, a, b, c));
    const Math::Vector3df edges[3][2] = { { a, b }, { b, c }, { c, a } };
    for (const auto& e : edges) {
        Math::Vector3df s, t;
        closestPointsSegmentSegment(p, q, e[0], e[1], s, t);
        consider(s, t);
    }
}

} // namespace Physics
} // namespace Phantom
