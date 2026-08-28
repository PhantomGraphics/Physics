#include "ShapeMatchingConstraint.h"
#include "CGLib/Math/Quaternion.h"

namespace Phantom {
namespace Physics {

void ShapeMatchingConstraint::build(const std::vector<int>& idx,
                                    const SoftParticleSoA& particles) {
    indices = idx;
    masses.clear();
    restPos.clear();

    float totalMass = 0.f;
    Math::Vector3df cm = { 0.f, 0.f, 0.f };

    for (int i : indices) {
        float invMass = particles.inverseMasses[i];
        float m = (invMass > 0.f) ? 1.f / invMass : 0.f;
        masses.push_back(m);
        cm += m * particles.positions[i];
        totalMass += m;
    }
    if (totalMass > 1e-9f) cm /= totalMass;

    for (int i : indices)
        restPos.push_back(particles.positions[i] - cm);
}

Math::Matrix3df ShapeMatchingConstraint::extractRotation(const Math::Matrix3df& Apq, int maxIter) {
    // 反復回転抽出 (Müller et al. 2016)
    // GLM は列優先: Apq[col][row]
    glm::quat q(1.f, 0.f, 0.f, 0.f); // identity quaternion (w,x,y,z)
    for (int iter = 0; iter < maxIter; ++iter) {
        Math::Matrix3df R = glm::mat3_cast(q);
        Math::Vector3df omega =
            (glm::cross(R[0], Apq[0]) + glm::cross(R[1], Apq[1]) + glm::cross(R[2], Apq[2]))
            / (std::abs(glm::dot(R[0], Apq[0]) + glm::dot(R[1], Apq[1]) + glm::dot(R[2], Apq[2])) + 1e-9f);

        float len = glm::length(omega);
        if (len < 1e-9f) break;
        q = glm::normalize(glm::angleAxis(len, omega / len) * q);
    }
    return glm::mat3_cast(q);
}

void ShapeMatchingConstraint::project(SoftParticleSoA& particles, float alphaHat) {
    if (indices.empty()) return;

    // 現在の重心
    float totalMass = 0.f;
    Math::Vector3df cm = { 0.f, 0.f, 0.f };
    for (size_t k = 0; k < indices.size(); ++k) {
        cm        += masses[k] * particles.predicted[indices[k]];
        totalMass += masses[k];
    }
    if (totalMass < 1e-9f) return;
    cm /= totalMass;

    // Apq = Σ m_k * (p_k - cm) ⊗ q_k  (3×3 行列、GLM 列優先)
    Math::Matrix3df Apq(0.f);
    for (size_t k = 0; k < indices.size(); ++k) {
        Math::Vector3df dp = particles.predicted[indices[k]] - cm;
        const Math::Vector3df& q = restPos[k];
        float m = masses[k];
        // outer product (dp)(q)^T 加算
        Apq[0] += m * dp * q.x;
        Apq[1] += m * dp * q.y;
        Apq[2] += m * dp * q.z;
    }

    Math::Matrix3df R = extractRotation(Apq);

    // 目標位置 g_k = R * restPos[k] + cm
    // compliance を blendFactor に変換: β ≈ 1 / (1 + alphaHat)
    float beta = 1.f / (1.f + alphaHat);

    for (size_t k = 0; k < indices.size(); ++k) {
        int i = indices[k];
        if (particles.isPinned(i)) continue;
        Math::Vector3df goal = R * restPos[k] + cm;
        particles.predicted[i] += beta * particles.inverseMasses[i] * (goal - particles.predicted[i]);
    }
}

} // namespace Physics
} // namespace Phantom
