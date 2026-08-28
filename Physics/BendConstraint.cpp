#include "BendConstraint.h"

namespace Phantom {
namespace Physics {

// 二面角制約のグラジェント（導出:
//   n0 = cross(e, d02), n1 = cross(e, d03)
//   f  = n̂0·n̂1 = cos(θ)
//   A  = n̂1 - f*n̂0,  B = n̂0 - f*n̂1
//   df/dp2 = A×e / A0
//   df/dp3 = B×e / A1
//   df/dp0 = A×d12/A0 + B×d13/A1
//   df/dp1 = d02×A/A0 + d03×B/A1
//   (Σ grad_pi = 0 が成立, 並進不変)
// )
void BendConstraint::project(SoftParticleSoA& particles, float alphaHat) {
    using Vec3 = Math::Vector3df;

    const Vec3& pred0 = particles.predicted[i0];
    const Vec3& pred1 = particles.predicted[i1];
    const Vec3& pred2 = particles.predicted[i2];
    const Vec3& pred3 = particles.predicted[i3];

    Vec3 e   = pred1 - pred0;
    Vec3 d02 = pred2 - pred0;
    Vec3 d03 = pred3 - pred0;
    Vec3 d12 = pred2 - pred1;
    Vec3 d13 = pred3 - pred1;

    Vec3  n0  = glm::cross(e, d02);
    Vec3  n1  = glm::cross(e, d03);
    float A0  = glm::length(n0);
    float A1  = glm::length(n1);
    if (A0 < 1e-8f || A1 < 1e-8f) return;

    Vec3 nn0 = n0 / A0;
    Vec3 nn1 = n1 / A1;

    float f = glm::clamp(glm::dot(nn0, nn1), -1.f, 1.f);
    float C = f - restCos;
    if (std::abs(C) < 1e-9f) return;

    Vec3 A = nn1 - f * nn0;
    Vec3 B = nn0 - f * nn1;

    Vec3 g2 = glm::cross(A, e)   / A0;
    Vec3 g3 = glm::cross(B, e)   / A1;
    Vec3 g0 = glm::cross(A, d12) / A0 + glm::cross(B, d13) / A1;
    Vec3 g1 = glm::cross(d02, A) / A0 + glm::cross(d03, B) / A1;

    float w0 = particles.inverseMasses[i0];
    float w1 = particles.inverseMasses[i1];
    float w2 = particles.inverseMasses[i2];
    float w3 = particles.inverseMasses[i3];

    float w = w0 * glm::dot(g0, g0)
            + w1 * glm::dot(g1, g1)
            + w2 * glm::dot(g2, g2)
            + w3 * glm::dot(g3, g3);
    if (w < 1e-10f) return;

    float dLambda = -(C + alphaHat * lambda) / (w + alphaHat);
    lambda += dLambda;

    particles.predicted[i0] += w0 * dLambda * g0;
    particles.predicted[i1] += w1 * dLambda * g1;
    particles.predicted[i2] += w2 * dLambda * g2;
    particles.predicted[i3] += w3 * dLambda * g3;
}

} // namespace Physics
} // namespace Phantom
