#include "VolumeConstraint.h"

namespace Phantom {
namespace Physics {

// 四面体体積のグラジェント:
//   V = dot(p_b - p_a, cross(p_c - p_a, p_d - p_a)) / 6
//   ∂V/∂p_b = cross(p_c-p_a, p_d-p_a) / 6
//   ∂V/∂p_c = cross(p_d-p_a, p_b-p_a) / 6
//   ∂V/∂p_d = cross(p_b-p_a, p_c-p_a) / 6
//   ∂V/∂p_a = -(∂V/∂p_b + ∂V/∂p_c + ∂V/∂p_d)  (並進不変)
void VolumeConstraint::project(SoftParticleSoA& particles, float alphaHat) {
    using Vec3 = Math::Vector3df;

    const Vec3& predA = particles.predicted[a];
    const Vec3& predB = particles.predicted[b];
    const Vec3& predC = particles.predicted[c];
    const Vec3& predD = particles.predicted[d];

    Vec3 ab = predB - predA;
    Vec3 ac = predC - predA;
    Vec3 ad = predD - predA;

    float V = glm::dot(ab, glm::cross(ac, ad)) / 6.f;
    float C = V - restVolume;
    if (std::abs(C) < 1e-10f) return;

    Vec3 gb = glm::cross(ac, ad) / 6.f;
    Vec3 gc = glm::cross(ad, ab) / 6.f;
    Vec3 gd = glm::cross(ab, ac) / 6.f;
    Vec3 ga = -(gb + gc + gd);

    float wa = particles.inverseMasses[a];
    float wb = particles.inverseMasses[b];
    float wc = particles.inverseMasses[c];
    float wd = particles.inverseMasses[d];

    float w = wa * glm::dot(ga, ga)
            + wb * glm::dot(gb, gb)
            + wc * glm::dot(gc, gc)
            + wd * glm::dot(gd, gd);
    if (w < 1e-10f) return;

    float dLambda = -(C + alphaHat * lambda) / (w + alphaHat);
    lambda += dLambda;

    particles.predicted[a] += wa * dLambda * ga;
    particles.predicted[b] += wb * dLambda * gb;
    particles.predicted[c] += wc * dLambda * gc;
    particles.predicted[d] += wd * dLambda * gd;
}

} // namespace Physics
} // namespace Phantom
