#include "DistanceConstraint.h"

namespace Phantom {
namespace Physics {

void DistanceConstraint::project(SoftParticleSoA& particles, float alphaHat) {
    Math::Vector3df n = particles.predicted[a] - particles.predicted[b];
    float d = glm::length(n);
    if (d < 1e-8f) return;

    float wa = particles.inverseMasses[a];
    float wb = particles.inverseMasses[b];

    float C = d - restLength;
    float w = wa + wb;
    if (w < 1e-10f) return;

    float dLambda = -(C + alphaHat * lambda) / (w + alphaHat);
    lambda += dLambda;

    n /= d;
    particles.predicted[a] += ( wa * dLambda) * n;
    particles.predicted[b] += (-wb * dLambda) * n;
}

} // namespace Physics
} // namespace Phantom
