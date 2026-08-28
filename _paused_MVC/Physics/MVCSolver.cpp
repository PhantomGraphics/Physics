#include "pch.h"

#include <vector>
#include <cmath> // ・ｽﾇ会ｿｽ
#include <algorithm> // std::max

#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/pi.h"

#include "CGLib/Space/Space/CompactSpaceHash.h"

#include "MVCParticle.h"
#include "MVCSolver.h"
#include "MVCFluid.h"
#include "RigidBoundary.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {

// Non-owning (fluid-SoA, index) handle used by the solver's per-step working
// set. Lets the loop bodies below read/mutate "a particle" with the same
// ergonomics as the old vector<MVCParticle*>, while the real storage stays
// SoA (per-fluid, contiguous) in MVCFluid.
struct ParticleView {
    MVCParticleSoA* soa;
    size_t idx;

    Vector3df& position() const { return soa->positions[idx]; }
    Vector3df& velocity() const { return soa->velocities[idx]; }
    float&     mass()     const { return soa->masses[idx]; }
    float&     volume()   const { return soa->volumes[idx]; }
    float&     density()  const { return soa->densities[idx]; }
    Vector3df& delta()    const { return soa->deltas[idx]; }
};

// Solver-wide kernel/constraint constants (dt/maxIter are now simulate() parameters).
const float h = 0.1f;           // smoothing kernel radius
const float nu = 0.1f;           // viscosity coefficient
const float V0 = 1.0f;           // target volume
const float epsilon = 1e-6f;     // gradient-length regularizer

// ・ｽJ・ｽ[・ｽl・ｽ・ｽ・ｽﾖ撰ｿｽ・ｽiPoly6・ｽ・ｽj
float W(float r, float h) {
    if (r > h) return 0.0f;
    float x = (h * h - r * r);
    return 315.0f / (64.0f * PIf * powf(h, 9.0f)) * x * x * x;
}

// ・ｽ・ｽ・ｽz・ｽJ・ｽ[・ｽl・ｽ・ｽ
Vector3df gradW(Vector3df r_ij, float h) {
    float r = glm::length(r_ij);
    if (r > h || r < 1e-6f) return Vector3df(0, 0, 0);
    return -945.0f / (32.0f * PIf * pow(h, 9.0f)) * pow(h * h - r * r, 2.0f) * r_ij;
}

// MVCParticle has no per-step force accumulator, so the boundary
// acceleration is integrated directly into velocity (then position),
// mirroring the external-force application just above in simulate().
void addRigidBoundaryPressure(const std::vector<RigidBoundary*>& rigidBoundaries,
                               const std::vector<ParticleView>& particles,
                               const std::vector<bool>& isStaticMask, const float dt) {
    if (rigidBoundaries.empty()) return;
    const int N = static_cast<int>(particles.size());
    for (int i = 0; i < N; ++i) {
        if (isStaticMask[i]) continue;
        const auto& pi = particles[i];
        for (auto* rb : rigidBoundaries) {
            const auto force = rb->getBoundaryForce(pi.position());
            pi.velocity() += dt * force;
            pi.position() += dt * dt * force;
        }
    }
}

} // namespace

int MVCSolver::getParticleCount() const
{
    int count = 0;
    for (const auto* fluid : fluids) {
        count += fluid->getNumParticles();
    }
    return count;
}

std::vector<Vector3df> MVCSolver::getParticlePositions() const
{
    std::vector<Vector3df> positions;
    for (const auto* fluid : fluids) {
        const auto& soa = fluid->getParticles();
        positions.insert(positions.end(), soa.positions.begin(), soa.positions.end());
    }
    return positions;
}

// Main update step
void MVCSolver::simulate(const float dt, const int maxIter) {

    std::vector<ParticleView> particles;
    std::vector<bool> isStaticMask;
    for (auto& f : fluids) {
        const bool staticFluid = f->isStatic();
        auto& soa = f->getParticles();
        for (size_t i = 0; i < soa.size(); ++i) {
            particles.push_back(ParticleView{ &soa, i });
            isStaticMask.push_back(staticFluid);
        }
    }

    int N = static_cast<int>(particles.size());

    // 1. External force and predicted position (skip static fluids).
    for (int i = 0; i < N; ++i) {
        if (isStaticMask[i]) continue;
        const auto& p = particles[i];
        p.velocity() += dt * externalForce;
        p.position() += dt * p.velocity();
        p.delta() = Vector3df(0, 0, 0);
    }

    addRigidBoundaryPressure(rigidBoundaries_, particles, isStaticMask, dt);

    // ・ｽ・ｽﾔハ・ｽb・ｽV・ｽ・ｽ・ｽ・ｽ・ｽ・ｽx・ｽ\・ｽz・ｽi・ｽS・ｽ・ｽ・ｽv・ｽZ・ｽp・ｽj
    const int tableSize = std::max(16, N * 2);
    Phantom::Space::CompactSpaceHash spaceHash(h, tableSize);
    for (int i = 0; i < N; ++i) {
        spaceHash.add(particles[i].position());
    }

    // 2. ・ｽS・ｽ・ｽ・ｽi・ｽﾟ傍・ｽﾌみ）
    for (int i = 0; i < N; ++i) {
        const auto& pi = particles[i];
        Vector3df visc(0, 0, 0);
        const auto neighbors = spaceHash.findNeighborIndices(i);
        for (const auto jIndex : neighbors) {
            const auto& pj = particles[jIndex];
            float w = W(glm::length(pi.position() - pj.position()), h);
            visc += (pj.velocity() - pi.velocity()) * w;
        }
        pi.velocity() += nu * visc;
    }

    // 3. ・ｽﾌ積補正・ｽ・ｽ・ｽ・ｽ
    for (int iter = 0; iter < maxIter; ++iter) {
        // ・ｽ・ｽ・ｽx・ｽE・ｽﾌ積計・ｽZ
        for (int i = 0; i < N; ++i) {
            const auto& pi = particles[i];
            float rho = 0.0f;
            //rho += pi->mass / pi->volume * W(0.0f, h);
            const auto neighbors = spaceHash.findNeighborIndices(i);
            for (const auto jIndex : neighbors) {
                const auto& pj = particles[jIndex];
                float r = glm::length(pi.position() - pj.position());
                rho += pj.mass() / pj.volume() * W(r, h);
            }
            pi.density() = rho;
        }

        // ・ｽﾌ積拘・ｽ・ｽ・ｽ竦ｳ・ｽi・ｽﾟ傍・ｽﾌみ）
        for (int i = 0; i < N; ++i) {
            const auto& pi = particles[i];
            float Ci = 0.0f;
            Vector3df gradCi(0, 0, 0);

            // ・ｽ・ｽ・ｽﾈ奇ｿｽ^・ｽiW(0) ・ｽﾍ費ｿｽ[・ｽ・ｽ・ｽAgrad ・ｽﾍゼ・ｽ・ｽ・ｽj
            //Ci += pi->volume * W(0.0f, h);

            const auto neighbors = spaceHash.findNeighborIndices(i);
            for (const auto jIndex : neighbors) {
                const auto& pj = particles[jIndex];
                Vector3df r_ij = pi.position() - pj.position();
                float w = W(glm::length(r_ij), h);
                Ci += pj.volume() * w;
                gradCi += gradW(r_ij, h);
            }
            Ci = Ci / V0 - 1.0f; // ・ｽﾌ積誤差
            float gradCiLenSq = glm::length(gradCi);
            gradCiLenSq *= gradCiLenSq;
            pi.delta() = -Ci * gradCi / (gradCiLenSq + epsilon);
        }

        // Constraint correction (skip static fluids).
        for (int i = 0; i < N; ++i) {
            if (isStaticMask[i]) continue;
            particles[i].position() += particles[i].delta();
        }
    }

    // 4. Final velocity update (skip static fluids).
    for (int i = 0; i < N; ++i) {
        if (isStaticMask[i]) continue;
        const auto& p = particles[i];
        p.velocity() = (p.position() - (p.position() - dt * p.velocity())) / dt;
    }

    // 5. Floor boundary (skip static fluids).
    for (int i = 0; i < N; ++i) {
        if (isStaticMask[i]) continue;
        const auto& p = particles[i];
        if (p.position().y < 0.0f) {
            p.position().y = 0.0f;
            if (p.velocity().y < 0) p.velocity().y *= -0.5f;
        }
    }
}
