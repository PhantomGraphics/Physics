#pragma once

#include "../../CGLib/Math/Vector3d.h"
#include <vector>

namespace Phantom
{
    namespace Physics {

/**
 * @brief Plain data structure representing a single MVC fluid particle.
 *
 * Stores the per-particle state used by the MVC (Moving Voronoi Cell)
 * fluid solver: position, velocity, mass, volume, density, and a
 * correction delta accumulated during constraint solving.
 *
 * The particle's live storage is Structure-of-Arrays (see MVCParticleSoA
 * below, held by MVCFluid); this type is used only to build up initial
 * values for push_back() and as a single-particle test double.
 */
struct MVCParticle {
    /** @brief World-space position. */
    Math::Vector3df position;
    /** @brief Current velocity. */
    Math::Vector3df velocity;
    /** @brief Particle mass. */
    float mass = 1.0f;
    /** @brief Current particle volume. */
    float volume = 1.0f;
    /** @brief Current density. */
    float density = 0.0f;
    /** @brief Position correction vector accumulated during constraint solving. */
    Math::Vector3df delta;
};

// SoA (Structure of Arrays) 粒子コンテナ。MVCFluid::particles_ の実体。
struct MVCParticleSoA {
    std::vector<Math::Vector3df> positions;
    std::vector<Math::Vector3df> velocities;
    std::vector<float>           masses;
    std::vector<float>           volumes;
    std::vector<float>           densities;
    std::vector<Math::Vector3df> deltas;

    size_t size()  const { return positions.size(); }
    bool   empty() const { return positions.empty(); }

    void clear() {
        positions.clear();
        velocities.clear();
        masses.clear();
        volumes.clear();
        densities.clear();
        deltas.clear();
    }

    void push_back(const MVCParticle& p) {
        positions.push_back(p.position);
        velocities.push_back(p.velocity);
        masses.push_back(p.mass);
        volumes.push_back(p.volume);
        densities.push_back(p.density);
        deltas.push_back(p.delta);
    }
};
    }
}
