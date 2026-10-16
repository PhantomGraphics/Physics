#pragma once

#include "CGLib/Math/Core/Box3d.h"
#include "CGLib/Math/Core/Vector3d.h"

#include <cstddef>
#include <vector>

namespace Phantom {

/**
 * @brief The initial particle layout FluidWorld::reset() hands to whichever solver it builds.
 *
 * Pure geometry (no solver / Vulkan types) so the seeding rule can be tested without a fluid and
 * is written once instead of once per solver type. The rule:
 *  - custom positions (setCustomInitialData(), or the multi-region sources FluidWorld rebuilds on
 *    every reset) win whenever there are any; velocities are optional and may be shorter than the
 *    positions (the particles beyond them start at rest);
 *  - otherwise a regular lattice of spacing `diameter` fills `fluidBounds`, z-major then y then x.
 * The particle order is part of the contract: SPH sums are order-dependent in float, and the
 * deterministic-regression baselines were recorded with exactly this order.
 */
struct FluidSeed {
    std::vector<Math::Vector3df> positions;
    std::vector<Math::Vector3df> velocities;  ///< empty, or one entry per leading position.
};

/** Calls fn(position) for each lattice point of `bounds` (spacing `diameter`; none if <= 0). */
template <class Fn>
void forEachFluidSeedInBounds(const Math::Box3df& bounds, float diameter, Fn&& fn)
{
    const Math::Vector3df bmin = bounds.getMin();
    const Math::Vector3df bmax = bounds.getMax();
    if (diameter <= 0.0f) return;

    for (float z = bmin.z; z <= bmax.z + 1e-4f; z += diameter) {
        for (float y = bmin.y; y <= bmax.y + 1e-4f; y += diameter) {
            for (float x = bmin.x; x <= bmax.x + 1e-4f; x += diameter) {
                fn(Math::Vector3df(x, y, z));
            }
        }
    }
}

inline FluidSeed makeFluidSeed(const std::vector<Math::Vector3df>& customPositions,
                               const std::vector<Math::Vector3df>& customVelocities,
                               const Math::Box3df& fluidBounds, float diameter)
{
    FluidSeed seed;
    if (!customPositions.empty()) {
        seed.positions = customPositions;
        seed.velocities = customVelocities;
        return seed;
    }
    forEachFluidSeedInBounds(fluidBounds, diameter, [&](const Math::Vector3df& pos) {
        seed.positions.push_back(pos);
    });
    return seed;
}

/**
 * Feeds the seed to a fluid: create(position) for every position in order, then
 * setLastVelocity(velocity) for the positions that have one.
 */
template <class CreateFn, class VelocityFn>
void spawnSeed(const FluidSeed& seed, CreateFn create, VelocityFn setLastVelocity)
{
    for (std::size_t i = 0; i < seed.positions.size(); ++i) {
        create(seed.positions[i]);
        if (i < seed.velocities.size()) setLastVelocity(seed.velocities[i]);
    }
}

} // namespace Phantom
