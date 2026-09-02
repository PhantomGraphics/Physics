#pragma once

#include "CGLib/Math/Vector3d.h"

#include <cmath>
#include <random>
#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A circular emission region that spawns new particles over time.
 *
 * Shared data model used by WCSPHFluid/DFSPHFluid/PBSPHFluid's
 * updateEmitters() (internal design notes). Mirrors
 * FlameFluid::Emitter's center/radius/rate/accumulator, plus an initial
 * emission direction/speed since (unlike Flame's buoyancy-driven particles)
 * plain SPH particles need an explicit initial velocity to look like a jet
 * rather than particles appearing at rest.
 */
struct Emitter {
	Math::Vector3df center{ 0.0f, 0.0f, 0.0f };
	float radius = 0.1f;

	// Radius of each spawned particle (passed straight through to the
	// fluid's own createParticle()). Independent of the emission disk's
	// radius above.
	//
	// IMPORTANT: WCSPHFluid/DFSPHFluid/PBSPHFluid all derive a spawned
	// particle's SPH mass from this radius (mass ~ density * (2*radius)^3,
	// see e.g. WCSPHParticle::getMass()/PBSPHParticle::getMass()/
	// DFSPHFluid::updateEmitters()'s own diameter^3 formula). If this
	// doesn't match the radius the rest of the scene's particles were
	// created with, spawned particles carry a wildly different mass than
	// the density/pressure solve was calibrated for (internal design notes) and the solver can diverge trying to
	// reconcile the mismatch -- this is exactly what happened when this
	// field defaulted to 0.05 against a default scene radius of 1.0 (a
	// ~1:8000 mass ratio). Defaults to 1.0 to match FluidWorld::Params::
	// radius's own default; FluidWorld::addEmitter() additionally forces
	// this to the scene's actual params().radius before registering, so
	// scenario/UI-created emitters can't reintroduce the mismatch even if
	// this default ever drifts from Params::radius's.
	float particleRadius = 1.0f;

	// Initial velocity given to newly spawned particles: direction (need not
	// be pre-normalized -- accumulateEmission()/updateEmitters() callers
	// normalize it) times speed, plus +/-speedJitter of random variation.
	Math::Vector3df direction{ 0.0f, 1.0f, 0.0f };
	float speed = 1.0f;
	float speedJitter = 0.1f;

	float rate = 50.0f; // particles/sec

	// Fractional-particle carry between steps (rate*dt is rarely integral).
	// Not user-facing tuning; updateEmitters() reads/writes this each call so
	// the average emission rate stays correct across steps instead of every
	// step rounding down (or up) independently.
	float accumulator = 0.0f;

	// Backing store for nextDiskLatticeOffset() -- emission positions cycle
	// through a fixed lattice rather than being drawn at random (see that
	// function for why). Rebuilt whenever radius/particleRadius/direction
	// change; not user-facing tuning.
	std::vector<Math::Vector3df> latticeOffsets;
	int latticeIndex = 0;
	float latticeRadius = -1.0f;
	float latticeSpacing = -1.0f;
	Math::Vector3df latticeNormal{ 0.0f, 0.0f, 0.0f };
};

/**
 * @brief Consumes rate*dt from the emitter's accumulator and returns how many
 * whole particles should be spawned this step, capped at maxToEmit (e.g. the
 * remaining headroom under a fluid's maxParticles). Leftover fractional
 * emission stays in e.accumulator for the next call.
 * @param e          Emitter to update (accumulator is mutated in place).
 * @param dt         Time step (seconds).
 * @param maxToEmit  Upper bound on the returned count (e.g. remaining
 *                   particle budget); pass a large value for no cap.
 * @return Number of particles to spawn this step (0 if the emitter hasn't
 *         accumulated a whole particle yet, or maxToEmit is 0).
 */
inline int accumulateEmission(Emitter& e, const float dt, const int maxToEmit)
{
	e.accumulator += e.rate * dt;
	int n = 0;
	while (e.accumulator >= 1.0f && n < maxToEmit) {
		e.accumulator -= 1.0f;
		++n;
	}
	return n;
}

/**
 * @brief Samples a uniform-random point within a disk of the given radius,
 * centered at the origin and lying in the plane perpendicular to `normal` --
 * i.e. the jet's actual cross-section.
 *
 * This used to hard-code the XZ plane (Y-normal), a leftover from the
 * always-upward jets FlameFluid::updateEmitters() emits. Any emitter firing
 * along a non-Y direction therefore spawned its particles on a zero-thickness
 * ribbon standing edge-on to the flow instead of a disk across it, and the SPH
 * density of the resulting column had nothing to do with the intended one
 * (internal design notes section 2.3). For
 * normal = (0,1,0) the returned distribution is statistically identical to the
 * old one -- the same disk, merely rotated within its own plane.
 *
 * @param radius Disk radius.
 * @param rng    Random engine to draw from.
 * @param normal Unit-length disk normal (the emitter's normalized direction).
 * @return Offset to add to an emitter's center.
 */
inline std::vector<Math::Vector3df> makeDiskLatticeOffsets(
	const float radius, const float spacing, const Math::Vector3df& normal)
{
	// Orthonormal basis (u, v) spanning the plane perpendicular to `normal`.
	// Seeding the first cross product with whichever cardinal axis `normal` is
	// least aligned with keeps it well conditioned (|u| >= 0.43 before
	// normalization) for every possible direction.
	const Math::Vector3df seed = (std::abs(normal.z) < 0.9f)
		? Math::Vector3df(0.0f, 0.0f, 1.0f)
		: Math::Vector3df(1.0f, 0.0f, 0.0f);
	const Math::Vector3df u = glm::normalize(glm::cross(normal, seed));
	const Math::Vector3df v = glm::cross(normal, u);

	std::vector<Math::Vector3df> offsets;
	if (spacing <= 0.0f) {
		offsets.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
		return offsets;
	}

	const int n = static_cast<int>(std::floor(radius / spacing));
	const float radiusSq = radius * radius;
	for (int j = -n; j <= n; ++j) {
		for (int i = -n; i <= n; ++i) {
			const float a = i * spacing;
			const float b = j * spacing;
			if (a * a + b * b > radiusSq) continue;
			offsets.push_back(u * a + v * b);
		}
	}
	// A disk narrower than one spacing still has to emit somewhere.
	if (offsets.empty()) {
		offsets.push_back(Math::Vector3df(0.0f, 0.0f, 0.0f));
	}
	return offsets;
}

/**
 * @brief Returns the next emission offset, cycling through a square lattice
 * that tiles the emitter's disk at the spawned particles' own spacing.
 *
 * Emission positions are deliberately NOT random. Uniform-random sampling of
 * the disk makes the emitted stream a Poisson point process, and a Poisson
 * process clumps: at the correct *mean* number density (one particle per
 * spacing^3) about 41% of particles are born within half a spacing of a
 * neighbor. SPH's rest density is calibrated against a regular lattice, so
 * every one of those pairs starts at rho >> rho0, and WCSPH's
 * p = k*max(0, rho-rho0) blows them apart on the spot -- the jet shreds itself
 * into spray the instant it is created instead of falling as a column
 * (internal design notes section 12 predicted this and
 * proposed stratified sampling; a lattice is the stronger form of the same fix).
 *
 * Cycling through the lattice also produces the correct *three-dimensional*
 * lattice for free, with no extra bookkeeping: one full cycle takes
 * N/rate seconds, and since a correctly configured emitter has
 * rate = pi*R^2*v / spacing^3 while the disk holds N ~ pi*R^2 / spacing^2
 * sites, that is spacing/v seconds -- exactly the time the jet needs to travel
 * one spacing. Successive cycles therefore stack one spacing apart, and the
 * raster ordering within a cycle merely shears the result by a fraction of a
 * degree.
 *
 * @param e      Emitter to draw from (its lattice cache and index are mutated).
 * @param normal Unit-length disk normal (the emitter's normalized direction).
 * @return Offset to add to the emitter's center.
 */
inline Math::Vector3df nextDiskLatticeOffset(Emitter& e, const Math::Vector3df& normal)
{
	const float spacing = e.particleRadius * 2.0f;
	if (e.latticeOffsets.empty()
		|| e.latticeRadius != e.radius
		|| e.latticeSpacing != spacing
		|| e.latticeNormal != normal) {
		e.latticeOffsets = makeDiskLatticeOffsets(e.radius, spacing, normal);
		e.latticeRadius = e.radius;
		e.latticeSpacing = spacing;
		e.latticeNormal = normal;
		e.latticeIndex = 0;
	}

	const int count = static_cast<int>(e.latticeOffsets.size());
	const Math::Vector3df& offset = e.latticeOffsets[e.latticeIndex % count];
	e.latticeIndex = (e.latticeIndex + 1) % count;
	return offset;
}

	}
}
