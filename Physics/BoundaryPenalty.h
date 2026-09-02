#pragma once

namespace Phantom {
	namespace Physics {

/**
 * @brief Clamps a boundary damping ratio to the range the explicit penalty
 * integrator can actually take.
 *
 * The penalty spring PlaneBoundary/SphereBoundary apply has stiffness
 * k/m = 1/dt^2, i.e. omega*dt == 1: contact lasts only ~pi steps, so the
 * damper's per-step velocity change is 2*zeta*|v_n|. At zeta = 0.5 that
 * removes the whole normal velocity component in a single step; anything
 * beyond that *reverses* it and re-introduces the very bounce the damper
 * exists to remove. 0.5 is therefore a hard ceiling, not a taste parameter.
 * @param ratio Requested damping ratio.
 * @return ratio clamped to [0, 0.5].
 */
inline float clampBoundaryDampingRatio(const float ratio)
{
	if (!(ratio > 0.0f)) return 0.0f;   // also maps NaN to 0
	return (ratio > 0.5f) ? 0.5f : ratio;
}

/**
 * @brief Signed magnitude (along the boundary's inward normal) of the penalty
 * acceleration for a penetrating particle.
 *
 * The spring term alone (-d/dt^2) is a *conservative* position-restoring
 * impulse: it stores the penetration as spring energy and gives every bit of
 * it back, so the wall behaves as a trampoline with restitution ~1 (measured
 * 1.03 -- semi-implicit Euler at omega*dt == 1 adds a little). That is
 * invisible under a settled pool, where the weight of the water above and the
 * viscous coupling to it eat the rebound, but a jet hitting an *empty*
 * container has nothing above it, so the whole rebound turns into upward
 * spray (internal design notes section 3).
 *
 * The damper term removes that energy instead: it opposes motion along the
 * normal while the particle is in contact, giving an explicitly controllable
 * restitution rather than an accidental 1.0. dampingRatio == 0 reproduces the
 * historical pure-spring behavior exactly, which is why it is the default
 * everywhere.
 *
 * The result is clamped to be non-negative (i.e. inward-only): late in a
 * contact the damper can exceed the spring, and a negative value would mean
 * the wall *pulling* the particle further out through itself. A wall may push;
 * it may never pull.
 *
 * @param signedDistance Boundary signed distance at the particle (< 0 when penetrating).
 * @param normalVelocity Particle velocity projected onto the inward normal
 *        (negative while the particle is still moving into the wall).
 * @param timeStep       Time step used to scale the penalty.
 * @param dampingRatio   Damping ratio; see clampBoundaryDampingRatio().
 * @return Non-negative acceleration magnitude along the inward normal.
 */
inline float boundaryPenaltyAcceleration(const float signedDistance, const float normalVelocity,
                                          const float timeStep, const float dampingRatio)
{
	const float spring = -signedDistance / (timeStep * timeStep);
	const float damper = -2.0f * clampBoundaryDampingRatio(dampingRatio) * normalVelocity / timeStep;
	const float acceleration = spring + damper;
	return (acceleration > 0.0f) ? acceleration : 0.0f;
}

	}
}
