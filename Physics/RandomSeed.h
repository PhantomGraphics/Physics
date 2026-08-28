#pragma once

/**
 * @file RandomSeed.h
 * @brief The default seed every stochastic subsystem in Physics starts from,
 * and the reason it is a fixed number rather than std::random_device.
 *
 * The emitters (WCSPHFluid/DFSPHFluid/PBSPHFluid::updateEmitters()'s speed
 * jitter, FlameFluid's spawn disk / spark / smoke sampling) and
 * WhiteWaterSystem's spray/foam lifetimes all used to seed from
 * std::random_device{}(), which made every scene that uses them
 * unreproducible: the same scene, run twice in the same process, diverged.
 *
 * That is a correctness problem, not a cosmetic one:
 *
 *  - A showcase bake could not be repeated. Re-running a 240-frame sequence
 *    produced different footage, so a render could not be reproduced after a
 *    crash, and a bake could not be split across machines.
 *  - It silently invalidated A/B measurements. Measured spread on the
 *    water-sphere showcase (speedJitter = 0.02) was ~10% on peak speed and
 *    ~5% on rho_max -- larger than most of the parameter differences that
 *    investigation was trying to resolve. See
 *    docs/issue/water_sphere_showcase_emitter_instability.md section 11.4.
 *  - FluidDeterministicTest never caught it, because none of its scenes used
 *    an emitter.
 *
 * Deterministic-by-default matches how the rest of this library already
 * behaves (see Physics/CLAUDE.md on the gather-based neighbor passes, which
 * go out of their way to keep floating-point summation order fixed so repeat
 * runs are bit-identical).
 *
 * Callers that actually want a different draw -- several takes of the same
 * shot, or a Monte-Carlo sweep -- ask for it explicitly via the owning
 * class's setRandomSeed(), rather than getting it whether they wanted it or
 * not. Passing std::random_device{}() there restores the old behavior.
 */

namespace Phantom {
namespace Physics {

/**
 * @brief Fixed default seed for every std::mt19937 in this library.
 *
 * The value itself is arbitrary (it is the date this was introduced); only
 * its being constant matters.
 */
inline constexpr unsigned int kDefaultRandomSeed = 20260827u;

}
}
