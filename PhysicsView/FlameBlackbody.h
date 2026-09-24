#pragma once

#include <glm/glm.hpp>

#include <array>

namespace Phantom {

/**
 * @brief Blackbody (Planck) color and radiance for the Flame renderer
 * (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phase 3).
 *
 * Replaces the hand-written cold/mid/hot LDR gradient that used to live in
 * flame_point.frag *and* in a CPU copy in FluidApp.cpp. Vulkan-independent so
 * PhysicsTest can check it; the shaders read it through a small LUT uploaded
 * in FlamePointUBO (see makeLut()).
 *
 * Color: Planck spectral radiance integrated against the CIE 1931 2-degree
 * color-matching functions (Wyman, Sloan & Shirley 2013 multi-lobe analytic
 * fit), 380-780 nm, then XYZ -> linear sRGB (D65). Negative (out-of-gamut)
 * components are clipped and the result renormalized to unit luminance, so
 * the color carries chromaticity only.
 *
 * White balance (chromatic adaptation): optionally, Bradford adaptation from
 * the white point of a blackbody at `whiteTemperature` to D65, with an
 * adaptation degree D in [0, 1] (CIECAM02-style incomplete adaptation: 0 =
 * none, 1 = full). Without it a ~1500 K flame is nearly pure sRGB red, which
 * is colorimetrically right for a D65 display but not how an eye or a camera
 * set to the fire's light sees it; adapting to the flame's own hottest
 * temperature renders that as white and cooler gas as orange/red.
 *
 * Brightness: relative total radiant exitance (Stefan-Boltzmann),
 * (T^4 - Ta^4) / (Tref^4 - Ta^4), clamped at 0 -- 1 at the reference
 * temperature, and allowed to exceed 1 above it (HDR; the ACES tone mapper
 * downstream rolls the hottest core off to white).
 */
namespace FlameBlackbody {

/** @brief CIE XYZ of a blackbody at temperature (K), normalized to Y = 1. */
glm::vec3 xyz(float temperature);

/**
 * @brief Unit-luminance linear-sRGB chromaticity of a blackbody at temperature (K),
 * optionally white-balanced: whiteTemperature > 0 adapts (Bradford) the white of
 * a blackbody at that temperature toward D65 by `degree` (0..1).
 */
glm::vec3 chromaticity(float temperature, float whiteTemperature = 0.0f, float degree = 1.0f);

/** @brief (T^4 - Ta^4) / (Tref^4 - Ta^4), >= 0. Tref <= Ta yields 0. */
float relativeRadiance(float temperature, float ambient, float reference);

/** @brief Rec.709 luminance of a linear-sRGB color. */
inline float luminance(const glm::vec3& c) { return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b; }

constexpr int kLutSize = 64;
constexpr float kLutMinT = 500.0f;
constexpr float kLutMaxT = 4000.0f;

/** @brief chromaticity() sampled at kLutSize evenly spaced temperatures in [kLutMinT, kLutMaxT] (w = 1). */
std::array<glm::vec4, kLutSize> makeLut(float whiteTemperature = 0.0f, float degree = 1.0f);

} // namespace FlameBlackbody

} // namespace Phantom
