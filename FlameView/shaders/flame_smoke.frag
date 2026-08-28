#version 450

layout(location = 0) in float inOpacity;
layout(location = 1) in float inTemperature;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform UBO {
    mat4 mvp;
    vec4 smokeColor;
    float pointSize;
    float tMin;
    float tMax;
} ubo;

// Soft round soot-smoke sprite, standard (non-additive) alpha blended so it
// can visually darken/obscure what is behind it, unlike the flame's additive
// glow. Not physically accurate (no scattering/absorption) -- purely a
// cosmetic secondary particle, see FlameFluid::updateSecondaryParticles().
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }

    // Smoke inherits its source primary particle's temperature at spawn and
    // cools toward ambient over its first second (see
    // FlameFluid::updateSecondaryParticles()); tint it toward a warm ember
    // glow while still hot so freshly emitted smoke reads as coming off the
    // flame front, fading to the flat sooty smokeColor as it cools.
    float t = clamp((inTemperature - ubo.tMin) / max(ubo.tMax - ubo.tMin, 1.0e-4), 0.0, 1.0);
    vec3 emberGlow = vec3(1.0, 0.45, 0.12);
    vec3 tint = mix(ubo.smokeColor.rgb, emberGlow, t * t);

    // Non-reversed edge order (0.0 < 0.25), then inverted: smoothstep with
    // edge0 > edge1 is undefined per the GLSL spec. The flame's additive
    // shader gets away with the reversed form (a bad value just clamps
    // harmlessly into ADD blending), but it corrupted the alpha channel here
    // under real src-alpha blending -- see FlameSmokePipeline's doc comment
    // on why this pipeline is non-additive.
    float falloff = 1.0 - smoothstep(0.0, 0.25, r2);
    outColor = vec4(tint, inOpacity * falloff);
}
