#version 450
#extension GL_GOOGLE_include_directive : require

// Heat-haze refraction field (screen space, low resolution): every hot particle
// splats "temperature excess" into a small offscreen target. Additive blending
// sums the excess along each view ray -- a cheap stand-in for the line integral
// of the refractive-index gradient that produces heat shimmer.
layout(location = 0) in vec3 inPos;
layout(location = 1) in float inTemperature;
layout(location = 2) in float inSize; // world-space diameter

#include "flame_common.glsl"

layout(location = 0) out float outHeat;
layout(location = 1) out float outDepth;

void main() {
    gl_Position = ubo.mvp * vec4(inPos, 1.0);
    // ubo.smoke.w = haze extent: the haze reaches beyond the visible flame body.
    gl_PointSize = flamePointSize(inSize * ubo.smoke.w, gl_Position);
    float excess = (inTemperature - ubo.thermal.x) / max(ubo.thermal.y - ubo.thermal.x, 1.0);
    outHeat = clamp(excess, 0.0, 1.5) * ubo.thermal.w;
    outDepth = gl_Position.z / max(gl_Position.w, 1.0e-4);
}
