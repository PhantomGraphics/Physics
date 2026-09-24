#version 450
#extension GL_GOOGLE_include_directive : require

// Emissive flame / spark sprite (Normal mode, and the emission part of every
// PBVR ensemble).
layout(location = 0) in vec3 inPos;
layout(location = 1) in float inTemperature;
layout(location = 2) in float inSize; // world-space diameter

#include "flame_common.glsl"

layout(location = 0) out vec3 outRadiance;

void main() {
    gl_Position = ubo.mvp * vec4(inPos, 1.0);
    gl_PointSize = flamePointSize(inSize, gl_Position);
    // Per-particle constant -> evaluate once per vertex, not per fragment.
    outRadiance = flameEmission(inTemperature);
}
