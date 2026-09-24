#version 450
#extension GL_GOOGLE_include_directive : require

// Absorbing soot-smoke sprite (Normal mode).
layout(location = 0) in vec3 inPos;
layout(location = 1) in float inDensity; // soot optical density (envelope * soot), >= 0
layout(location = 2) in float inSize;    // world-space diameter
layout(location = 3) in float inTemperature;

#include "flame_common.glsl"

layout(location = 0) out float outTau;   // optical depth through the sprite centre
layout(location = 1) out vec3 outColor;  // re-emitted colour per unit opacity

void main() {
    gl_Position = ubo.mvp * vec4(inPos, 1.0);
    gl_PointSize = flamePointSize(inSize, gl_Position);
    outTau = ubo.smoke.x * inDensity;
    outColor = smokeColor(inTemperature);
}
