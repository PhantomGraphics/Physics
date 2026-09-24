#version 450
#extension GL_GOOGLE_include_directive : require

// Opaque PBVR sub-particle (written by flame_pbvr_generate.comp).
layout(location = 0) in vec4 inPosSize; // xyz, w = world diameter
layout(location = 1) in vec4 inColor;

#include "flame_common.glsl"

layout(location = 0) out vec3 outColor;

void main() {
    gl_Position = ubo.mvp * vec4(inPosSize.xyz, 1.0);
    gl_PointSize = flamePointSize(inPosSize.w, gl_Position);
    outColor = inColor.rgb;
}
