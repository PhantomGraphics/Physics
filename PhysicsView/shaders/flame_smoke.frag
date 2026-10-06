#version 450
#extension GL_GOOGLE_include_directive : require
#include "flame_common.glsl"
#include "flame_smoke_profile.glsl"

layout(location = 0) in float inTau;
layout(location = 1) in vec3 inColor;
layout(location = 0) out vec4 outColor;

// Premultiplied-alpha (ONE / ONE_MINUS_SRC_ALPHA) Beer-Lambert sprite: the
// puff uses the same mass-normalized density profile as PBVR. Its projected
// optical depth is integrated analytically (no volume or ray marching). Colour is
// premultiplied by that alpha, so it darkens (absorbs) what is behind it and
// adds its own ambient-lit albedo + thermal glow in the same blend.
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }
    float column = smokeProfileProjectedColumn(4.0*r2,ubo.smoke.w);
    float alpha = 1.0 - exp(-inTau * column);
    outColor = vec4(inColor * alpha, alpha);
}
