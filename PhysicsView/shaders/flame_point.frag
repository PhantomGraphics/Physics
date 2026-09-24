#version 450

layout(location = 0) in vec3 inRadiance;
layout(location = 0) out vec4 outColor;

// Additive (ONE/ONE) HDR emission. Alpha is written as 0 so emission never
// counts as coverage -- in a PBVR ensemble target alpha holds the opaque
// (absorbing) coverage only, which the composite uses to attenuate the scene.
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }
    // Increasing-edge smoothstep inverted (plan A5: e0 >= e1 is undefined GLSL).
    float falloff = 1.0 - smoothstep(0.0, 0.25, r2);
    outColor = vec4(inRadiance * falloff, 0.0);
}
