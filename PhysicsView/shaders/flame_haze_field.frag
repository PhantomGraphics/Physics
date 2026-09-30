#version 450

layout(location = 0) in float inHeat;
layout(location = 1) in float inDepth;
layout(location = 0) out vec4 outColor;

// Additive (ONE/ONE). r = accumulated heat, g = heat-weighted NDC depth (so
// g / r is the mean depth of the hot gas: geometry nearer than that is in front
// of the flame and must not be refracted by it).
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }
    float w = inHeat * (1.0 - smoothstep(0.0, 0.25, r2));
    outColor = vec4(w, w * inDepth, 0.0, 0.0);
}
