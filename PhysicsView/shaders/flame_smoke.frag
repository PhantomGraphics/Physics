#version 450

layout(location = 0) in float inTau;
layout(location = 1) in vec3 inColor;
layout(location = 0) out vec4 outColor;

// Premultiplied-alpha (ONE / ONE_MINUS_SRC_ALPHA) Beer-Lambert sprite: the
// puff is a uniform sphere, so the path length through it at radius r is its
// chord sqrt(1 - (2r)^2) -- alpha = 1 - exp(-tau * chord). Colour is
// premultiplied by that alpha, so it darkens (absorbs) what is behind it and
// adds its own ambient-lit albedo + thermal glow in the same blend.
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }
    float chord = sqrt(max(0.0, 1.0 - 4.0 * r2));
    float alpha = 1.0 - exp(-inTau * chord);
    outColor = vec4(inColor * alpha, alpha);
}
