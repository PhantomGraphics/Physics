#version 450

layout(location = 0) in vec3 inColor;
layout(location = 0) out vec4 outColor;

// Hard circular cutout, fully opaque, depth-written: the nearest sub-particle
// wins with no sort (the PBVR premise). Alpha = 1 marks absorbing coverage in
// the ensemble target; the ensemble average of that alpha is 1 - transmittance.
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    if (dot(d, d) > 0.25) {
        discard;
    }
    outColor = vec4(inColor, 1.0);
}
