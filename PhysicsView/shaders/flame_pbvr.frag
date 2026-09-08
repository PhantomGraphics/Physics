#version 450

layout(location = 0) in vec4 fragColor;
layout(location = 0) out vec4 outColor;

// Hard circular cutout only, no soft alpha falloff -- this pipeline is opaque
// (blendEnable=false, see FlamePBVRPipeline), so alpha is never used as a blend weight;
// baking a soft edge into it would do nothing but darken a ring at the edge. This matches
// the established Phantom::Volume PBVR fragment shader's technique (pbvr_render.frag).
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    if (dot(d, d) > 0.25) {
        discard;
    }
    outColor = vec4(fragColor.rgb, 1.0);
}
