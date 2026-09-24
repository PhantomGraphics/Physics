#version 450

// Composites the averaged PBVR image over the HDR scene with premultiplied
// alpha: rgb already holds E[opaque colour * coverage] + E[visible emission],
// alpha holds E[coverage] = 1 - transmittance of the smoke.
layout(set = 0, binding = 0) uniform sampler2D historyImage;

layout(location = 0) out vec4 outColor;

void main() {
    outColor = texelFetch(historyImage, ivec2(gl_FragCoord.xy), 0);
}
