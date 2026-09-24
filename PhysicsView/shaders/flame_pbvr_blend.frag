#version 450

// Folds one PBVR ensemble image into the running average:
//   history_next = mix(history_prev, ensemble, weight)
// weight = 1/(n+1) gives the exact progressive mean of n+1 ensembles (static
// frame); a capped n turns it into an exponential moving average while the
// simulation is running (FlameRenderer::recordPBVR()).
layout(set = 0, binding = 0) uniform sampler2D ensembleImage;
layout(set = 0, binding = 1) uniform sampler2D historyImage;

layout(push_constant) uniform PC {
    float weight;
} pc;

layout(location = 0) out vec4 outColor;

void main() {
    ivec2 p = ivec2(gl_FragCoord.xy);
    vec4 e = texelFetch(ensembleImage, p, 0);
    if (pc.weight >= 1.0) {
        // First sample: never read the history -- after a reset/resize it is
        // uninitialized memory, and mix(NaN, e, 1) is still NaN.
        outColor = e;
        return;
    }
    vec4 h = texelFetch(historyImage, p, 0);
    outColor = mix(h, e, pc.weight);
}
