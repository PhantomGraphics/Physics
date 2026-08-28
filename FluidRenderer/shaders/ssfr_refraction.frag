#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uDepth;
layout(set = 0, binding = 1) uniform sampler2D uThickness;
layout(set = 0, binding = 2) uniform Params {
    vec4 tint;
    float strength;
};

void main() {
    float depth = texture(uDepth, vUV).r;
    float thick = texture(uThickness, vUV).r;
    float f = clamp(thick * 0.18 * strength, 0.0, 1.0);
    float absorb = exp(-thick * 0.06);
    vec3 color = tint.rgb * mix(0.25, 1.0, f) * mix(0.75, 1.0, absorb) * (1.0 - depth * 0.2);
    outColor = vec4(color, 1.0);
}
