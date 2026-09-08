#version 450

layout(location = 0) in float inTemperature;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform UBO {
    mat4 mvp;
    float pointSize;
    float tMin;
    float tMax;
} ubo;

// Simplified blackbody-ish gradient (not physically accurate): dark red at
// low temperature, through orange, to yellow-white at high temperature.
void main() {
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25) {
        discard;
    }

    float t = clamp((inTemperature - ubo.tMin) / max(ubo.tMax - ubo.tMin, 1.0e-4), 0.0, 1.0);

    vec3 cold = vec3(0.35, 0.02, 0.0);
    vec3 mid  = vec3(1.0, 0.55, 0.05);
    vec3 hot  = vec3(1.0, 0.95, 0.75);

    vec3 color = mix(cold, mid, clamp(t * 2.0, 0.0, 1.0));
    color = mix(color, hot, clamp(t * 2.0 - 1.0, 0.0, 1.0));

    // Soft circular falloff; multiplied directly into RGB since additive
    // (ONE/ONE) blending does not use the alpha channel as a weight.
    float falloff = smoothstep(0.25, 0.0, r2);
    outColor = vec4(color * falloff, 1.0);
}
