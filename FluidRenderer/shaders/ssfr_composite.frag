#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uDepth;
layout(set = 0, binding = 1) uniform sampler2D uThicknessRaw;
layout(set = 0, binding = 2) uniform sampler2D uThicknessSmooth;
layout(set = 0, binding = 3) uniform sampler2D uReflection;
layout(set = 0, binding = 4) uniform sampler2D uRefraction;
layout(set = 0, binding = 5) uniform sampler2D uSpray;
layout(set = 0, binding = 6) uniform sampler2D uFoam;
layout(set = 0, binding = 7) uniform Composite {
    int mode;
    float foamOpacity;
    float sprayOpacity;
    int showSpray;
    int showFoam;
    float _pad0;
    float _pad1;
    float _pad2;
};

void main() {
    float depth = texture(uDepth, vUV).r;
    float thickRaw = texture(uThicknessRaw, vUV).r;
    float thickSmoothed = texture(uThicknessSmooth, vUV).r;
    vec3 refl = texture(uReflection, vUV).rgb;
    vec3 refr = texture(uRefraction, vUV).rgb;
    float spray = texture(uSpray, vUV).r;
    float foam = texture(uFoam, vUV).r;

    if (mode == 0 || mode == 6) {
        if (depth <= 0.0) discard;
        outColor = vec4(vec3(depth), 1.0);
        return;
    }
    if (mode == 1) {
        if (thickRaw <= 0.0001) discard;
        outColor = vec4(vec3(thickRaw * 0.3), 1.0);
        return;
    }
    if (mode == 2) {
        if (thickSmoothed <= 0.0001) discard;
        outColor = vec4(vec3(thickSmoothed * 0.3), 1.0);
        return;
    }
    if (mode == 3) {
        if (thickSmoothed <= 0.0001) discard;
        outColor = vec4(refl, 1.0);
        return;
    }
    if (mode == 4) {
        if (thickSmoothed <= 0.0001) discard;
        outColor = vec4(refr, 1.0);
        return;
    }

    if (thickSmoothed <= 0.0001) discard;
    float mixFactor = clamp(thickSmoothed * 0.12, 0.0, 1.0);
    vec3 color = mix(refr, refl, 0.35 + 0.35 * mixFactor);

    if (showFoam != 0) {
        float foamA = clamp(foam * foamOpacity, 0.0, 1.0);
        color = mix(color, vec3(0.95), foamA);
    }
    if (showSpray != 0) {
        float sprayA = clamp(spray * sprayOpacity, 0.0, 1.0);
        color += vec3(0.95) * sprayA;
    }

    outColor = vec4(color, 1.0);
}
