#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uDepth;
layout(set = 0, binding = 1) uniform sampler2D uThickness;

layout(set = 0, binding = 2) uniform Params {
    vec4  tint;
    float strength;
    int   hasEnvMap;
    float _p0;
    float _p1;
    mat4  invProj;
    mat4  invViewRot;
};

layout(set = 0, binding = 3) uniform samplerCube uEnvMap;

vec3 reconstructViewPos(vec2 uv) {
    float d = texture(uDepth, uv).r;
    vec4 ndc = vec4(uv * 2.0 - 1.0, d, 1.0);
    vec4 viewH = invProj * ndc;
    return viewH.xyz / viewH.w;
}

void main() {
    float depth = texture(uDepth, vUV).r;
    float thick = texture(uThickness, vUV).r;

    if (depth <= 0.0) discard;

    float f = clamp(thick * 0.22 * strength, 0.0, 1.0);
    vec3 viewPos = reconstructViewPos(vUV);
    vec2 texelSize = 1.0 / vec2(textureSize(uDepth, 0));
    vec3 posR = reconstructViewPos(vUV + vec2(texelSize.x, 0.0));
    vec3 posU = reconstructViewPos(vUV + vec2(0.0, texelSize.y));

    // Do not form normals across the fluid silhouette: invalid neighbours
    // otherwise create bright, unstable streaks at the boundary.
    float depthR = texture(uDepth, vUV + vec2(texelSize.x, 0.0)).r;
    float depthU = texture(uDepth, vUV + vec2(0.0, texelSize.y)).r;
    vec3 dx = depthR > 0.0 ? posR - viewPos : dFdx(viewPos);
    vec3 dy = depthU > 0.0 ? posU - viewPos : dFdy(viewPos);
    vec3 normal = normalize(cross(dx, dy));
    vec3 viewDir = normalize(viewPos);
    if (dot(normal, viewDir) > 0.0) normal = -normal;

    vec3 reflView = reflect(viewDir, normal);
    vec3 reflWorld = normalize(mat3(invViewRot) * reflView);
    vec3 color;

    if (hasEnvMap != 0) {
        vec3  envColor = texture(uEnvMap, reflWorld).rgb;
        float fresnel  = pow(1.0 - clamp(-dot(viewDir, normal), 0.0, 1.0), 3.0);
        color = mix(tint.rgb, envColor, fresnel) * mix(0.35, 1.0, f);
    } else {
        // Built-in neutral outdoor/studio environment.  This keeps Fresnel
        // reflections readable even when no external cubemap is installed.
        float skyMix = smoothstep(-0.25, 0.55, reflWorld.y);
        vec3 ground = vec3(0.10, 0.12, 0.14);
        vec3 sky = mix(vec3(0.38, 0.48, 0.62), vec3(0.82, 0.90, 1.0),
                       clamp(reflWorld.y, 0.0, 1.0));
        vec3 sunDir = normalize(vec3(0.35, 0.75, 0.25));
        float sun = pow(max(dot(reflWorld, sunDir), 0.0), 96.0);
        vec3 envColor = mix(ground, sky, skyMix) + vec3(1.0, 0.86, 0.62) * sun;
        float fresnel = 0.02 + 0.98 * pow(1.0 - clamp(-dot(viewDir, normal), 0.0, 1.0), 5.0);
        color = mix(tint.rgb * mix(0.45, 0.9, f), envColor, fresnel);
    }

    outColor = vec4(color, 1.0);
}
