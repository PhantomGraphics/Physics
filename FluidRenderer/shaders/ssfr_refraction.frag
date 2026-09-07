#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;
layout(set = 0, binding = 0) uniform sampler2D uDepth;
layout(set = 0, binding = 1) uniform sampler2D uThickness;
layout(set = 0, binding = 2) uniform Params {
    vec4 tint; float strength; int hasScene; int hasEnvMap; float ior;
    mat4 invProj; mat4 invViewRot; vec4 viewportNearFar; vec4 absorptionColor;
};
layout(set = 0, binding = 3) uniform sampler2D uSceneColor;
layout(set = 0, binding = 4) uniform sampler2D uSceneDepth;
layout(set = 0, binding = 5) uniform samplerCube uEnvMap;

vec3 viewPos(vec2 uv, float d) {
    vec4 h = invProj * vec4(uv * 2.0 - 1.0, d, 1.0);
    return h.xyz / max(abs(h.w), 1.0e-6);
}
void main() {
    float d = texture(uDepth, vUV).r;
    float t = texture(uThickness, vUV).r;
    if (d <= 0.0 || t <= 0.0001) discard;
    vec2 px = 1.0 / viewportNearFar.xy;
    vec3 p = viewPos(vUV, d);
    float dr = texture(uDepth, vUV + vec2(px.x, 0.0)).r;
    float du = texture(uDepth, vUV + vec2(0.0, px.y)).r;
    vec3 dx = dr > 0.0 ? viewPos(vUV + vec2(px.x, 0.0), dr) - p : dFdx(p);
    vec3 dy = du > 0.0 ? viewPos(vUV + vec2(0.0, px.y), du) - p : dFdy(p);
    vec3 n = normalize(cross(dx, dy));
    vec3 v = normalize(p);
    if (dot(n, v) > 0.0) n = -n;
    vec3 rd = refract(v, n, 1.0 / max(ior, 1.0001));
    vec2 uv = vUV + rd.xy * (0.012 * strength) * clamp(t, 0.0, 8.0);
    bool inside = all(greaterThanEqual(uv, vec2(0.001))) && all(lessThanEqual(uv, vec2(0.999)));
    bool continuous = false;
    if (hasScene != 0 && inside) {
        float a = texture(uSceneDepth, vUV).r;
        float b = texture(uSceneDepth, uv).r;
        continuous = abs(b - a) < 0.08 || b >= 0.9999;
    }
    vec3 c;
    if (hasScene != 0 && inside && continuous) c = texture(uSceneColor, uv).rgb;
    else if (hasEnvMap != 0) c = texture(uEnvMap, normalize(mat3(invViewRot) * rd)).rgb;
    else if (hasScene != 0) c = texture(uSceneColor, vUV).rgb;
    else c = tint.rgb * (1.0 - d * 0.2);
    outColor = vec4(c, 1.0);
}
