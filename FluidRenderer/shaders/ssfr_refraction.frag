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

vec3 surfaceNormal(vec2 uv, vec3 center, vec2 px) {
    float dl = texture(uDepth, uv - vec2(px.x, 0.0)).r;
    float dr = texture(uDepth, uv + vec2(px.x, 0.0)).r;
    float dd = texture(uDepth, uv - vec2(0.0, px.y)).r;
    float du = texture(uDepth, uv + vec2(0.0, px.y)).r;

    vec3 dx = dFdx(center);
    if (dl > 0.0 && dr > 0.0) {
        vec3 left  = viewPos(uv - vec2(px.x, 0.0), dl);
        vec3 right = viewPos(uv + vec2(px.x, 0.0), dr);
        dx = 0.5 * (right - left);
    } else if (dl > 0.0) {
        dx = center - viewPos(uv - vec2(px.x, 0.0), dl);
    } else if (dr > 0.0) {
        dx = viewPos(uv + vec2(px.x, 0.0), dr) - center;
    }

    vec3 dy = dFdy(center);
    if (dd > 0.0 && du > 0.0) {
        vec3 down = viewPos(uv - vec2(0.0, px.y), dd);
        vec3 up   = viewPos(uv + vec2(0.0, px.y), du);
        dy = 0.5 * (up - down);
    } else if (dd > 0.0) {
        dy = center - viewPos(uv - vec2(0.0, px.y), dd);
    } else if (du > 0.0) {
        dy = viewPos(uv + vec2(0.0, px.y), du) - center;
    }

    vec3 n = normalize(cross(dx, dy));
    vec3 ray = normalize(center);
    return dot(n, ray) > 0.0 ? -n : n;
}

vec2 limitOffset(vec2 offset, float limit) {
    float m = length(offset);
    return m > limit ? offset * (limit / m) : offset;
}

void main() {
    float d = texture(uDepth, vUV).r;
    float t = texture(uThickness, vUV).r;
    if (d <= 0.0 || t <= 0.0001) discard;
    vec2 px = 2.0 / viewportNearFar.xy;
    vec3 p = viewPos(vUV, d);
    vec3 n = surfaceNormal(vUV, p, px);
    vec3 v = normalize(p);
    vec3 rd = refract(v, n, 1.0 / max(ior, 1.0001));

    // Project an approximate exit ray after travelling through the measured
    // fluid thickness.  Subtracting the unrefracted ray is essential: the old
    // rd.xy-only offset displaced even a flat, front-facing water surface.
    float viewDepth = max(-p.z, 1.0e-3);
    vec2 incidentSlope = v.xy / max(-v.z, 0.05);
    vec2 refractedSlope = rd.xy / max(-rd.z, 0.05);
    vec2 focal = 0.5 / max(abs(vec2(invProj[0][0], invProj[1][1])), vec2(1.0e-4));
    float pathRatio = min(max(t, 0.0), viewDepth * 0.5) / viewDepth;
    vec2 uv = vUV + limitOffset((refractedSlope - incidentSlope) * focal * pathRatio * strength,
                                0.08);
    bool inside = all(greaterThanEqual(uv, vec2(0.001))) && all(lessThanEqual(uv, vec2(0.999)));
    vec3 c;
    if (hasScene != 0 && inside) c = texture(uSceneColor, uv).rgb;
    else if (hasEnvMap != 0) c = texture(uEnvMap, normalize(mat3(invViewRot) * rd)).rgb;
    else if (hasScene != 0) c = texture(uSceneColor, vUV).rgb;
    else c = tint.rgb * (1.0 - d * 0.2);
    outColor = vec4(c, 1.0);
}
