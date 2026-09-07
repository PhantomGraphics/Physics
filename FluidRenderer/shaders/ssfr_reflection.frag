#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uDepth;
layout(set = 0, binding = 1) uniform sampler2D uThickness;

layout(set = 0, binding = 2) uniform Params {
    vec4  lightDirection;
    vec4  lightColorIntensity;
    float roughness;
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

vec3 reconstructNormal(vec2 uv, vec3 center, vec2 texelSize) {
    float dl = texture(uDepth, uv - vec2(texelSize.x, 0.0)).r;
    float dr = texture(uDepth, uv + vec2(texelSize.x, 0.0)).r;
    float dd = texture(uDepth, uv - vec2(0.0, texelSize.y)).r;
    float du = texture(uDepth, uv + vec2(0.0, texelSize.y)).r;

    vec3 dx = dFdx(center);
    if (dl > 0.0 && dr > 0.0) {
        vec3 left  = reconstructViewPos(uv - vec2(texelSize.x, 0.0));
        vec3 right = reconstructViewPos(uv + vec2(texelSize.x, 0.0));
        dx = 0.5 * (right - left);
    } else if (dl > 0.0) {
        dx = center - reconstructViewPos(uv - vec2(texelSize.x, 0.0));
    } else if (dr > 0.0) {
        dx = reconstructViewPos(uv + vec2(texelSize.x, 0.0)) - center;
    }

    vec3 dy = dFdy(center);
    if (dd > 0.0 && du > 0.0) {
        vec3 down = reconstructViewPos(uv - vec2(0.0, texelSize.y));
        vec3 up   = reconstructViewPos(uv + vec2(0.0, texelSize.y));
        dy = 0.5 * (up - down);
    } else if (dd > 0.0) {
        dy = center - reconstructViewPos(uv - vec2(0.0, texelSize.y));
    } else if (du > 0.0) {
        dy = reconstructViewPos(uv + vec2(0.0, texelSize.y)) - center;
    }

    vec3 normal = normalize(cross(dx, dy));
    vec3 viewDir = normalize(center);
    return dot(normal, viewDir) > 0.0 ? -normal : normal;
}

void main() {
    float depth = texture(uDepth, vUV).r;
    float thick = texture(uThickness, vUV).r;

    if (depth <= 0.0) discard;

    vec3 viewPos = reconstructViewPos(vUV);
    // A two-pixel derivative footprint suppresses residual sub-particle depth
    // noise without blurring the already filtered silhouette.
    vec2 texelSize = 2.0 / vec2(textureSize(uDepth, 0));
    vec3 normal = reconstructNormal(vUV, viewPos, texelSize);
    vec3 viewDir = normalize(viewPos);

    vec3 reflView = reflect(viewDir, normal);
    vec3 reflWorld = normalize(mat3(invViewRot) * reflView);
    vec3 envColor;

    if (hasEnvMap != 0) {
        envColor = texture(uEnvMap, reflWorld).rgb;
    } else {
        // Built-in neutral outdoor/studio environment.  This keeps Fresnel
        // reflections readable even when no external cubemap is installed.
        float skyMix = smoothstep(-0.25, 0.55, reflWorld.y);
        vec3 ground = vec3(0.10, 0.12, 0.14);
        vec3 sky = mix(vec3(0.38, 0.48, 0.62), vec3(0.82, 0.90, 1.0),
                       clamp(reflWorld.y, 0.0, 1.0));
        vec3 sunDir = normalize(vec3(0.35, 0.75, 0.25));
        float sun = pow(max(dot(reflWorld, sunDir), 0.0), 96.0);
        envColor = mix(ground, sky, skyMix) + vec3(1.0, 0.86, 0.62) * sun;
    }

    // Treat the directional light as a finite bright source in the reflected
    // environment.  This gives otherwise low-frequency cubemaps the crisp
    // specular cue that makes a dielectric surface read as water.
    float highlightPower = mix(192.0, 24.0, clamp(roughness, 0.0, 1.0));
    float highlight = pow(max(dot(reflWorld, normalize(lightDirection.xyz)), 0.0),
                          highlightPower);
    envColor += lightColorIntensity.rgb * lightColorIntensity.a * highlight;

    // Fresnel belongs in the final material composite.  Store N.V in alpha so
    // it is computed from exactly the same filtered surface normal as the
    // reflection direction, instead of approximating it from fluid thickness.
    float nDotV = clamp(dot(normal, -viewDir), 0.0, 1.0);
    outColor = vec4(envColor, nDotV);
}
