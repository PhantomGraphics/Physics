#version 450

// set=0: global (per-frame)
layout(set = 0, binding = 0) uniform GlobalUBO {
    mat4  model;
    mat4  view;
    mat4  proj;
    mat4  lightVP;
    vec4  camPos;
    vec4  lightPos;    // w=0: directional, w=1: point
    vec4  lightColor;  // w=intensity
    int   useIBL;
    int   shadowEnabled;
    float shadowBias;
    float shadowStrength;
} cam;
layout(set = 0, binding = 1) uniform samplerCube irradianceMap;
layout(set = 0, binding = 2) uniform samplerCube prefilteredEnvMap;
layout(set = 0, binding = 3) uniform sampler2D   brdfLUT;
layout(set = 0, binding = 4) uniform sampler2D   shadowMap;

// set=1: per-material
layout(set = 1, binding = 0) uniform MaterialUBO {
    vec4  baseColorFactor;
    float metallicFactor;
    float roughnessFactor;
    float normalScale;
    float occlusionStrength;
    vec3  emissiveFactor;
    int   hasBaseColorTex;
    int   hasMetallicRoughnessTex;
    int   hasNormalTex;
    int   hasOcclusionTex;
    int   hasEmissiveTex;
} mat;

layout(set = 1, binding = 1) uniform sampler2D baseColorTex;
layout(set = 1, binding = 2) uniform sampler2D metallicRoughnessTex;
layout(set = 1, binding = 3) uniform sampler2D normalTex;
layout(set = 1, binding = 4) uniform sampler2D occlusionTex;
layout(set = 1, binding = 5) uniform sampler2D emissiveTex;

layout(location = 0) in vec3 fragPos;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec3 fragTangent;
layout(location = 4) in vec3 fragBitangent;
layout(location = 5) in vec4 fragPosLightSpace;

layout(location = 0) out vec4 outColor;

const float PI = 3.14159265358979323846;
const float MAX_REFLECTION_LOD = 4.0;

float distributionGGX(float NdotH, float roughness) {
    float a  = roughness * roughness;
    float a2 = a * a;
    float d  = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

float geometrySmith(float NdotV, float NdotL, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    float gv = NdotV / (NdotV * (1.0 - k) + k);
    float gl = NdotL / (NdotL * (1.0 - k) + k);
    return gv * gl;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// PCF (3x3) shadow lookup. Returns 0 = fully lit, 1 = fully occluded.
float computeShadow(vec4 posLightSpace, float NdotL) {
    vec3 proj = posLightSpace.xyz / posLightSpace.w;
    proj.xy = proj.xy * 0.5 + 0.5; // NDC -> [0,1] UV (GLM_FORCE_DEPTH_ZERO_TO_ONE: z already [0,1])

    if (proj.z > 1.0 || any(lessThan(proj.xy, vec2(0.0))) || any(greaterThan(proj.xy, vec2(1.0))))
        return 0.0; // outside the light's frustum -- treat as unshadowed

    float bias = max(cam.shadowBias * (1.0 - NdotL), cam.shadowBias * 0.1);
    vec2 texel = 1.0 / vec2(textureSize(shadowMap, 0));

    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            float closestDepth = texture(shadowMap, proj.xy + vec2(x, y) * texel).r;
            shadow += (proj.z - bias > closestDepth) ? 1.0 : 0.0;
        }
    }
    return (shadow / 9.0) * cam.shadowStrength;
}

void main() {
    // Base color
    vec4 baseColor = mat.baseColorFactor;
    if (mat.hasBaseColorTex != 0)
        baseColor *= texture(baseColorTex, fragTexCoord);

    if (baseColor.a < 0.01) discard;

    // Metallic-roughness
    float metallic  = mat.metallicFactor;
    float roughness = mat.roughnessFactor;
    if (mat.hasMetallicRoughnessTex != 0) {
        vec4 mr = texture(metallicRoughnessTex, fragTexCoord);
        roughness *= mr.g;
        metallic  *= mr.b;
    }
    roughness = clamp(roughness, 0.04, 1.0);

    // Normal
    vec3 N = normalize(fragNormal);
    if (mat.hasNormalTex != 0) {
        vec3 tn = texture(normalTex, fragTexCoord).xyz * 2.0 - 1.0;
        tn.xy  *= mat.normalScale;
        mat3 TBN = mat3(normalize(fragTangent), normalize(fragBitangent), N);
        N = normalize(TBN * tn);
    }

    vec3 V = normalize(cam.camPos.xyz - fragPos);
    vec3 R = reflect(-V, N);

    // Light direction (w=0: directional, w=1: point)
    vec3 L;
    if (cam.lightPos.w == 0.0) {
        L = normalize(cam.lightPos.xyz);
    } else {
        L = normalize(cam.lightPos.xyz - fragPos);
    }
    vec3 H = normalize(V + L);

    float NdotV = max(dot(N, V), 0.0001);
    float NdotL = max(dot(N, L), 0.0);
    float NdotH = max(dot(N, H), 0.0);
    float VdotH = max(dot(V, H), 0.0);

    vec3 albedo = baseColor.rgb;
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    // Cook-Torrance BRDF
    float D = distributionGGX(NdotH, roughness);
    float G = geometrySmith(NdotV, NdotL, roughness);
    vec3  F = fresnelSchlick(VdotH, F0);

    vec3 numerator = D * G * F;
    float denominator = 4.0 * NdotV * NdotL + 0.0001;
    vec3 specular = numerator / denominator;

    vec3 kD = (vec3(1.0) - F) * (1.0 - metallic);
    float shadow = (cam.shadowEnabled != 0) ? computeShadow(fragPosLightSpace, NdotL) : 0.0;
    vec3 Lo = (kD * albedo / PI + specular) * cam.lightColor.rgb * NdotL * (1.0 - shadow);

    // Ambient
    vec3 ambient = vec3(0.03) * albedo;

    // IBL
    if (cam.useIBL == 1) {
        vec3 F_ibl  = fresnelSchlickRoughness(NdotV, F0, roughness);
        vec3 kD_ibl = (vec3(1.0) - F_ibl) * (1.0 - metallic);
        vec3 irradiance      = texture(irradianceMap, N).rgb;
        vec3 diffuseIBL      = kD_ibl * irradiance * albedo;
        vec2 brdf            = texture(brdfLUT, vec2(NdotV, roughness)).rg;
        vec3 prefilteredColor = textureLod(prefilteredEnvMap, R, roughness * MAX_REFLECTION_LOD).rgb;
        vec3 specularIBL     = prefilteredColor * (F_ibl * brdf.x + brdf.y);
        ambient = diffuseIBL + specularIBL;
    }

    // Occlusion
    if (mat.hasOcclusionTex != 0) {
        float ao = texture(occlusionTex, fragTexCoord).r;
        ambient = mix(ambient, ambient * ao, mat.occlusionStrength);
    }

    // Emissive
    vec3 emissive = mat.emissiveFactor;
    if (mat.hasEmissiveTex != 0)
        emissive *= texture(emissiveTex, fragTexCoord).rgb;

    vec3 color = ambient + Lo + emissive;

    // Tone mapping (Reinhard)
    color = color / (color + vec3(1.0));
    // Gamma correction
    color = pow(color, vec3(1.0 / 2.2));

    outColor = vec4(color, baseColor.a);
}
