
layout(set = 0, binding = 0) uniform GlobalUBO {
    mat4  model;
    mat4  view;
    mat4  proj;
    mat4  lightVP;
    vec4  camPos;
    vec4  lightPos;
    vec4  lightColor;
    int   useIBL;
    int   shadowEnabled;
    float shadowBias;
    float shadowStrength;
} cam;

layout(set = 0, binding = 5) uniform BoneUBO {
    mat4 bones[256];
} boneUBO;

layout(location = 0) in vec3  inPosition;
layout(location = 1) in vec3  inNormal;
layout(location = 2) in vec2  inTexCoord;
layout(location = 3) in vec4  inTangent;
layout(location = 4) in ivec4 inJointIndices;
layout(location = 5) in vec4  inJointWeights;

layout(location = 0) out vec3 fragPos;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec2 fragTexCoord;
layout(location = 3) out vec3 fragTangent;
layout(location = 4) out vec3 fragBitangent;
layout(location = 5) out vec4 fragPosLightSpace;

#ifdef COMBUSTIBLE_SCALARS
layout(std430, set = 0, binding = 8) readonly buffer ScalarField { vec4 samples[]; };
layout(location = 6) out vec4 fragScalarColor;

vec4 scalarColor() {
    int mode = clamp(int(samples[0].x), 0, 2);
    int index = clamp(int(inTexCoord.x + 0.5), 0, samples.length() - 2);
    vec4 state = samples[index + 1]; // temperature, fuel mass, initial fuel mass
    float value = mode == 1 ? (state.x - 300.0) / 1200.0 : state.y / max(state.z, 1e-30);
    float level = float(clamp(int(value * 15.0), 0, 15)) / 15.0;
    const vec3 low[3] = vec3[](vec3(0.015), vec3(0.02,0.1,0.6), vec3(0.1,0.01,0.01));
    const vec3 high[3] = vec3[](vec3(0.45,0.22,0.08), vec3(1,0.12,0.01), vec3(0.1,0.7,0.1));
    return vec4(mix(low[mode], high[mode], level), 1.0);
}
#endif

void main() {
#ifdef COMBUSTIBLE_SCALARS
    fragScalarColor = scalarColor();
#endif
    mat4 skinMat =
        inJointWeights.x * boneUBO.bones[inJointIndices.x] +
        inJointWeights.y * boneUBO.bones[inJointIndices.y] +
        inJointWeights.z * boneUBO.bones[inJointIndices.z] +
        inJointWeights.w * boneUBO.bones[inJointIndices.w];

    vec4 worldPos = cam.model * skinMat * vec4(inPosition, 1.0);
    fragPos = worldPos.xyz;
    fragPosLightSpace = cam.lightVP * worldPos;

    mat3 normalMatrix = transpose(inverse(mat3(cam.model) * mat3(skinMat)));
    fragNormal = normalize(normalMatrix * inNormal);

    vec3 T = normalize(normalMatrix * inTangent.xyz);
    fragTangent = T;
    fragBitangent = cross(fragNormal, T) * inTangent.w;

    fragTexCoord = inTexCoord;
    gl_Position = cam.proj * cam.view * worldPos;
}
