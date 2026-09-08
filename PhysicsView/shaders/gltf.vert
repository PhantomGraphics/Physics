#version 450

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

void main() {
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
