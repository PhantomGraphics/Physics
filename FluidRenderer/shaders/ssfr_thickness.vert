#version 450

layout(location = 0) in vec3 inPosition;

layout(set = 0, binding = 0) uniform ThicknessUBO {
    mat4  proj;
    mat4  modelView;
    float particleRadius;
    float thicknessScale;
    float viewportHeight;
};

layout(location = 0) out vec4 vViewPos;
layout(location = 1) out float vParticleRadius;
layout(location = 2) out float vThicknessScale;

void main() {
    vViewPos = modelView * vec4(inPosition, 1.0);
    gl_Position = proj * vViewPos;
    float viewDistance = max(-vViewPos.z, 1.0e-4);
    gl_PointSize = max(particleRadius * abs(proj[1][1]) * viewportHeight /
                       viewDistance, 1.0);
    vParticleRadius = particleRadius;
    vThicknessScale = thicknessScale;
}
