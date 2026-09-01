#version 450

layout(location = 0) in vec3 inPosition;

layout(set = 0, binding = 0) uniform DepthUBO {
    mat4  proj;
    mat4  modelView;
    float particleRadius;
    float viewportHeight;
};

layout(location = 0) out vec4  vViewPos;
layout(location = 1) out float vParticleRadius;

void main() {
    vViewPos    = modelView * vec4(inPosition, 1.0);
    gl_Position = proj * vViewPos;
    // pointSize はスクリーン空間（px）として扱う
    float viewDistance = max(-vViewPos.z, 1.0e-4);
    gl_PointSize = max(particleRadius * abs(proj[1][1]) * viewportHeight /
                       viewDistance, 1.0);
    vParticleRadius = particleRadius;
}
