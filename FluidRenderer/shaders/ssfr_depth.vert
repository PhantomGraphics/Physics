#version 450

layout(location = 0) in vec4 inPosition;

layout(set = 0, binding = 0) uniform DepthUBO {
    mat4  proj;
    mat4  modelView;
    float particleRadius;
    float viewportHeight;
};

layout(location = 0) out vec4  vViewPos;
layout(location = 1) out float vParticleRadius;

void main() {
    // Negative uniform selects the external buffer radius; CPU/GPU_CSPH retain uniform sizing.
    float radius = particleRadius < 0.0 ? -particleRadius * inPosition.w : particleRadius;
    vViewPos    = modelView * vec4(inPosition.xyz, 1.0);
    gl_Position = proj * vViewPos;
    // pointSize はスクリーン空間（px）として扱う
    float viewDistance = abs(proj[3][3]) > 0.5 ? 1.0 : max(-vViewPos.z, 1.0e-4);
    gl_PointSize = max(radius * abs(proj[1][1]) * viewportHeight /
                       viewDistance, 1.0);
    vParticleRadius = radius;
}
