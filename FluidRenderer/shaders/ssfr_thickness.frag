#version 450

layout(location = 0) in vec4 vViewPos;
layout(location = 1) in float vParticleRadius;
layout(location = 2) in float vThicknessScale;
layout(location = 0) out float outThickness;

void main() {
    vec2 coord = gl_PointCoord * 2.0 - 1.0;
    float r2 = dot(coord, coord);
    if (r2 > 1.0) discard;

    float sphereZ = sqrt(1.0 - r2);
    outThickness = 2.0 * vParticleRadius * sphereZ * vThicknessScale;
}
