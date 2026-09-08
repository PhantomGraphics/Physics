#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in float inTemperature;
layout(location = 2) in float inSize;

layout(set = 0, binding = 0) uniform UBO {
    mat4 mvp;
    float pointSize;
    float tMin;
    float tMax;
} ubo;

layout(location = 0) out float outTemperature;

void main() {
    gl_Position = ubo.mvp * vec4(inPos, 1.0);
    gl_PointSize = ubo.pointSize * inSize;
    outTemperature = inTemperature;
}
