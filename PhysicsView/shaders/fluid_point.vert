#version 450
layout(binding = 0) uniform UBO {
    mat4 mvp;
    vec4 colorParams;
} ubo;
layout(location = 0) in vec4 inPositionDensity;
layout(location = 0) out float densityValue;
layout(location = 1) flat out float useDensity;
void main() {
    gl_Position  = ubo.mvp * vec4(inPositionDensity.xyz, 1.0);
    gl_PointSize = 3.0;
    densityValue = clamp((inPositionDensity.w - ubo.colorParams.x) /
                         max(ubo.colorParams.y - ubo.colorParams.x, 1e-6), 0.0, 1.0);
    useDensity = ubo.colorParams.z;
}
