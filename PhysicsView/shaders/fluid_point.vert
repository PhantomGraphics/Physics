#version 450
layout(binding = 0) uniform UBO { mat4 mvp; } ubo;
layout(location = 0) in vec3 inPosition;
void main() {
    gl_Position  = ubo.mvp * vec4(inPosition, 1.0);
    gl_PointSize = 3.0;
}
