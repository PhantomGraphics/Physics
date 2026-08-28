#version 450

layout(location = 0) in vec3 inPosition;

layout(set = 0, binding = 0) uniform ThicknessUBO {
    mat4  proj;
    mat4  modelView;
    float pointSize;
    float thicknessScale;
};

layout(location = 0) out vec4 vViewPos;
layout(location = 1) out float vPointSize;
layout(location = 2) out float vThicknessScale;

void main() {
    vViewPos = modelView * vec4(inPosition, 1.0);
    gl_Position = proj * vViewPos;
    gl_PointSize = max(pointSize, 1.0);
    vPointSize = gl_PointSize;
    vThicknessScale = thicknessScale;
}
