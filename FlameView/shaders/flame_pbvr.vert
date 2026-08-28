#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec4 inColor;
layout(location = 2) in float inSize;

layout(set = 0, binding = 0) uniform UBO {
    mat4 mvp;
} ubo;

layout(location = 0) out vec4 fragColor;

// inSize is the actual gl_PointSize (not a multiplier against some uniform point size), since
// this pipeline draws both flame/spark and smoke particles together and they use very different
// base sizes -- see FlamePBVRPipeline's class doc comment.
void main() {
    gl_Position = ubo.mvp * vec4(inPos, 1.0);
    gl_PointSize = inSize;
    fragColor = inColor;
}
