#version 450

// Depth-only shadow-caster pass. Node transforms are baked into the mesh, while
// the renderer's per-instance model matrix is applied here just as in the PBR pass.
layout(push_constant) uniform PushConstants {
    mat4 lightVP;
    mat4 model;
} pc;

layout(location = 0) in vec3 inPosition;

void main() {
    gl_Position = pc.lightVP * pc.model * vec4(inPosition, 1.0);
}
