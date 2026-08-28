#version 450

layout(location = 0) in vec3 inPos;

layout(set = 0, binding = 0) uniform UBO {
    mat4 projection;
    mat4 view;       // translation stripped - only rotation
} ubo;

layout(location = 0) out vec3 fragTexCoord;

void main() {
    fragTexCoord = inPos;
    vec4 pos     = ubo.projection * ubo.view * vec4(inPos, 1.0);
    // Place skybox at maximum depth (z = w) so it is always behind everything.
    gl_Position  = pos.xyww;
}
