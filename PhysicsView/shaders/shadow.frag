#version 450

// The offscreen target's color attachment is a throwaway 1-channel image required by
// VulkanOffscreen but never sampled -- only the depth attachment matters for shadow mapping.
layout(location = 0) out vec4 outColor;

void main() {
    outColor = vec4(0.0);
}
