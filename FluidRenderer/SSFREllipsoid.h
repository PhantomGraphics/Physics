#pragma once

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include "SSFRPipeline.h"

#include <vector>

namespace Phantom {

// Anisotropic-kernel (Yu & Turk 2013) ellipsoid splats, shared by the depth
// and thickness passes (shaders/ssfr_aniso.vert + ssfr_{depth,thickness}_aniso.frag).
//
// Vertex input: binding 0 = per-particle centre (vec4: smoothed position,
// world radius), binding 1 = per-particle axes (3 x vec4, xyz = columns of
// T = R diag(sigma), dimensionless). See Phantom::Physics::computeAnisotropy().
struct SSFREllipsoidUBO {
    glm::mat4 proj;
    glm::mat4 invProj;
    glm::mat4 modelView;
    glm::vec4 params;   // x = radius scale, y = viewport w, z = viewport h, w = max point size
    glm::vec4 params2;  // x = thickness scale
};
static_assert(sizeof(SSFREllipsoidUBO) == 224, "UBO layout mismatch");

// One particle's axes as uploaded to binding 1.
struct SSFREllipsoidAxes {
    glm::vec4 axis[3];
};
static_assert(sizeof(SSFREllipsoidAxes) == 48, "axes layout mismatch");

// Buffers + count of one frame's ellipsoid draw.
struct SSFREllipsoidDraw {
    VkBuffer centers = VK_NULL_HANDLE;
    VkBuffer axes    = VK_NULL_HANDLE;
    uint32_t count   = 0;
    float    radiusScale  = 1.5f;
    float    maxPointSize = 64.0f;
    VkExtent2D extent{ 1, 1 };

    bool valid() const { return count > 0 && centers != VK_NULL_HANDLE && axes != VK_NULL_HANDLE; }
};

inline SSFRPassConfig makeEllipsoidPassConfig(std::vector<uint32_t> vertSpv,
                                              std::vector<uint32_t> fragSpv,
                                              uint32_t framesInFlight)
{
    VkVertexInputBindingDescription centerBinding{};
    centerBinding.binding   = 0;
    centerBinding.stride    = sizeof(glm::vec4);
    centerBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    VkVertexInputBindingDescription axesBinding{};
    axesBinding.binding   = 1;
    axesBinding.stride    = sizeof(SSFREllipsoidAxes);
    axesBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::vector<VkVertexInputAttributeDescription> attrs(4);
    attrs[0] = { 0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0 };
    for (uint32_t i = 0; i < 3; ++i)
        attrs[1 + i] = { 1 + i, 1, VK_FORMAT_R32G32B32A32_SFLOAT, i * static_cast<uint32_t>(sizeof(glm::vec4)) };

    VkDescriptorSetLayoutBinding uboBinding{};
    uboBinding.binding         = 0;
    uboBinding.descriptorType  = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    uboBinding.descriptorCount = 1;
    uboBinding.stageFlags      = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    SSFRPassConfig cfg;
    cfg.vertSpv            = std::move(vertSpv);
    cfg.fragSpv            = std::move(fragSpv);
    cfg.bindingDescs       = { centerBinding, axesBinding };
    cfg.attrDescs          = attrs;
    cfg.descriptorBindings = { uboBinding };
    cfg.topology           = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
    cfg.framesInFlight     = framesInFlight;
    cfg.uboSize            = sizeof(SSFREllipsoidUBO);
    return cfg;
}

inline SSFREllipsoidUBO makeEllipsoidUBO(const glm::mat4& proj, const glm::mat4& modelView,
                                         const SSFREllipsoidDraw& draw, float thicknessScale)
{
    SSFREllipsoidUBO ubo{};
    ubo.proj      = proj;
    ubo.invProj   = glm::inverse(proj);
    ubo.modelView = modelView;
    ubo.params    = glm::vec4(draw.radiusScale,
                              static_cast<float>(draw.extent.width),
                              static_cast<float>(draw.extent.height),
                              draw.maxPointSize);
    ubo.params2   = glm::vec4(thicknessScale, 0.0f, 0.0f, 0.0f);
    return ubo;
}

inline void drawEllipsoids(VkCommandBuffer cmd, uint32_t frameIndex, SSFRPipeline& pipeline,
                           const SSFREllipsoidDraw& draw)
{
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.getPipeline());
    VkDescriptorSet ds = pipeline.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipeline.getLayout(), 0, 1, &ds, 0, nullptr);
    const VkBuffer buffers[2] = { draw.centers, draw.axes };
    const VkDeviceSize offsets[2] = { 0, 0 };
    vkCmdBindVertexBuffers(cmd, 0, 2, buffers, offsets);
    vkCmdDraw(cmd, draw.count, 1, 0, 0);
}

} // namespace Phantom
