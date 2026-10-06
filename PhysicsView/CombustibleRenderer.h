#pragma once
#include "FlameWorld.h"
#include "../../CGLib/GltfRenderer/Renderer/GltfSceneRenderer.h"
#include <map>

namespace Phantom {
// Opaque depth-writing primitives shared by the Normal and PBVR scene paths.
class CombustibleRenderer : public ::VKG::IVkSubRenderer {
public:
    void bind(FlameWorld* world) { world_=world; }
    void setOnChanged(std::function<void()> callback) { onChanged_=std::move(callback); }
    void setShaders(Gltf::GltfSceneRenderer::Shaders shaders) { shaders_=std::move(shaders); }
    void setCamera(const glm::mat4& view,const glm::mat4& proj,const glm::vec3& eye) { view_=view; proj_=proj; eye_=eye; }
    void setLight(const glm::vec4& direction,const glm::vec4& color) { lightDirection_=direction; lightColor_=color; }
    void onInit(VKG::VulkanContext& ctx,const VKG::VulkanCommandPool& pool,VkRenderPass pass,uint32_t frames) override;
    void onUpdate(uint32_t frame) override;
    void onRender(VkCommandBuffer cmd,uint32_t frame) override;
    void renderSampledLight(VkCommandBuffer cmd,uint32_t frame,VkDescriptorSet shadows,const Gltf::GltfSampledLight& light);
    void onCleanup(VkDevice device) override;
private:
    struct Instance { Gltf::GltfDocument doc; Gltf::GltfSceneRenderer renderer; std::vector<glm::vec4> scalarValues; glm::vec3 center{0}; Math::Quaternion orientation{1,0,0,0}; };
    FlameWorld* world_=nullptr;
    std::function<void()> onChanged_;
    VKG::VulkanContext* ctx_=nullptr;
    const VKG::VulkanCommandPool* pool_=nullptr;
    VkRenderPass pass_=VK_NULL_HANDLE;
    uint32_t frames_=0;
    Gltf::GltfSceneRenderer::Shaders shaders_;
    glm::mat4 view_{1},proj_{1}; glm::vec3 eye_{0};
    glm::vec4 lightDirection_{-0.3f,-1.0f,-0.25f,0};
    glm::vec4 lightColor_{1,1,1,3};
    std::map<uint64_t,std::unique_ptr<Instance>> instances_;
};
}
