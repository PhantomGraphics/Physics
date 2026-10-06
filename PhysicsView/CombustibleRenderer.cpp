#include "pch.h"
#include "CombustibleRenderer.h"
#include "CombustibleGltf.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include <glm/gtc/matrix_transform.hpp>
#include <set>

using namespace Phantom;

void CombustibleRenderer::onInit(VKG::VulkanContext& ctx,const VKG::VulkanCommandPool& pool,VkRenderPass pass,uint32_t frames)
{ ctx_=&ctx; pool_=&pool; pass_=pass; frames_=frames; }

void CombustibleRenderer::onUpdate(uint32_t frame)
{
    if(!ctx_ || !world_) return;
    bool changed=false;
    std::set<uint64_t> live;
    for(const auto& b:world_->bodies()) live.insert(b->id());
    for(auto it=instances_.begin();it!=instances_.end();) {
        if(live.count(it->first)) { ++it; continue; }
        vkDeviceWaitIdle(ctx_->getDevice()); it->second->renderer.onCleanup(ctx_->getDevice()); it=instances_.erase(it);
        changed=true;
    }
    for(const auto& b:world_->bodies()) {
        auto& ptr=instances_[b->id()]; const bool fresh=!ptr;
        if(fresh) ptr=std::make_unique<Instance>();
        auto& inst=*ptr;
        if(fresh) {
            changed=true;
            inst.doc=makeCombustibleScalarGltf(*b);
            inst.renderer.setDocument(inst.doc); inst.renderer.setShaders(shaders_);
            inst.renderer.onInit(*ctx_,*pool_,pass_,frames_);
        }
        std::vector<glm::vec4> values;
        values.reserve(b->samples().size()+1);
        values.emplace_back(static_cast<float>(world_->solidDebugColor),0,0,0);
        for(const auto& s:b->samples()) values.emplace_back(s.temperature,static_cast<float>(s.fuel),static_cast<float>(s.initialFuel),0);
        changed=changed || values!=inst.scalarValues;
        inst.scalarValues=values;
        inst.renderer.setScalarField(std::move(values));
        const auto& physical=world_->physicalTransform();
        const glm::vec3 center=physical.toScene(b->center());
        changed=changed || center!=inst.center || b->orientation()!=inst.orientation; inst.center=center; inst.orientation=b->orientation();
        inst.renderer.setModelMatrix(glm::scale(glm::translate(glm::mat4(1),center)*glm::mat4_cast(b->orientation()),glm::vec3(physical.scale)));
        inst.renderer.setCamera(view_,proj_,eye_);
        inst.renderer.setLight(lightDirection_,lightColor_);
        inst.renderer.onUpdate(frame);
    }
    if(changed && onChanged_) onChanged_();
}
void CombustibleRenderer::onRender(VkCommandBuffer cmd,uint32_t frame)
{ if(world_ && world_->isPopulated()) for(auto& [id,inst]:instances_) inst->renderer.onRender(cmd,frame); }
void CombustibleRenderer::onCleanup(VkDevice device)
{ for(auto& [id,inst]:instances_) inst->renderer.onCleanup(device); instances_.clear(); ctx_=nullptr; }

void CombustibleRenderer::renderSampledLight(VkCommandBuffer cmd,uint32_t frame,VkDescriptorSet shadows,const Gltf::GltfSampledLight& light)
{
    if(world_ && world_->isPopulated())
        for(auto& [id,inst]:instances_) inst->renderer.renderSampledLight(cmd,frame,shadows,light);
}
