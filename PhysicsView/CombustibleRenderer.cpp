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
        auto& inst=*ptr; std::vector<int> keys;
        for(const auto& s:b->samples()) keys.push_back(combustibleColorLevel(s,world_->solidDebugColor)+16*world_->solidDebugColor);
        if(inst.colorKeys!=keys) {
            changed=true;
            inst.doc=makeCombustibleGltf(*b,world_->solidDebugColor);
            if(fresh) { inst.renderer.setDocument(inst.doc); inst.renderer.setShaders(shaders_);
                inst.renderer.onInit(*ctx_,*pool_,pass_,frames_); }
            else { vkDeviceWaitIdle(ctx_->getDevice()); inst.renderer.loadDocument(inst.doc); }
            inst.colorKeys=std::move(keys);
        }
        const auto& physical=world_->physicalTransform();
        const glm::vec3 center=physical.toScene(b->center());
        changed=changed || center!=inst.center || b->orientation()!=inst.orientation; inst.center=center; inst.orientation=b->orientation();
        inst.renderer.setModelMatrix(glm::scale(glm::translate(glm::mat4(1),center)*glm::mat4_cast(b->orientation()),glm::vec3(physical.scale)));
        inst.renderer.setCamera(view_,proj_,eye_);
        inst.renderer.setLight(glm::vec4(glm::normalize(glm::vec3(-0.3f,-1,-0.25f)),0),glm::vec4(1,1,1,3));
        inst.renderer.onUpdate(frame);
    }
    if(changed && onChanged_) onChanged_();
}
void CombustibleRenderer::onRender(VkCommandBuffer cmd,uint32_t frame)
{ if(world_ && world_->isPopulated()) for(auto& [id,inst]:instances_) inst->renderer.onRender(cmd,frame); }
void CombustibleRenderer::onCleanup(VkDevice device)
{ for(auto& [id,inst]:instances_) inst->renderer.onCleanup(device); instances_.clear(); ctx_=nullptr; }
