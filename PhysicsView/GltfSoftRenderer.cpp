#include "pch.h"
#include "GltfSoftRenderer.h"

#include "SoftMeshGltf.h"
#include "SoftBodyWorld.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"

#include <unordered_set>

namespace Phantom {

namespace {
const glm::vec4 kSoftColor{0.82f, 0.46f, 0.52f, 1.0f}; // warm cloth/jelly tone
} // namespace

Phantom::Gltf::GltfSceneRenderer::Shaders GltfSoftRenderer::makeShaders() const {
    Phantom::Gltf::GltfSceneRenderer::Shaders s;
    s.vertSpv = vertSpv_;
    s.fragSpv = fragSpv_;
    s.shadowVertSpv = shadowVertSpv_;
    s.shadowFragSpv = shadowFragSpv_;
    return s;
}

GltfSoftRenderer::Instance GltfSoftRenderer::makeInstance(const Physics::ISoftBody& body) const {
    const Physics::SoftMesh& mesh = body.getMesh();

    Instance inst;
    inst.doc = makeSoftBodyGltf(mesh.particles.positions, mesh.faces, kSoftColor);
    inst.vertexCount = mesh.particles.positions.size();

    inst.renderer = std::make_unique<Phantom::Gltf::GltfSceneRenderer>();
    inst.renderer->setDynamic(true);                     // keep the CPU vertex mirror for updateMorphedGeometry()
    inst.renderer->setCullMode(VK_CULL_MODE_NONE);       // zero-thickness surface -- see SoftMeshGltf.h
    inst.renderer->setDocument(inst.doc);
    inst.renderer->setShaders(makeShaders());
    inst.renderer->setCamera(view_, proj_, eye_);
    inst.renderer->onInit(*ctx_, *pool_, renderPass_, framesInFlight_);
    // setLight() moving from before onInit() to inside applyLightShadowState() (after onInit())
    // is behavior-preserving -- GltfSceneRenderer::setLight() only stores member fields that
    // onUpdate() reads later, onInit() never touches them.
    Phantom::Gltf::applyLightShadowState(*inst.renderer, state_);
    return inst;
}

void GltfSoftRenderer::enableShadows(VkRenderPass shadowRenderPass, VkImageView shadowView,
                                    VkSampler shadowSampler) {
    state_.shadowCasterRenderPass = shadowRenderPass;
    state_.shadowEnabled          = true;
    state_.shadowView             = shadowView;
    state_.shadowSampler          = shadowSampler;
    for (auto& [body, inst] : instances_) {
        (void)body;
        Phantom::Gltf::applyLightShadowState(*inst.renderer, state_);
    }
}

void GltfSoftRenderer::setShadowLightVP(const glm::mat4& lightVP) {
    state_.shadowVP = lightVP;
    for (auto& [body, inst] : instances_) { (void)body; inst.renderer->setShadowLightVP(lightVP); }
}

void GltfSoftRenderer::renderShadowCasters(VkCommandBuffer cmd, const glm::mat4& lightVP) {
    if (!ready_ || !state_.shadowEnabled) return;
    for (auto& [body, inst] : instances_) { (void)body; inst.renderer->renderShadowCasters(cmd, lightVP); }
}

void GltfSoftRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                              const Phantom::VKG::VulkanCommandPool& pool,
                              VkRenderPass renderPass, uint32_t framesInFlight) {
    ctx_            = &ctx;
    pool_           = &pool;
    renderPass_     = renderPass;
    framesInFlight_ = framesInFlight;
    ready_          = true;
    syncFromWorld();
}

void GltfSoftRenderer::syncFromWorld() {
    if (!ready_ || !world_) return;

    const auto& bodies = world_->getBodyPointers();
    std::unordered_set<const Physics::ISoftBody*> live(bodies.begin(), bodies.end());

    bool removedAny = false;
    for (auto it = instances_.begin(); it != instances_.end();) {
        if (live.count(it->first)) { ++it; continue; }
        if (!removedAny && ctx_) { vkDeviceWaitIdle(ctx_->getDevice()); removedAny = true; }
        if (ctx_) it->second.renderer->onCleanup(ctx_->getDevice());
        it = instances_.erase(it);
    }

    for (const Physics::ISoftBody* body : bodies) {
        if (!body || instances_.count(body)) continue;
        // Rope (SoftMesh with edges but no faces) has no surface -- stays
        // wireframe-only.
        if (body->getMesh().faces.empty()) continue;
        instances_.emplace(body, makeInstance(*body));
    }
}

void GltfSoftRenderer::setCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& eye) {
    view_ = view;
    proj_ = proj;
    eye_  = eye;
}

void GltfSoftRenderer::setLight(const glm::vec4& dirW0, const glm::vec4& colorIntensityW) {
    state_.lightPos   = dirW0;
    state_.lightColor = colorIntensityW;
}

void GltfSoftRenderer::onUpdate(uint32_t frameIndex) {
    if (!ready_) return;
    for (auto& [body, inst] : instances_) {
        const Physics::SoftMesh& mesh = body->getMesh();
        const auto& positions = mesh.particles.positions;
        if (positions.size() != inst.vertexCount) {
            // Particle count changed under us (shouldn't for the built presets);
            // skip this frame -- syncFromWorld() rebuilds on preset switch.
            inst.renderer->onUpdate(frameIndex);
            continue;
        }
        computeSoftBodyNormals(positions, mesh.faces, normalScratch_);
        inst.renderer->updateMorphedGeometry(0, 0, positions, normalScratch_);
        inst.renderer->setCamera(view_, proj_, eye_);
        inst.renderer->setLight(state_.lightPos, state_.lightColor);
        inst.renderer->onUpdate(frameIndex);
    }
}

void GltfSoftRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex) {
    if (!ready_ || !enabled_ || mode_ == Mode::Wireframe) return;
    for (auto& [body, inst] : instances_) {
        (void)body;
        inst.renderer->onRender(cmd, frameIndex);
    }
}

void GltfSoftRenderer::onCleanup(VkDevice device) {
    if (device != VK_NULL_HANDLE) vkDeviceWaitIdle(device);
    for (auto& [body, inst] : instances_) {
        (void)body;
        inst.renderer->onCleanup(device);
    }
    instances_.clear();
    ready_ = false;
}

} // namespace Phantom
