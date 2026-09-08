#include "pch.h"
#include "GltfBodyRenderer.h"

#include "PrimitiveGltf.h"
#include "RigidBodyWorld.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <unordered_set>

namespace Phantom {

namespace {
// Fixed per-body colour by static-ness (contact-flash, which the wireframe
// renderer does, is left out -- it would mean rebuilding the material every
// frame; see the class doc).
const glm::vec4 kStaticColor {0.45f, 0.47f, 0.50f, 1.0f};
const glm::vec4 kDynamicColor{0.35f, 0.55f, 0.90f, 1.0f};
} // namespace

bool GltfBodyRenderer::parseMode(const std::string& s, Mode& out) {
    if (s == "wire" || s == "wireframe") { out = Mode::Wireframe; return true; }
    if (s == "shaded")                   { out = Mode::Shaded;    return true; }
    if (s == "both")                     { out = Mode::Both;      return true; }
    return false;
}

const char* GltfBodyRenderer::modeName(Mode m) {
    switch (m) {
    case Mode::Wireframe: return "wire";
    case Mode::Shaded:    return "shaded";
    case Mode::Both:      return "both";
    }
    return "wire";
}

Phantom::Gltf::GltfSceneRenderer::Shaders GltfBodyRenderer::makeShaders() const {
    Phantom::Gltf::GltfSceneRenderer::Shaders s;
    s.vertSpv = vertSpv_;
    s.fragSpv = fragSpv_;
    return s;
}

glm::mat4 GltfBodyRenderer::bodyModelMatrix(const Physics::RigidBody& body) {
    const glm::vec3 pos(body.position.x, body.position.y, body.position.z);
    glm::mat4 m = glm::translate(glm::mat4(1.f), pos);

    if (!body.shape) return m;

    switch (body.shape->getType()) {
    case Physics::ShapeType::Sphere: {
        const auto* ss = static_cast<const Physics::SphereShape*>(body.shape);
        m *= glm::mat4_cast(body.orientation);
        m = glm::scale(m, glm::vec3(ss->radius));
        break;
    }
    case Physics::ShapeType::Box: {
        const auto* bs = static_cast<const Physics::BoxShape*>(body.shape);
        m *= glm::mat4_cast(body.orientation);
        m = glm::scale(m, glm::vec3(bs->halfExtents.x, bs->halfExtents.y, bs->halfExtents.z));
        break;
    }
    case Physics::ShapeType::Plane:
    case Physics::ShapeType::Mesh:
        // Plane doc is already a large flat quad in local y=0; just place it.
        break;
    }
    return m;
}

GltfBodyRenderer::Instance GltfBodyRenderer::makeInstance(const Physics::RigidBody& body) const {
    const glm::vec4 color = body.isStatic() ? kStaticColor : kDynamicColor;

    Instance inst;
    if (body.shape && body.shape->getType() == Physics::ShapeType::Sphere) {
        inst.doc = makeUnitSphereGltf(color);
    } else if (body.shape && body.shape->getType() == Physics::ShapeType::Box) {
        inst.doc = makeUnitBoxGltf(color);
    } else {
        inst.doc = makePlaneGltf(color);
    }

    inst.renderer = std::make_unique<Phantom::Gltf::GltfSceneRenderer>();
    inst.renderer->setDocument(inst.doc);
    inst.renderer->setShaders(makeShaders());
    inst.renderer->setModelMatrix(bodyModelMatrix(body));
    inst.renderer->setCamera(view_, proj_, eye_);
    inst.renderer->setLight(lightDirW0_, lightColorIntensity_);
    inst.renderer->onInit(*ctx_, *pool_, renderPass_, framesInFlight_);
    return inst;
}

void GltfBodyRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                              const Phantom::VKG::VulkanCommandPool& pool,
                              VkRenderPass renderPass, uint32_t framesInFlight) {
    ctx_            = &ctx;
    pool_           = &pool;
    renderPass_     = renderPass;
    framesInFlight_ = framesInFlight;
    ready_          = true;
    syncFromWorld();
}

void GltfBodyRenderer::syncFromWorld() {
    if (!ready_ || !world_) return;

    const auto& bodies = world_->getWorld().getBodies();
    std::unordered_set<const Physics::RigidBody*> live(bodies.begin(), bodies.end());

    // Drop instances whose body is gone (preset switch / RemoveBody). A command
    // like SetPreset runs mid-frame (dispatcher_.processQueue() in onUpdate,
    // before onRender), so a previous frame's command buffer may still reference
    // this instance's pipeline -- wait for the device to go idle first, mirroring
    // Universe's GltfRenderer::removeEntity().
    bool removedAny = false;
    for (auto it = instances_.begin(); it != instances_.end();) {
        if (live.count(it->first)) { ++it; continue; }
        if (!removedAny && ctx_) { vkDeviceWaitIdle(ctx_->getDevice()); removedAny = true; }
        if (ctx_) it->second.renderer->onCleanup(ctx_->getDevice());
        it = instances_.erase(it);
    }

    // Add instances for new bodies. Plane-shape bodies (the preset floor) are
    // skipped: a shaded infinite floor is redundant with the glTF background
    // stage and z-fights with it when both sit at y=0. The wireframe renderer
    // still draws the floor grid as a spatial reference.
    for (const Physics::RigidBody* body : bodies) {
        if (!body || instances_.count(body)) continue;
        if (body->shape && body->shape->getType() == Physics::ShapeType::Plane) continue;
        instances_.emplace(body, makeInstance(*body));
    }
}

void GltfBodyRenderer::setCamera(const glm::mat4& view, const glm::mat4& proj, const glm::vec3& eye) {
    view_ = view;
    proj_ = proj;
    eye_  = eye;
}

void GltfBodyRenderer::setLight(const glm::vec4& dirW0, const glm::vec4& colorIntensityW) {
    lightDirW0_          = dirW0;
    lightColorIntensity_ = colorIntensityW;
}

void GltfBodyRenderer::onUpdate(uint32_t frameIndex) {
    if (!ready_) return;
    for (auto& [body, inst] : instances_) {
        inst.renderer->setModelMatrix(bodyModelMatrix(*body));
        inst.renderer->setCamera(view_, proj_, eye_);
        inst.renderer->setLight(lightDirW0_, lightColorIntensity_);
        inst.renderer->onUpdate(frameIndex);
    }
}

void GltfBodyRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex) {
    if (!ready_ || !enabled_ || mode_ == Mode::Wireframe) return;
    for (auto& [body, inst] : instances_) {
        (void)body;
        inst.renderer->onRender(cmd, frameIndex);
    }
}

void GltfBodyRenderer::onCleanup(VkDevice device) {
    if (device != VK_NULL_HANDLE) vkDeviceWaitIdle(device);
    for (auto& [body, inst] : instances_) {
        (void)body;
        inst.renderer->onCleanup(device);
    }
    instances_.clear();
    ready_ = false;
}

} // namespace Phantom
