#pragma once

#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"
#include "../../CGLib/GltfRenderer/Renderer/GltfSceneRenderer.h"

#include <glm/glm.hpp>

#include <array>
#include <string>

namespace Phantom {

class SSFluidRenderer;

/**
 * @brief Shared glTF background/set + environment + directional light for
 * PhysicsView's Vulkan-native rendering
 * (docs/todo/PLAN_physicsview_gltf_rendering.md, Phase 1).
 *
 * FluidApp owns one instance and wires it to the single hosted
 * GltfSceneRenderer (bgGltfRenderer_) plus the SSFR renderer. CommandDispatcher
 * drives it from the LoadRenderBackground / SetRenderBackgroundTransform /
 * SetEnvironment / SetLight / SetRenderUseIBL / GetRenderSceneState commands and
 * RenderingPanel exposes the same knobs in the Control window.
 *
 * The document is loaded through GltfSceneRenderer::loadDocument() (hot-reload,
 * no pipeline rebuild); this class owns the GltfDocument it points at so it
 * outlives every in-flight frame.
 *
 * Deviations from FluidStudio's equivalent (VkFluidRenderer, Phase 1):
 *  - SetEnvironment forwards the cube-map faces to SSFluidRenderer::loadEnvMap()
 *    only (SSFR reflections + the SSFR-mode skybox). GltfSceneRenderer has no
 *    skybox pass of its own and no IBL shaders are supplied here, so a
 *    standalone skybox for the point-fluid path and real IBL are deferred to
 *    Phase 6 -- setUseIBL() still stores the flag for GetRenderSceneState.
 *  - Headless (bgGltfRenderer_ never onInit'd) is not a real PhysicsView mode,
 *    but the null guards mirror FluidStudio's no-op-success contract anyway.
 */
class RenderBackground {
public:
    // Call before setInitialDocument()/any command. ssfr may be null.
    void bind(Phantom::Gltf::GltfSceneRenderer* gltf, SSFluidRenderer* ssfr) {
        gltf_ = gltf;
        ssfr_ = ssfr;
    }

    // The shipped cube-map directory (next to the executable). ClearRenderEnvironment
    // reloads it so the viewer returns to its default studio look.
    void setDefaultEnvDir(std::string dir) { defaultEnvDir_ = std::move(dir); }

    // Install the startup background (Phase 0's synthesized stage, or a generated
    // GLB). Requires gltf_ to have been onInit'd already.
    void setInitialDocument(Phantom::Gltf::GltfDocument doc);

    // Push the current light to the glTF pass + SSFR (call once after init).
    void applyLight();

    // ---- command surface (all no-op-success when gltf_ == nullptr) ----
    bool loadBackground(const std::string& path);   // .glb/.gltf/.obj/.stl
    void clearBackground();
    void setTransform(const glm::vec3& posMeters, const glm::vec3& rotDegXYZ, float scale);
    bool setEnvironment(const std::string& faceDir); // faceDir/{right,left,top,bottom,front,back}.png
    void clearEnvironment();
    void setLight(const glm::vec3& direction, const glm::vec3& color, float intensity);
    void setUseIBL(bool v);
    // Phase 4: whether the glTF passes cast the shared light's shadow map.
    // FluidApp owns the ShadowMapPass and reads this; RenderBackground just
    // holds the flag (like useIBL) so the command + panel have one home.
    void setCastShadows(bool v) { castShadows_ = v; }
    bool castShadows() const    { return castShadows_; }

    std::string sceneStateJson() const;

    // ---- panel read-back ----
    const std::string& backgroundPath() const { return bgPath_; }
    const std::string& environmentDir() const { return envDir_; }
    bool  hasEnvironment() const { return hasEnv_; }
    bool  useIBL() const { return useIBL_; }
    int   primitiveCount() const { return gltf_ ? gltf_->primitiveCount() : 0; }
    glm::vec3 lightDirection() const { return lightDir_; }
    glm::vec3 lightColor() const { return lightColor_; }
    float lightIntensity() const { return lightIntensity_; }
    glm::vec3 transformPosition() const { return xfPos_; }
    glm::vec3 transformRotationDeg() const { return xfRotDeg_; }
    float transformScale() const { return xfScale_; }

private:
    static std::array<std::string, 6> cubeFacePaths(const std::string& dir);
    void applyTransform();

    Phantom::Gltf::GltfSceneRenderer* gltf_ = nullptr;
    SSFluidRenderer*                  ssfr_ = nullptr;

    Phantom::Gltf::GltfDocument doc_;
    Phantom::Gltf::GltfDocument emptyDoc_;

    std::string bgPath_;
    std::string envDir_;
    std::string defaultEnvDir_;
    bool hasEnv_       = false;
    bool useIBL_       = false;
    bool castShadows_  = true;

    glm::vec3 xfPos_{0.f};
    glm::vec3 xfRotDeg_{0.f};
    float     xfScale_ = 1.f;

    // Default: a slightly raking key light, matching the Phase 0 wiring.
    glm::vec3 lightDir_{-0.3f, -1.0f, -0.25f};
    glm::vec3 lightColor_{1.f};
    float     lightIntensity_ = 3.f;
};

} // namespace Phantom
