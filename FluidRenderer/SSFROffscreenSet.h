#pragma once

#include "../../CGLib/VulkanGraphics/VulkanOffscreen.h"
#include "../../CGLib/VulkanGraphics/VulkanSampler.h"

#include <vulkan/vulkan.h>

namespace Phantom::VKG { class VulkanContext; }

namespace Phantom {

// Manages all offscreen render targets used by the SSFR pipeline.
// Each pass runs sequentially so framesInFlight is always 1.
// Recreate with destroy() then create() on swapchain resize.
class SSFROffscreenSet {
public:
    SSFROffscreenSet() = default;
    SSFROffscreenSet(const SSFROffscreenSet&) = delete;
    SSFROffscreenSet& operator=(const SSFROffscreenSet&) = delete;

    // Create all offscreen targets and the shared sampler.
    void create(const Phantom::VKG::VulkanContext& ctx, uint32_t width, uint32_t height);

    // Release all resources.
    void destroy(const Phantom::VKG::VulkanContext& ctx);

    bool isValid() const { return depth_.isValid(); }
    VkExtent2D getExtent() const { return extent_; }

    // Shared sampler for all targets (CLAMP_TO_EDGE).
    VkSampler getSampler() const { return sampler_.get(); }

    // Pass 1: depth (R32_SFLOAT) — fragment shader writes corrected depth to gl_FragCoord.z
    Phantom::VKG::VulkanOffscreen& depth()         { return depth_; }
    // Pass 2: thickness (R32_SFLOAT, additive blend, configured at pipeline side)
    Phantom::VKG::VulkanOffscreen& thickness()     { return thickness_; }
    // Pass 3a: bilateral-filtered smoothed (R32_SFLOAT)
    Phantom::VKG::VulkanOffscreen& smoothed()      { return smoothed_; }
    // Pass 3b: bilateral-filtered smoothed depth (R32_SFLOAT, for depth reconstruction)
    Phantom::VKG::VulkanOffscreen& smoothedDepth() { return smoothedDepth_; }
    // Pass 4a: environment reflection (RGBA8)
    Phantom::VKG::VulkanOffscreen& reflection()    { return reflection_; }
    // Pass 4b: refraction color (RGBA8)
    Phantom::VKG::VulkanOffscreen& refraction()    { return refraction_; }
    // Pass 5: spray opacity (R32_SFLOAT)
    Phantom::VKG::VulkanOffscreen& spray()         { return spray_; }
    // Pass 6: foam opacity (R32_SFLOAT)
    Phantom::VKG::VulkanOffscreen& foam()          { return foam_; }

    const Phantom::VKG::VulkanOffscreen& depth()         const { return depth_; }
    const Phantom::VKG::VulkanOffscreen& thickness()     const { return thickness_; }
    const Phantom::VKG::VulkanOffscreen& smoothed()      const { return smoothed_; }
    const Phantom::VKG::VulkanOffscreen& smoothedDepth() const { return smoothedDepth_; }
    const Phantom::VKG::VulkanOffscreen& reflection()    const { return reflection_; }
    const Phantom::VKG::VulkanOffscreen& refraction()    const { return refraction_; }
    const Phantom::VKG::VulkanOffscreen& spray()         const { return spray_; }
    const Phantom::VKG::VulkanOffscreen& foam()          const { return foam_; }

private:
    Phantom::VKG::VulkanOffscreen depth_;
    Phantom::VKG::VulkanOffscreen thickness_;
    Phantom::VKG::VulkanOffscreen smoothed_;
    Phantom::VKG::VulkanOffscreen smoothedDepth_;
    Phantom::VKG::VulkanOffscreen reflection_;
    Phantom::VKG::VulkanOffscreen refraction_;
    Phantom::VKG::VulkanOffscreen spray_;
    Phantom::VKG::VulkanOffscreen foam_;
    Phantom::VKG::VulkanSampler   sampler_;
    VkExtent2D           extent_{};
};

} // namespace VKSSFR
