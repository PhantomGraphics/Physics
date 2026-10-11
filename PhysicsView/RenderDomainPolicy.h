#pragma once

#include "BodyRenderMode.h"

namespace Phantom {

/**
 * @brief Which scene renderers draw this frame -- the visibility rules FluidApp::onUpdate() used
 * to spell out inline between its simulation and camera work.
 *
 * Pure decision logic (no Vulkan / renderer types), so the combinations -- SSFR on/off, the
 * Cloud page owning the viewport, flame present or not, wire/shaded/both bodies -- are tabulated
 * in PhysicsTest instead of only being visible in screenshots. FluidApp builds a
 * RenderDomainInputs from its state and applies the returned RenderDomainPlan with plain
 * setEnabled()/setVisible() calls.
 *
 * Rules:
 *  - Only the Cloud page owns the viewport. While it is active the fluid SSFR, flame, hair,
 *    rigid, soft and glTF background renderers are off; the shared FluidRenderer shows the air
 *    particles as points unless the volume view is on, in which case the volume renderer draws.
 *  - Otherwise SSFR and the point renderer are alternatives (exactly one draws the fluid).
 *    SSFR "disabled" still leaves its composite as the final pass (passthrough); `ssfr` here
 *    gates only the fluid-surface work.
 *  - The flame draws whenever it is in the scene; its fixed-carrier debug view needs carriers.
 *  - Rigid and soft bodies each draw wire, shaded, or both, per their render mode.
 */
struct RenderDomainInputs {
    bool useSSFR = false;               ///< SSFR panel "enabled".
    bool cloudPageActive = false;
    bool cloudVolumeMode = false;       ///< cloud shown by ray marching instead of points.
    bool flameInScene = false;
    bool flameFixedCarriers = false;
    BodyRenderMode rigidMode = BodyRenderMode::Wireframe;
    BodyRenderMode softMode = BodyRenderMode::Wireframe;
};

struct RenderDomainPlan {
    bool ssfr = false;                  ///< SSFR fluid-surface passes.
    bool fluidPoints = false;           ///< FluidRenderer point sprites.
    bool flame = false;
    bool flameDebug = false;
    bool cloudVolume = false;
    bool hair = false;
    bool rigidWire = false;
    bool rigidShaded = false;
    bool softWire = false;
    bool softShaded = false;
    bool background = false;            ///< glTF background stage.
};

inline RenderDomainPlan planRenderDomains(const RenderDomainInputs& in)
{
    RenderDomainPlan p;
    const bool ownsViewport = in.cloudPageActive;

    p.ssfr        = !in.cloudPageActive && in.useSSFR;
    p.fluidPoints = in.cloudPageActive ? !in.cloudVolumeMode : !in.useSSFR;
    p.flame       = in.flameInScene && !in.cloudPageActive;
    p.flameDebug  = p.flame && in.flameFixedCarriers;
    p.cloudVolume = in.cloudPageActive && in.cloudVolumeMode;

    p.hair        = !ownsViewport;
    p.rigidWire   = !ownsViewport && bodyRenderModeWantsWire(in.rigidMode);
    p.rigidShaded = !ownsViewport && bodyRenderModeWantsShaded(in.rigidMode);
    p.softWire    = !ownsViewport && bodyRenderModeWantsWire(in.softMode);
    p.softShaded  = !ownsViewport && bodyRenderModeWantsShaded(in.softMode);
    p.background  = !ownsViewport;
    return p;
}

} // namespace Phantom
