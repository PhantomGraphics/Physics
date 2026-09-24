#pragma once

#include "IEmbeddedPanel.h"
#include "FlameWorld.h"
#include "FlamePBVRPass.h"

#include <functional>

namespace Phantom {

/**
 * @brief Flame (reacting hot-gas SPH) scene controls, embedded into the shared
 * Control window as ControlPage::Flame.
 *
 * A direct port of the former FlameView::FlameApp::onImGui() -- Run/Step/Reset,
 * emitter, combustion, buoyancy/vorticity/noise and secondary-particle
 * sections -- plus the display transform (render scale / offset) that maps the
 * native-scale plume onto FluidApp's shared camera. drawContents() uses the
 * UI::Immediate facade directly rather than a declarative widget tree: it is a
 * faithful, low-churn transcription of the standalone viewer's panel and does
 * not open its own window (IEmbeddedPanel contract).
 */
class FlameControlPanel : public IEmbeddedPanel {
public:
    explicit FlameControlPanel(FlameWorld* world) : world_(world) {}

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }
    void setPBVRStatsSource(std::function<FlamePBVRPass::Stats()> fn) { pbvrStats_ = std::move(fn); }

    void drawContents() override;

private:
    FlameWorld*           world_ = nullptr;
    std::function<void()> onWorldChanged_;
    std::function<FlamePBVRPass::Stats()> pbvrStats_;

    void notifyWorldChanged() { if (onWorldChanged_) onWorldChanged_(); }
};

} // namespace Phantom
