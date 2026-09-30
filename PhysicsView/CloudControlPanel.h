#pragma once

#include "IEmbeddedPanel.h"
#include "CloudWorld.h"

#include <functional>

namespace Phantom {

/**
 * @brief Cloud (moist-air SPH) controls, embedded into the shared Control window
 * as ControlPage::Cloud (docs/todo/PLAN_cloud_sph_pbvr.md Phase 2).
 *
 * Every action here is one CloudWorld call that CloudCommandDispatcher also
 * exposes, so the panel and the scenarios drive the same API. Does not open its
 * own window (IEmbeddedPanel contract).
 */
class CloudControlPanel : public IEmbeddedPanel {
public:
    explicit CloudControlPanel(CloudWorld* world) : world_(world) {}

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }

    void drawContents() override;

private:
    CloudWorld*           world_ = nullptr;
    std::function<void()> onWorldChanged_;
    int                   selectedSource_ = 0;
    int                   pendingSeed_ = 1;

    void notifyWorldChanged() { if (onWorldChanged_) onWorldChanged_(); }
};

} // namespace Phantom
