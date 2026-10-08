#pragma once
#include "IEmbeddedPanel.h"
#include "HairWorld.h"
#include <functional>

namespace Phantom {
class HairControlPanel : public IEmbeddedPanel {
public:
    explicit HairControlPanel(HairWorld* world) : world_(world) {}
    void setOnWorldChanged(std::function<void()> callback) { changed_ = std::move(callback); }
    void setOnFrameGuides(std::function<void()> callback) { frame_ = std::move(callback); }
    void drawContents() override;
private:
    HairWorld* world_ = nullptr;
    std::function<void()> changed_;
    std::function<void()> frame_;
};
} // namespace Phantom
