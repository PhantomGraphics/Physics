#pragma once

#include <functional>
#include <utility>

namespace Phantom {

/**
 * @brief A panel whose *content* rendering is separated from its window
 * creation, so the same controls can be shown either as a standalone ImGui
 * window (via the panel's own onImGui()) or embedded into the shared Control
 * window managed by ControlPanelHost.
 *
 * Contract: drawContents() MUST NOT call ImGui::Begin()/End() (or the
 * Immediate beginWindow()/endWindow() wrappers) -- it is always invoked with a
 * window already open. onImGui() (when a panel keeps one for backward
 * compatibility) is the only place a window may be started, and it should do
 * nothing more than begin the window, call drawContents(), and end it.
 *
 * PhysicsView-local for now; promote to CGLib only once a second app needs the
 * same mechanism (see GUI_RESTRUCTURING_PLAN.md section 9).
 */
class IEmbeddedPanel {
public:
    virtual ~IEmbeddedPanel() = default;
    virtual void drawContents() = 0;
};

/**
 * @brief Adapts anything with content-only rendering (e.g. a library panel
 * that exposes a drawEmbedded() but cannot derive from IEmbeddedPanel) into an
 * IEmbeddedPanel via a callable.
 */
class FnEmbeddedPanel : public IEmbeddedPanel {
public:
    explicit FnEmbeddedPanel(std::function<void()> fn) : fn_(std::move(fn)) {}
    void drawContents() override { if (fn_) fn_(); }

private:
    std::function<void()> fn_;
};

} // namespace Phantom
