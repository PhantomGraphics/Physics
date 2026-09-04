#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"

#include "ControlPage.h"
#include "IEmbeddedPanel.h"

#include <array>
#include <functional>
#include <string>

namespace Phantom {

/**
 * @brief The single shared "Control" window (GUI_RESTRUCTURING_PLAN.md 5.2).
 *
 * Holds one active ControlPage, a page<->panel table, an optional common
 * status area drawn above every page, and renders the selected page's
 * embedded panel. It keeps only non-owning pointers -- the panels are still
 * owned by FluidApp and their lifetime is unchanged.
 */
class ControlPanelHost : public ::VKG::IVkUIPanel {
public:
    void registerPage(ControlPage page, IEmbeddedPanel* panel);
    void setPage(ControlPage page);
    ControlPage getPage() const { return activePage_; }
    bool isPageRegistered(ControlPage page) const;

    // Marks a page selectable/greyed in the Physics menu. reason is shown as a
    // tooltip and as an in-page note when the page is unavailable.
    void setPageEnabled(ControlPage page, bool enabled, const std::string& reason = {});
    bool isPageEnabled(ControlPage page) const;
    const std::string& pageDisabledReason(ControlPage page) const;

    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    // Drawn at the top of the window, before the active page, on every frame
    // (GUI_RESTRUCTURING_PLAN.md section 7). Optional.
    void setStatusDrawer(std::function<void()> fn) { statusDrawer_ = std::move(fn); }

    // Enables persistence of the last active page + window-visible flag to a
    // small ini file (GUI_RESTRUCTURING_PLAN.md Phase 6). The Control window's
    // position/size and section open/closed state are already persisted by
    // Dear ImGui itself via imgui.ini.
    void setLayoutFile(std::string path) { layoutPath_ = std::move(path); }

    void onImGui() override;

private:
    struct PageSlot {
        IEmbeddedPanel* panel = nullptr;
        bool            enabled = true;
        std::string     disabledReason;
    };

    void loadLayout();
    void saveLayout() const;

    std::array<PageSlot, kControlPageCount> pages_{};
    ControlPage activePage_ = ControlPage::Fluid;
    bool visible_ = true;
    std::function<void()> statusDrawer_;

    std::string layoutPath_;
    bool        layoutLoaded_ = false;
    ControlPage lastSavedPage_ = ControlPage::Fluid;
    bool        lastSavedVisible_ = true;
};

} // namespace Phantom
