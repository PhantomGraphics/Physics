#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/IWindow.h"
#include "../../CGLib/UIWidgets/Window.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Separator.h"
#include "../../CGLib/UIWidgets/IdScope.h"

#include "ControlPage.h"
#include "IEmbeddedPanel.h"

#include <array>
#include <string>

namespace Phantom {

/**
 * @brief The single shared "Control" window (GUI_RESTRUCTURING_PLAN.md 5.2).
 *
 * Holds one active ControlPage, a page<->panel table, an optional common
 * status view drawn above every page, and renders the selected page's
 * embedded panel. It keeps only non-owning pointers -- the panels are still
 * owned by FluidApp and their lifetime is unchanged.
 *
 * Declarative build (docs/todo/PLAN_physicsview_declarative_ui.md Phase 2):
 * init() loads the layout file once and assembles a Window widget tree
 * (status view + page header + page body). onImGui() just shows that root
 * and persists page/visible changes -- no per-frame UI::Immediate:: calls.
 */
class ControlPanelHost : public ::VKG::IVkUIPanel {
public:
    ControlPanelHost();

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

    // Composite view drawn at the top of the window, before the active page,
    // on every frame (GUI_RESTRUCTURING_PLAN.md section 7). Non-owning, optional.
    // Replaces the old setStatusDrawer(std::function<void()>).
    void setStatusView(UI::IWindow* view) { statusView_ = view; }

    // Enables persistence of the last active page + window-visible flag to a
    // small ini file (GUI_RESTRUCTURING_PLAN.md Phase 6). The Control window's
    // position/size and section open/closed state are already persisted by
    // Dear ImGui itself via imgui.ini.
    void setLayoutFile(std::string path) { layoutPath_ = std::move(path); }

    // Loads the layout file (once) and assembles the widget tree. Call after
    // every registerPage()/setPageEnabled()/setStatusView(). onImGui() also
    // calls it lazily as a safety net.
    void init();

    void onImGui() override;

private:
    struct PageSlot {
        IEmbeddedPanel* panel = nullptr;
        bool            enabled = true;
        std::string     disabledReason;
    };

    // Renders the active page's body: the disabled / no-panel notice, or the
    // panel's drawContents() inside a per-page ImGui ID scope.
    class PageBodyView : public UI::IWindow {
    public:
        PageBodyView();
        void setHost(ControlPanelHost* host) { host_ = host; }
        void onShow() override;
    private:
        // Calls the active IEmbeddedPanel's drawContents(); the actual bridge
        // to the still-immediate panels (kept for Phase 2, see plan).
        class PanelDrawView : public UI::IWindow {
        public:
            PanelDrawView() : UI::IWindow("PanelDraw") {}
            IEmbeddedPanel* panel = nullptr;
            void onShow() override { if (panel) panel->drawContents(); }
        };

        ControlPanelHost* host_ = nullptr;
        PanelDrawView     panelDraw_;
        UI::IdScope       idScope_ {"page"};
    };

    const PageSlot& activeSlot() const { return pages_[static_cast<std::size_t>(activePage_)]; }

    void loadLayout();
    void saveLayout() const;
    std::string pageHeaderText() const;

    std::array<PageSlot, kControlPageCount> pages_{};
    ControlPage activePage_ = ControlPage::Fluid;
    bool visible_ = true;
    UI::IWindow* statusView_ = nullptr;

    std::string layoutPath_;
    bool        layoutLoaded_ = false;
    bool        uiBuilt_ = false;
    ControlPage lastSavedPage_ = ControlPage::Fluid;
    bool        lastSavedVisible_ = true;

    // --- widget tree -------------------------------------------------------
    UI::Window    window_ {"Control"};
    UI::Separator statusSeparator_;
    UI::Label     pageHeaderLabel_ {[this] { return pageHeaderText(); }};
    UI::Separator headerSeparator_;
    UI::Label     unavailableLabel_ {
        [this] { return std::string("\"") + toString(activePage_) +
                        "\" is not available with the current setup."; },
        UI::Label::Style::Wrapped };
    UI::Label     unavailableReasonLabel_ {
        [this] { return activeSlot().disabledReason; }, UI::Label::Style::Wrapped };
    UI::Label     noPanelLabel_ {
        [this] { return std::string("\"") + toString(activePage_) +
                        "\" has no panel registered."; },
        UI::Label::Style::Wrapped };
    PageBodyView  pageBodyView_;
};

} // namespace Phantom
