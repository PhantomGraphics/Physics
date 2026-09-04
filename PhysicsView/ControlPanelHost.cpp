#include "pch.h"
#include "ControlPanelHost.h"

#include <fstream>
#include <string>

namespace Phantom {

void ControlPanelHost::loadLayout()
{
    layoutLoaded_ = true;
    if (layoutPath_.empty()) return;

    std::ifstream f(layoutPath_);
    if (!f) return;
    std::string key;
    int page = 0, vis = 1;
    bool gotPage = false, gotVis = false;
    while (f >> key) {
        if (key == "page")         { f >> page; gotPage = true; }
        else if (key == "visible") { f >> vis;  gotVis = true; }
    }
    if (gotPage && page >= 0 && page < static_cast<int>(kControlPageCount))
        activePage_ = static_cast<ControlPage>(page);
    if (gotVis)
        visible_ = (vis != 0);

    lastSavedPage_ = activePage_;
    lastSavedVisible_ = visible_;
}

void ControlPanelHost::saveLayout() const
{
    if (layoutPath_.empty()) return;
    std::ofstream f(layoutPath_, std::ios::trunc);
    if (!f) return;
    f << "page " << static_cast<int>(activePage_) << '\n'
      << "visible " << (visible_ ? 1 : 0) << '\n';
}

void ControlPanelHost::registerPage(ControlPage page, IEmbeddedPanel* panel)
{
    if (page == ControlPage::Count) return;
    pages_[static_cast<std::size_t>(page)].panel = panel;
}

void ControlPanelHost::setPage(ControlPage page)
{
    if (page == ControlPage::Count) return;
    activePage_ = page;
}

bool ControlPanelHost::isPageRegistered(ControlPage page) const
{
    if (page == ControlPage::Count) return false;
    return pages_[static_cast<std::size_t>(page)].panel != nullptr;
}

void ControlPanelHost::setPageEnabled(ControlPage page, bool enabled, const std::string& reason)
{
    if (page == ControlPage::Count) return;
    auto& slot = pages_[static_cast<std::size_t>(page)];
    slot.enabled = enabled;
    slot.disabledReason = reason;
}

bool ControlPanelHost::isPageEnabled(ControlPage page) const
{
    if (page == ControlPage::Count) return false;
    return pages_[static_cast<std::size_t>(page)].enabled;
}

const std::string& ControlPanelHost::pageDisabledReason(ControlPage page) const
{
    static const std::string kEmpty;
    if (page == ControlPage::Count) return kEmpty;
    return pages_[static_cast<std::size_t>(page)].disabledReason;
}

void ControlPanelHost::onImGui()
{
    if (!layoutLoaded_) loadLayout();

    if (!visible_) {
        if (lastSavedVisible_) { saveLayout(); lastSavedVisible_ = false; }
        return;
    }

    UI::Immediate::setNextWindowPosition(10.f, 35.f);
    UI::Immediate::setNextWindowSize(430.f, 640.f);
    if (!UI::Immediate::beginWindow("Control", &visible_)) {
        UI::Immediate::endWindow();
        return;
    }

    if (statusDrawer_) {
        statusDrawer_();
        UI::Immediate::separator();
    }

    const auto& slot = pages_[static_cast<std::size_t>(activePage_)];
    UI::Immediate::text("Page: %s", toString(activePage_));
    UI::Immediate::separator();

    if (!slot.enabled) {
        UI::Immediate::textWrapped("\"%s\" is not available with the current setup.",
                                   toString(activePage_));
        if (!slot.disabledReason.empty())
            UI::Immediate::textWrapped("%s", slot.disabledReason.c_str());
    } else if (!slot.panel) {
        UI::Immediate::textWrapped("\"%s\" has no panel registered.",
                                   toString(activePage_));
    } else {
        UI::Immediate::pushId(toString(activePage_));
        slot.panel->drawContents();
        UI::Immediate::popId();
    }

    UI::Immediate::endWindow();

    if (activePage_ != lastSavedPage_ || visible_ != lastSavedVisible_) {
        saveLayout();
        lastSavedPage_ = activePage_;
        lastSavedVisible_ = visible_;
    }
}

} // namespace Phantom
