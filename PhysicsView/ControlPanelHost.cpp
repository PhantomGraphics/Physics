#include "pch.h"
#include "ControlPanelHost.h"

#include <fstream>
#include <string>

namespace Phantom {

ControlPanelHost::ControlPanelHost() = default;

ControlPanelHost::PageBodyView::PageBodyView() : UI::IWindow("PageBody")
{
    idScope_.add(&panelDraw_);
}

void ControlPanelHost::PageBodyView::onShow()
{
    if (!host_) return;
    const auto& slot = host_->activeSlot();
    if (!slot.enabled || !slot.panel) return;  // notices are separate Labels

    panelDraw_.panel = slot.panel;
    idScope_.setId(toString(host_->getPage()));
    idScope_.show();
}

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

std::string ControlPanelHost::pageHeaderText() const
{
    return std::string("Page: ") + toString(activePage_);
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

void ControlPanelHost::init()
{
    if (!layoutLoaded_) loadLayout();
    if (uiBuilt_) return;
    uiBuilt_ = true;

    window_.setOpenFlag(&visible_);
    window_.setInitialPosition(10.f, 35.f);
    window_.setInitialSize(430.f, 640.f);

    pageBodyView_.setHost(this);

    statusSeparator_.setVisibleWhen([this] { return statusView_ != nullptr; });
    unavailableLabel_.setVisibleWhen([this] { return !activeSlot().enabled; });
    unavailableReasonLabel_.setVisibleWhen([this] {
        return !activeSlot().enabled && !activeSlot().disabledReason.empty();
    });
    noPanelLabel_.setVisibleWhen([this] {
        return activeSlot().enabled && activeSlot().panel == nullptr;
    });

    if (statusView_) window_.add(statusView_);
    window_.add(&statusSeparator_);
    window_.add(&pageHeaderLabel_);
    window_.add(&headerSeparator_);
    window_.add(&unavailableLabel_);
    window_.add(&unavailableReasonLabel_);
    window_.add(&noPanelLabel_);
    window_.add(&pageBodyView_);
}

void ControlPanelHost::onImGui()
{
    if (!uiBuilt_) init();

    if (!visible_) {
        if (lastSavedVisible_) { saveLayout(); lastSavedVisible_ = false; }
        return;
    }

    window_.show();

    if (activePage_ != lastSavedPage_ || visible_ != lastSavedVisible_) {
        saveLayout();
        lastSavedPage_ = activePage_;
        lastSavedVisible_ = visible_;
    }
}

} // namespace Phantom
