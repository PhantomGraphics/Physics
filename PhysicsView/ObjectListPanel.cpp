#include "pch.h"
#include "ObjectListPanel.h"

namespace Phantom {

void ObjectListPanel::buildUi()
{
    if (uiBuilt_) return;
    uiBuilt_ = true;

    window_.setOpenFlag(&visible_);
    window_.setInitialPosition(700.f, 35.f);
    window_.setInitialSize(340.f, 470.f);
    window_.add(&summaryLabel_);
    window_.add(&separator_);
    window_.add(&listLabel_);
}

std::string ObjectListPanel::summaryText() const
{
    if (!registry_) return "Objects: (none)";
    const auto& c = *registry_;
    char buf[192];
    std::snprintf(buf, sizeof(buf),
        "Objects: %d   (fluid %d, rigid %d, soft %d, boundary %d, emitter %d, outflow %d)",
        static_cast<int>(c.components().size()),
        c.count(SceneComponentKind::Fluid),
        c.count(SceneComponentKind::RigidBody),
        c.count(SceneComponentKind::SoftBody),
        c.count(SceneComponentKind::MeshBoundary),
        c.count(SceneComponentKind::Emitter),
        c.count(SceneComponentKind::OutflowRegion));
    return buf;
}

std::string ObjectListPanel::listText() const
{
    if (!registry_ || registry_->components().empty())
        return "(no objects)";

    std::string s;
    for (const auto& comp : registry_->components()) {
        char head[32];
        std::snprintf(head, sizeof(head), "#%-3d %-13s ", comp.id, comp.label.c_str());
        s += head;
        s += comp.describe ? comp.describe() : std::string{};
        s += '\n';
    }
    if (!s.empty() && s.back() == '\n') s.pop_back();
    return s;
}

void ObjectListPanel::onImGui()
{
    if (!uiBuilt_) buildUi();
    window_.show();
}

} // namespace Phantom
