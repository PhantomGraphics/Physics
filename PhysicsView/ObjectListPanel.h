#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/Window.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Separator.h"

#include "SceneComponent.h"

#include <string>

namespace Phantom {

/**
 * @brief Standalone window listing every scene object by id.
 *
 * Separate from the "Control" window (toggled from the View menu). Read-only:
 * a summary line plus one line per SceneComponent -- id, category and the
 * component's live describe() detail. Assembled once; drawn via window_.show()
 * with its close box wired to visible_ (docs/todo/PLAN_physicsview_declarative_ui.md
 * declarative style).
 */
class ObjectListPanel : public ::VKG::IVkUIPanel {
public:
    void bind(const SceneComponentRegistry* registry) { registry_ = registry; }
    void setVisible(bool v) { visible_ = v; }
    bool isVisible() const { return visible_; }

    void onImGui() override;

private:
    void buildUi();
    std::string summaryText() const;
    std::string listText() const;

    const SceneComponentRegistry* registry_ = nullptr;
    bool visible_ = true;
    bool uiBuilt_ = false;

    UI::Window    window_       {"Scene Objects"};
    UI::Label     summaryLabel_ {[this] { return summaryText(); }, UI::Label::Style::Wrapped};
    UI::Separator separator_;
    UI::Label     listLabel_    {[this] { return listText(); }};
};

} // namespace Phantom
