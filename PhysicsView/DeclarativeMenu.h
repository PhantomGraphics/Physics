#pragma once

#include "../../CGLib/UIWidgets/Menu.h"
#include "../../CGLib/UIWidgets/MenuItem.h"

#include <functional>
#include <list>
#include <string>
#include <vector>

namespace Phantom {

struct MenuItemDefinition {
    std::string label;
    std::function<void()> action;
    std::function<bool()> selected;
    std::function<bool()> enabled;
    std::function<std::string()> tooltip;
};

class DeclarativeMenu : public UI::Menu {
public:
    explicit DeclarativeMenu(const std::string& label) : UI::Menu(label) {}

    void build(const std::vector<MenuItemDefinition>& definitions) {
        for (const auto& definition : definitions) {
            items_.emplace_back(definition.label);
            UI::MenuItem& item = items_.back();
            item.setFunction(definition.action);
            if (definition.selected) item.setSelected(definition.selected);
            if (definition.enabled) item.setEnabled(definition.enabled);
            if (definition.tooltip) item.setTooltip(definition.tooltip);
            add(&item);
        }
    }

protected:
    std::list<UI::MenuItem> items_;
};

} // namespace Phantom
