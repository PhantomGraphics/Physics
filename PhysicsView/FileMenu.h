#pragma once
#include "DeclarativeMenu.h"
#include "../../CGLib/UIWidgets/Separator.h"
#include <utility>
namespace Phantom { class FileMenu : public DeclarativeMenu {
public: FileMenu() : DeclarativeMenu("File") {}
    void build(std::function<void()> newAction, std::function<void()> quitAction) {
        DeclarativeMenu::build({{"New", std::move(newAction)}});
        add(&separator_);
        DeclarativeMenu::build({{"Quit", std::move(quitAction)}});
    }
private: UI::Separator separator_;
}; }
