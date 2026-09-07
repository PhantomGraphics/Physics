#pragma once

#include "CGLib/UIWidgets/IView.h"
#include "CGLib/UIWidgets/Label.h"

#include <string>

class ScenarioRunner;

namespace Phantom {

class FluidWorld;
class SoftBodyWorld;

/**
 * @brief 共通ステータス表示（Control ウィンドウ上部、全ページ共通）．
 *
 * 旧 FluidApp::drawStatusArea() + ControlPanelHost::setStatusDrawer() の
 * 任意描画コールバックを、provider 駆動の Label を並べた複合ビューに置き換えたもの
 * （docs/todo/PLAN_physicsview_declarative_ui.md Phase 2）．
 *
 * 状態は毎フレーム provider が読み取るだけで、モデルは変更しない．
 * bind() を一度だけ呼んでツリーを構築する．
 */
class FluidStatusView : public UI::IView {
public:
    FluidStatusView();

    void bind(const FluidWorld* world, const SoftBodyWorld* softWorld,
              const ScenarioRunner* runner);

private:
    std::string methodLine() const;
    std::string particleLine() const;
    std::string couplingLine() const;
    std::string scenarioRunningLine() const;
    std::string scenarioFailedLine() const;

    const FluidWorld*     world_     = nullptr;
    const SoftBodyWorld*  softWorld_ = nullptr;
    const ScenarioRunner* runner_    = nullptr;

    UI::Label methodLabel_   {[this] { return methodLine(); }};
    UI::Label particleLabel_ {[this] { return particleLine(); }};
    UI::Label couplingLabel_ {[this] { return couplingLine(); }};
    UI::Label scenarioRunningLabel_ {[this] { return scenarioRunningLine(); }};
    UI::Label scenarioFailedLabel_  {[this] { return scenarioFailedLine(); }, UI::Label::Style::Wrapped};
};

} // namespace Phantom
