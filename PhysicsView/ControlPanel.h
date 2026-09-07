#pragma once

#include "../../CGLib/VkAppBase/IVkSubRenderer.h"
#include "../../CGLib/UIWidgets/IView.h"
#include "../../CGLib/UIWidgets/Button.h"
#include "../../CGLib/UIWidgets/BoolView.h"
#include "../../CGLib/UIWidgets/FloatView.h"
#include "../../CGLib/UIWidgets/FloatSlider.h"
#include "../../CGLib/UIWidgets/FloatDrag.h"
#include "../../CGLib/UIWidgets/IntSlider.h"
#include "../../CGLib/UIWidgets/ComboBox.h"
#include "../../CGLib/UIWidgets/Box3dView.h"
#include "../../CGLib/UIWidgets/Vector3dView.h"
#include "../../CGLib/UIWidgets/StringView.h"
#include "../../CGLib/UIWidgets/Label.h"
#include "../../CGLib/UIWidgets/Row.h"
#include "../../CGLib/UIWidgets/Section.h"
#include "../../CGLib/UIWidgets/Separator.h"
#include "../../CGLib/UIWidgets/IdScope.h"
#include "../../CGLib/UIWidgets/DisableScope.h"

#include "FluidWorld.h"
#include "IEmbeddedPanel.h"

#include <functional>
#include <string>

namespace Phantom {

/**
 * @brief Fluid (SPH) scene controls, assembled declaratively.
 *
 * The whole widget tree -- method combo, Simulation / Material / Initial
 * Region / Boundaries / Emitters / Outflow Regions / White Water sections --
 * is built once in buildUi() from the constructor. drawContents() is just
 * contents_.show() (docs/todo/PLAN_physicsview_declarative_ui.md Phase 3).
 *
 * Every model binding reaches world_->params() / whiteWaterParams() /
 * getEmitters() ... on each call, so a method switch or reset never leaves a
 * binding pointing at freed state. Method-scale conditions (isWCSPH / isDFSPH
 * / isGPU) are re-evaluated per frame via setVisibleWhen / DisableScope.
 * Reset-vs-immediate semantics are unchanged: the sliders/inputs write
 * params_ fields that reset() applies; White Water params are read live.
 */
class ControlPanel : public ::VKG::IVkUIPanel, public IEmbeddedPanel {
public:
    explicit ControlPanel(FluidWorld* world);

    void setOnWorldChanged(std::function<void()> fn) { onWorldChanged_ = std::move(fn); }
    void setVisible(bool visible) { visible_ = visible; }
    bool isVisible() const { return visible_; }

    void onImGui() override;
    void drawContents() override;

private:
    FluidWorld* world_ = nullptr;
    std::function<void()> onWorldChanged_;
    bool visible_ = true;

    bool uiBuilt_ = false;
    void buildUi();
    void notifyWorldChanged() { if (onWorldChanged_) onWorldChanged_(); }

    bool isWCSPH() const { return world_->getSimulationType() == FluidWorld::SimulationType::WCSPH; }
    bool isDFSPH() const { return world_->getSimulationType() == FluidWorld::SimulationType::DFSPH; }
    bool isGPU()   const { return world_->getSimulationType() == FluidWorld::SimulationType::GPU_CSPH; }

    std::string stepTimeText() const;
    std::string gpuProfileText() const;
    std::string emitterListText() const;
    std::string outflowListText() const;
    std::string triangleCountText() const;

    FluidWorld::Params&                          p()  { return world_->params(); }
    Phantom::Physics::WhiteWaterSystem::Params&   ww() { return world_->whiteWaterParams(); }

    UI::IView contents_ {"FluidControl"};

    // --- Simulation ----------------------------------------------------
    UI::Section  simSection_    {"Simulation"};
    UI::ComboBox methodCombo_   {"Method"};
    UI::FloatView timeStepView_ {"TimeStep", 0.01f};
    UI::Row      actionsRow_    {"Actions"};
    UI::Button   runButton_     {"Run/Pause"};
    UI::Button   stepButton_    {"Step"};
    UI::Button   resetButton_   {"Reset"};
    UI::Label    stepTimeLabel_ {[this] { return stepTimeText(); }};
    UI::Label    gpuProfileLabel_ {[this] { return gpuProfileText(); }};

    // --- Material -----------------------------------------------------
    UI::Section   materialSection_ {"Material"};
    UI::FloatView densityView_     {"Density", 1.0f};
    UI::FloatView viscosityView_   {"Viscosity", 5.0f};
    UI::FloatView pressureCoeScaleView_ {"Pressure Coe Scale", 1960.0f};
    UI::FloatView stiffnessView_   {"Stiffness", 20.0f};
    UI::DisableScope tensionScope_ {"TensionScope"};
    UI::FloatSlider  tensionSlider_ {"Surface Tension", 0.0f, 5.0f};

    // --- Initial Region --------------------------------------------
    UI::Section   initialRegionSection_ {"Initial Region"};
    UI::Label     appliedOnResetLabel_ {std::string("Applied on Reset."), UI::Label::Style::Disabled};
    UI::FloatView radiusView_    {"Radius", 1.0f};
    UI::FloatView effectLenView_ {"EffectLength", 2.25f};
    UI::IdScope   fluidBoundsScope_ {"FluidBounds"};
    UI::Box3dView fluidBoundsView_  {"Fluid Bounds"};

    // --- Boundaries ----------------------------------------------
    UI::Section   boundariesSection_ {"Boundaries"};
    UI::IdScope   boundaryScope_ {"Boundary"};
    UI::Box3dView boundaryView_  {"Boundary"};
    UI::DisableScope boundaryDampingScope_ {"BoundaryDampingScope"};
    UI::FloatSlider  boundaryDampingSlider_ {"Boundary Damping", 0.0f, 0.5f};
    UI::Separator meshBoundarySeparator_;
    UI::Label     meshBoundaryHeaderLabel_ {std::string("Mesh Boundary")};
    UI::StringView meshBoundaryPathView_ {"STL Path"};
    UI::Row       meshBoundaryRow_ {"MeshBoundaryRow"};
    UI::Button    loadMeshBoundaryButton_  {"Load##MeshBoundary"};
    UI::Button    clearMeshBoundaryButton_ {"Clear##MeshBoundary"};
    UI::Label     triangleCountLabel_ {[this] { return triangleCountText(); }};

    // --- Emitters ----------------------------------------------
    UI::Section     emittersSection_ {"Emitters", false};
    UI::Vector3dView emitterCenterView_    {"Center"};
    UI::FloatDrag   emitterRadiusDrag_     {"Disk Radius", 0.01f, 0.0f, 100.0f};
    UI::FloatDrag   emitterRateDrag_       {"Rate (particles/sec)", 1.0f, 0.0f, 100000.0f};
    UI::Vector3dView emitterDirectionView_ {"Direction", Phantom::Math::Vector3df(0.0f, 1.0f, 0.0f)};
    UI::FloatDrag   emitterSpeedDrag_      {"Speed", 0.01f, 0.0f, 1000.0f};
    UI::Row         emitterButtonsRow_ {"EmitterButtons"};
    UI::Button      addEmitterButton_   {"Add Emitter"};
    UI::Button      clearEmittersButton_ {"Clear All##Emitters"};
    UI::Label       emitterListLabel_ {[this] { return emitterListText(); }};

    // --- Outflow Regions ------------------------------------
    UI::Separator outflowSeparator_;
    UI::Section   outflowSection_ {"Outflow Regions", false};
    UI::IdScope   newOutflowScope_ {"NewOutflowRegion"};
    UI::Box3dView newOutflowRegionView_ {"New Outflow Region"};
    UI::Row       outflowButtonsRow_ {"OutflowButtons"};
    UI::Button    addOutflowButton_   {"Add Outflow Region"};
    UI::Button    clearOutflowButton_ {"Clear All##OutflowRegions"};
    UI::Label     outflowListLabel_ {[this] { return outflowListText(); }};

    // --- White Water ---------------------------------------
    UI::Separator whiteWaterSeparator_;
    UI::Section   whiteWaterSection_ {"White Water", false};
    UI::FloatSlider sprayVelThresholdSlider_ {"Spray Vel Threshold", 0.5f, 10.0f};
    UI::FloatSlider sprayDensityRatioSlider_ {"Spray Density Ratio", 0.1f, 1.0f};
    UI::FloatSlider foamCurvThresholdSlider_ {"Foam Curv Threshold", 0.1f, 2.0f};
    UI::FloatSlider foamNeighborCountSlider_ {"Foam Neighbor Count", 5.0f, 60.0f};
    UI::IntSlider   maxSprayParticlesSlider_ {"Max Spray Particles", 0, 100000};
    UI::IntSlider   maxFoamParticlesSlider_  {"Max Foam Particles", 0, 100000};
    UI::FloatSlider foamBuoyancySlider_ {"Foam Buoyancy", 0.0f, 5.0f};
    UI::FloatSlider foamDragSlider_     {"Foam Drag", 0.70f, 1.0f};
};

} // namespace Phantom
