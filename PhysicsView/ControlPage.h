#pragma once

#include <cstddef>

namespace Phantom {

/**
 * @brief The operation page currently shown in the shared Control window.
 *
 * Exactly one page is active at a time (see GUI_RESTRUCTURING_PLAN.md 5.1):
 * the Physics main menu selects it, ControlPanelHost renders it. The order
 * here is the menu order. (The former test-only "SSFR Test" page was folded
 * into the SSFR page as a collapsible "SSFR Debug" section on 2026-09-09.)
 */
enum class ControlPage {
    Fluid = 0,
    RigidBody,
    SoftBody,
    Flame,
    FluidRendering,
    SSFR,
    Rendering,
    VolumeConversion,
    ScenarioBrowser,
    Count,
};

inline constexpr std::size_t kControlPageCount =
    static_cast<std::size_t>(ControlPage::Count);

inline const char* toString(ControlPage page)
{
    switch (page) {
    case ControlPage::Fluid:            return "Fluid";
    case ControlPage::RigidBody:        return "Rigid Body";
    case ControlPage::SoftBody:         return "Soft Body";
    case ControlPage::Flame:            return "Flame";
    case ControlPage::FluidRendering:   return "Fluid Rendering";
    case ControlPage::SSFR:             return "SSFR";
    case ControlPage::Rendering:        return "glTF Rendering";
    case ControlPage::VolumeConversion: return "Volume / Mesh Conversion";
    case ControlPage::ScenarioBrowser:  return "Scenario Browser";
    default:                            return "?";
    }
}

} // namespace Phantom
