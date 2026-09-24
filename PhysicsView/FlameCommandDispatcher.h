#pragma once

#include "FlamePBVRPass.h"

#include <functional>
#include <optional>
#include <string>

namespace Phantom {

class FlameWorld;

/**
 * @brief Scenario command surface for the Flame (reacting hot-gas SPH) page
 * (docs/todo/PLAN_flame_sph_pbvr_improvement.md Phase 0).
 *
 * Not an IScenarioDispatcher of its own: CommandDispatcher::route() asks it
 * first and it answers synchronously, returning nullopt for anything that is
 * not a Flame command. Every command name contains "Flame", so there is no
 * overlap with the fluid/rigid/soft surfaces.
 *
 *   SetFlamePage:{true|false} / IsFlamePage
 *       Show the Flame page (it takes over the viewport) / go back to Fluid.
 *   FlameReset
 *       Rebuild the default flame scene. Resets every SetFlameParam value to
 *       its default too (the fluid object is rebuilt) -- set params AFTER it.
 *       The world-level time step and the render params survive.
 *   FlameStep / FlameStep:<n>
 *       Advance <n> fixed steps synchronously (independent of the Running flag).
 *   SetFlameRunning:{true|false} / IsFlameRunning
 *       Free-run while the Flame page is shown (frame-rate dependent; scenarios
 *       that assert on numbers should stay paused and use FlameStep).
 *   SetFlameParam:<name>,<value> / GetFlameParam:<name>
 *       Simulation parameter (FlameFluid setters, emitter[0], timeStep, ...).
 *       Unknown name / bad value -> "Error:...". GetFlameParam:list lists names.
 *   SetFlameRenderParam:<name>,<value> / GetFlameRenderParam:<name>
 *       Display-only parameter (FlameWorld::RenderParams); never affects the sim.
 *   SetFlameRenderMode:{Normal|PBVR} / GetFlameRenderMode
 *   GetFlameParticleCount           -> "Count:<n>" (primary SPH particles)
 *   GetFlameSecondaryCount          -> "Count:<n>" (sparks + smoke)
 *   GetFlameStats                   -> one-line FlameStats::toString() summary
 *   GetFlameStat:<name>             -> bare number (for expect_range), see FlameStats::names()
 *   GetFlameSimTime                 -> bare number (seconds since FlameReset)
 *   GetFlamePBVRStats               -> displayedEnsembles/ensemblesThisFrame/generated/overflowed/gpuMs/...
 *   GetFlamePBVRStat:<name>         -> bare number (one of the GetFlamePBVRStats keys)
 */
class FlameCommandDispatcher {
public:
	void setWorld(FlameWorld* w) { world_ = w; }

	/** @brief Page switch hooks into FluidApp's ControlPanelHost. */
	void setPageHooks(std::function<void(bool)> setFlamePage, std::function<bool()> isFlamePage)
	{
		setFlamePage_ = std::move(setFlamePage);
		isFlamePage_ = std::move(isFlamePage);
	}

	/** @brief Source of the PBVR renderer's statistics (GetFlamePBVRStats / GetFlamePBVRStat:<name>). */
	void setPBVRStatsHook(std::function<FlamePBVRPass::Stats()> fn) { pbvrStats_ = std::move(fn); }

	/** @brief Called after anything that changes what the flame renderer should show. */
	void setOnFlameChanged(std::function<void()> fn) { onFlameChanged_ = std::move(fn); }

	/** @brief Returns the response, or nullopt if cmd is not a Flame command. */
	std::optional<std::string> route(const std::string& cmd);

private:
	FlameWorld* world_ = nullptr;
	std::function<void(bool)> setFlamePage_;
	std::function<bool()> isFlamePage_;
	std::function<void()> onFlameChanged_;
	std::function<FlamePBVRPass::Stats()> pbvrStats_;

	void notifyChanged() { if (onFlameChanged_) onFlameChanged_(); }
};

} // namespace Phantom
