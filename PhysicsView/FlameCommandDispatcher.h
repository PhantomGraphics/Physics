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
 *   FlameSpherePreset:{convection|combustion}[,<radius>,<spacing>]
 *       Stops/rebuilds a fixed-carrier sphere (defaults 0.6 / 0.08).
 *       Valid radius 0.05..10, spacing >=0.005, at most 12000 carriers.
 *       Fixed mode rejects solid fuel release and live density changes.
 *       sourcePower/sourceDuration/sourceRadius/wallTemperature/wallRate/wallThickness
 *       are exposed by SetFlameParam. StopFlameSource stops spherical heating too.
 *       carrierDebug (0 radiance, 1 temperature, 2 velocity) and sphereWire
 *       are exposed by SetFlameRenderParam.
 *   FlameStep / FlameStep:<n>
 *       Advance <n> fixed steps (independent of the Running flag). The steps are spread
 *       over frames within a time budget (tick()) so the window stays responsive during
 *       long runs; the answer ("OK") is deferred until the last step is done, and the
 *       dispatcher is busy() (CommandDispatcher holds back later commands) until then.
 *       The step count -- not the frame rate -- decides the result, so it stays deterministic.
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

	/**
	 * @brief Returns the response, or nullopt if cmd is not a Flame command.
	 * An empty string means "answer comes later" (FlameStep:<n>): see tick().
	 */
	std::optional<std::string> route(const std::string& cmd);

	/** @brief True while a FlameStep:<n> still has steps left. */
	bool busy() const { return stepsRemaining_ > 0; }

	/** @brief Receives the deferred answer of a finished FlameStep:<n>. */
	void setDeferredDone(std::function<void(const std::string&)> fn) { deferredDone_ = std::move(fn); }

	/**
	 * @brief Runs pending FlameStep work for at most ~budgetMs (always at least one
	 * step), then answers "OK" once the last step is done. Call once per frame.
	 */
	void tick(double budgetMs);

private:
	FlameWorld* world_ = nullptr;
	std::function<void(bool)> setFlamePage_;
	std::function<bool()> isFlamePage_;
	std::function<void()> onFlameChanged_;
	std::function<FlamePBVRPass::Stats()> pbvrStats_;
	std::function<void(const std::string&)> deferredDone_;
	int stepsRemaining_ = 0;

	void notifyChanged() { if (onFlameChanged_) onFlameChanged_(); }
};

} // namespace Phantom
