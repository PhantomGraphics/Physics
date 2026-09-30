#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

namespace Phantom {

class CloudWorld;

/**
 * @brief Scenario command surface for the Cloud (moist-air SPH) page
 * (docs/todo/PLAN_cloud_sph_pbvr.md Phase 2). The same operations the
 * CloudControlPanel drives, so UI and scenarios share one API.
 *
 * Not an IScenarioDispatcher of its own: CommandDispatcher::route() asks it and it
 * answers synchronously; nullopt = not a Cloud command. Every command name contains
 * "Cloud", so there is no overlap with the other surfaces.
 *
 *   SetCloudPage:{true|false} / IsCloudPage
 *   CloudReset / CloudReset:<seed>       Rebuild the air lattice (sources are kept).
 *                                        Lattice/environment params apply here.
 *   CloudStep / CloudStep:<n>            n solver steps (automatic dt), any Running state.
 *   CloudAdvance:<seconds>               Advance simulated seconds synchronously.
 *   SetCloudRunning:{true|false} / IsCloudRunning
 *   SetCloudParam:<name>,<value> / GetCloudParam:<name>   (GetCloudParam:list lists names)
 *   AddCloudSource:cx,cy,cz,radius,heatingRate,vaporRate,startTime,endTime -> "Index:<i>"
 *   SetCloudSource:<i>,cx,cy,cz,radius,heatingRate,vaporRate,startTime,endTime
 *   RemoveCloudSource:<i> / ClearCloudSources / GetCloudSourceCount
 *   SetCloudHumidity:<rh 0..1>           Acts on existing air; recorded in the water ledger.
 *   SetCloudWind:x,y,z
 *   GetCloudParticleCount, GetCloudSimTime
 *   GetCloudStats                        One CSV row; GetCloudCsvHeader gives the columns.
 *   GetCloudStat:<name>                  Bare number (names: statDefs() in the .cpp).
 *   GetCloudPbvrStat:<accumulated|overflowed|generated>  PBVR counters (Error if the renderer is not ready).
 */
class CloudCommandDispatcher {
public:
    void setWorld(CloudWorld* w) { world_ = w; }
    void setPageHooks(std::function<void(bool)> setPage, std::function<bool()> isPage)
    {
        setPage_ = std::move(setPage);
        isPage_ = std::move(isPage);
    }
    /** @brief PBVR counters for GetCloudPbvrStat:<accumulated|overflowed|generated> (-1 if unavailable). */
    void setPbvrStatsHook(std::function<std::optional<std::array<double, 3>>()> fn) { pbvrStats_ = std::move(fn); }
    /** @brief Called after anything that changes what the cloud display should show. */
    void setOnCloudChanged(std::function<void()> fn) { onChanged_ = std::move(fn); }

    std::optional<std::string> route(const std::string& cmd);

private:
    CloudWorld* world_ = nullptr;
    std::function<void(bool)> setPage_;
    std::function<bool()> isPage_;
    std::function<void()> onChanged_;
    std::function<std::optional<std::array<double, 3>>()> pbvrStats_;

    void notifyChanged() { if (onChanged_) onChanged_(); }
};

} // namespace Phantom
