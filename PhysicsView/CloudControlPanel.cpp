#include "pch.h"
#include "CloudControlPanel.h"

#include <algorithm>

namespace Phantom {

namespace Im = ::Phantom::UI::Immediate;

namespace {

// The config is double precision; the widget facade is float.
bool sliderD(const char* label, double& v, float lo, float hi)
{
    float f = static_cast<float>(v);
    if (Im::sliderFloat(label, f, lo, hi)) {
        v = f;
        return true;
    }
    return false;
}

} // namespace

void CloudControlPanel::drawContents()
{
    if (!world_) return;
    CloudWorld& w = *world_;
    w.ensureBuilt();
    auto& cfg = w.config();

    const Physics::CloudStats s = w.stats();
    const auto& d = w.diagnostics();
    Im::text("Air particles: %zu   t = %.1f s   steps = %llu",
             s.particleCount, w.simTime(), static_cast<unsigned long long>(w.stepCount()));

    bool running = w.isRunning();
    if (Im::checkbox("Running", running)) w.setRunning(running);
    Im::sameLine();
    if (Im::button("Step") && !running) {
        w.stepOnce();
        notifyWorldChanged();
    }
    Im::sameLine();
    if (Im::button("Reset")) {
        w.reset();
        notifyWorldChanged();
    }
    Im::sliderInt("Seed", pendingSeed_, 0, 9999);
    Im::sameLine();
    if (Im::button("Reset with Seed")) {
        w.reset(pendingSeed_);
        notifyWorldChanged();
    }
    sliderD("Time Scale", cfg.timeScale, 0.1f, 30.0f);

    if (Im::collapsingHeader("Environment", true)) {
        double rh = cfg.environment.relativeHumidity;
        if (sliderD("Relative Humidity", rh, 0.0f, 1.0f)) {
            w.setRelativeHumidity(rh);
            notifyWorldChanged();
        }
        Math::Vector3dd wind = cfg.environment.wind;
        bool windChanged = sliderD("Wind X (m/s)", wind.x, -20.0f, 20.0f);
        windChanged |= sliderD("Wind Y (m/s)", wind.y, -20.0f, 20.0f);
        if (windChanged) w.setWind(wind);
        Im::textDisabled("Lattice / temperature profile changes apply at Reset.");
    }

    if (Im::collapsingHeader("Sources", true)) {
        const int count = static_cast<int>(w.sources().size());
        if (Im::button("Add Source")) {
            Physics::CloudSourceParams src;
            src.center = Math::Vector3dd(0.5 * cfg.solver.domainMax.x, 0.5 * cfg.solver.domainMax.y,
                                         0.1 * cfg.solver.domainMax.z);
            src.radius = 0.1 * cfg.solver.domainMax.x;
            src.heatingRate = 0.5;
            src.vaporRate = 100.0;
            src.startTime = w.simTime();
            selectedSource_ = static_cast<int>(w.addSource(src));
            notifyWorldChanged();
        }
        if (count > 0) {
            selectedSource_ = std::clamp(selectedSource_, 0, count - 1);
            Im::sameLine();
            if (Im::button("Remove")) {
                w.removeSource(static_cast<size_t>(selectedSource_));
                notifyWorldChanged();
                return;
            }
            Im::sliderInt("Source", selectedSource_, 0, count - 1);
            Physics::CloudSourceParams src = w.sources()[static_cast<size_t>(selectedSource_)];
            bool ch = false;
            ch |= Im::checkbox("Enabled", src.enabled);
            ch |= sliderD("Center X", src.center.x, 0.0f, static_cast<float>(cfg.solver.domainMax.x));
            ch |= sliderD("Center Y", src.center.y, 0.0f, static_cast<float>(cfg.solver.domainMax.y));
            ch |= sliderD("Center Z", src.center.z, 0.0f, static_cast<float>(cfg.solver.domainMax.z));
            ch |= sliderD("Radius (m)", src.radius, 10.0f, 500.0f);
            ch |= sliderD("Heating (K/s)", src.heatingRate, 0.0f, 5.0f);
            ch |= sliderD("Vapour (kg/s)", src.vaporRate, 0.0f, 20000.0f);
            ch |= sliderD("Start (s)", src.startTime, 0.0f, 3600.0f);
            double endShown = std::min(src.endTime, 3600.0);
            if (sliderD("End (s)", endShown, 0.0f, 3600.0f)) {
                src.endTime = endShown;
                ch = true;
            }
            if (ch) w.setSource(static_cast<size_t>(selectedSource_), src);
        } else {
            Im::textDisabled("No sources: the air stays clear.");
        }
    }

    if (Im::collapsingHeader("Physics", false)) {
        Im::checkbox("Pressure", cfg.solver.enablePressure);
        Im::checkbox("Buoyancy", cfg.solver.enableBuoyancy);
        Im::checkbox("Adiabatic cooling", cfg.solver.enableAdiabatic);
        Im::checkbox("Condensation", cfg.solver.enableCondensation);
        Im::checkbox("Mixing", cfg.solver.enableMixing);
        sliderD("Sound Speed (m/s)", cfg.solver.soundSpeed, 10.0f, 200.0f);
        sliderD("Viscosity (m2/s)", cfg.solver.viscosity, 0.0f, 50.0f);
        sliderD("Mix Diffusivity (m2/s)", cfg.solver.mixDiffusivity, 0.0f, 200.0f);
        sliderD("Wind Relax (s)", cfg.solver.windRelaxTime, 0.0f, 600.0f);
    }

    if (Im::collapsingHeader("Rendering", true)) {
        auto& rp = w.render();
        if (Im::checkbox("Volume (raymarched)", rp.volumeMode)) notifyWorldChanged();
        if (rp.volumeMode) {
            int res = static_cast<int>(rp.gridResolution);
            if (Im::sliderInt("Density Grid", res, 16, 128)) {
                rp.gridResolution = static_cast<uint32_t>(res);
                notifyWorldChanged();
            }
            if (sliderD("Kernel Support (x spacing)", rp.supportScale, 0.5f, 4.0f)) notifyWorldChanged();
            sliderD("Extinction (m2/kg)", rp.extinction, 0.0f, 600.0f);
            sliderD("Albedo", rp.albedo, 0.0f, 1.0f);
            sliderD("Phase g", rp.phaseG, -0.9f, 0.95f);
            sliderD("Sun Azimuth", rp.sunAzimuthDeg, -180.0f, 180.0f);
            sliderD("Sun Elevation", rp.sunElevationDeg, 1.0f, 90.0f);
            sliderD("Sun Irradiance", rp.sunIrradiance, 0.0f, 10.0f);
            sliderD("Ambient", rp.ambient, 0.0f, 2.0f);
            sliderD("Step (x cell)", rp.stepScale, 0.2f, 2.0f);
            Im::textDisabled("Opaque scene depth is not sampled yet.");
        } else {
            Im::textDisabled("Air particles coloured by cloud water (diagnostic).");
        }
    }

    if (Im::collapsingHeader("Statistics", true)) {
        Im::text("Total water   %.4g kg", s.totalWater);
        Im::text("Cloud water   %.4g kg  (%zu cloudy)", s.totalCloudWater, s.cloudyParticles);
        Im::text("Base / top    %.0f / %.0f m", s.cloudBase, s.cloudTop);
        Im::text("Balance error %.3g", s.balanceError);
        Im::text("Max speed     %.2f m/s   density ratio %.3f..%.3f", d.maxSpeed, d.minDensityRatio,
                 d.maxDensityRatio);
        Im::text("Overloaded frames: %llu (dropped %.1f s)",
                 static_cast<unsigned long long>(w.overloadedFrames()), w.droppedSimTime());
        if (s.nonFiniteCount || s.negativeCount || s.adjustFailures) {
            Im::text("INVALID: %zu non-finite, %zu negative, %zu adjust failures", s.nonFiniteCount,
                     s.negativeCount, s.adjustFailures);
        }
    }
}

} // namespace Phantom
