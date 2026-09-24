#include "pch.h"
#include "FlameControlPanel.h"

namespace Phantom {

namespace Im = ::Phantom::UI::Immediate;

void FlameControlPanel::drawContents()
{
    if (!world_) return;

    auto& fluid  = world_->fluid();
    auto& render = world_->render();

    Im::text("Particles: %d / %d", fluid.getNumParticles(), fluid.getMaxParticles());

    bool running = world_->isRunning();
    if (Im::checkbox("Running", running)) world_->setRunning(running);
    Im::sameLine();
    if (Im::button("Step") && !running) {
        world_->stepOnce();
        notifyWorldChanged();
    }
    Im::sameLine();
    if (Im::button("Reset")) {
        world_->reset();
        notifyWorldChanged();
    }

    Im::sliderFloat("Particle Size", render.particleSize, 0.01f, 0.6f);

    if (Im::collapsingHeader("Render Mode", true)) {
        int mode = render.pbvrMode ? 1 : 0;
        const char* modes[] = { "Normal (splat)", "PBVR (GPU, ensemble-averaged)" };
        if (Im::combo("Mode", mode, modes, 2)) render.pbvrMode = (mode == 1);

        Im::textDisabled("Emission: blackbody HDR, radiance 1 at the reference temperature.");
        Im::sliderFloat("Exposure", render.exposure, 0.0f, 8.0f);
        Im::checkbox("Auto Reference Temperature", render.autoReferenceTemperature);
        if (!render.autoReferenceTemperature) {
            Im::sliderFloat("Reference Temperature (K)", render.referenceTemperature, 500.0f, 4000.0f);
        }
        const char* wbModes[] = { "Off (colorimetric D65)", "Auto (hottest = white)", "Manual" };
        Im::combo("White Balance", render.whiteBalance, wbModes, 3);
        if (render.whiteBalance == 2) {
            Im::sliderFloat("White Temperature (K)", render.whiteBalanceTemperature, 800.0f, 6500.0f);
        }
        if (render.whiteBalance != 0) {
            Im::sliderFloat("Adaptation Degree", render.whiteBalanceDegree, 0.0f, 1.0f);
        }
        Im::textDisabled("Absorption: soot smoke, optical depth = extinction * density.");
        Im::sliderFloat("Smoke Density Scale", render.smokeOpacityScale, 0.0f, 4.0f);
        Im::sliderFloat("Smoke Extinction", render.smokeExtinction, 0.0f, 40.0f);
        Im::sliderFloat("Smoke Glow", render.smokeGlow, 0.0f, 1.0f);
        Im::sliderFloat("Smoke Albedo", render.smokeAlbedo, 0.0f, 0.5f);

        if (render.pbvrMode) {
            const auto& st = pbvrStats_ ? pbvrStats_() : FlamePBVRPass::Stats{};
            Im::text("Ensembles: %u shown, %u this frame, %.2f ms GPU",
                     st.displayedEnsembles, st.ensemblesThisFrame, st.gpuMs);
            Im::text("Sub-particles: %u (%u dropped)", st.generated, st.overflowed);
            Im::checkbox("Adaptive R (GPU budget)", render.pbvrAdaptive);
            if (render.pbvrAdaptive) {
                Im::sliderFloat("GPU Budget (ms)", render.pbvrBudgetMs, 1.0f, 33.0f);
            } else {
                Im::sliderInt("Ensembles per Frame", render.pbvrEnsembles, 1, 8);
            }
            Im::sliderInt("Target Ensembles (static)", render.pbvrTargetEnsembles, 1, 1024);
            Im::sliderFloat("Temporal Frames (running)", render.pbvrTemporalFrames, 0.0f, 16.0f);
            Im::sliderFloat("Subdivision", render.pbvrSubdivision, 1.0f, 8.0f);
            Im::sliderFloat("Min Sub-particle (px)", render.pbvrMinSubPixels, 0.5f, 8.0f);
            Im::sliderFloat("Density Scale", render.pbvrDensityScale, 0.0f, 4.0f);
        }
    }

    if (Im::collapsingHeader("Display Transform", true)) {
        Im::textDisabled(
            "The flame sim runs at its native ~3-unit scale; these map it onto\n"
            "PhysicsView's shared camera (centred on 20,20,20).");
        Im::sliderFloat("Render Scale", render.renderScale, 1.0f, 40.0f);
        float offset[3] = { render.renderOffset.x, render.renderOffset.y, render.renderOffset.z };
        if (Im::dragFloat3("Render Offset", offset, 0.5f, -200.0f, 200.0f)) {
            render.renderOffset = glm::vec3(offset[0], offset[1], offset[2]);
        }
        Im::sliderFloat("Smoke Particle Size", render.smokeParticleSize, 0.01f, 1.0f);
    }

    if (Im::collapsingHeader("Emitter", true)) {
        auto& emitters = fluid.getEmittersMutable();
        if (!emitters.empty()) {
            Im::sliderFloat("Rate (particles/sec)", emitters[0].rate, 0.0f, 2000.0f);
            Im::sliderFloat("Air Rate (particles/sec)", emitters[0].airRate, 0.0f, 2000.0f);
            Im::sliderFloat("Emitter Radius", emitters[0].radius, 0.01f, 0.5f);
            Im::sliderFloat("Fuel Spawn Temperature (<0: ignition)", emitters[0].fuelTemperature, -1.0f, 2500.0f);
            Im::sliderFloat("Pilot Temperature (0: off)", emitters[0].pilotTemperature, 0.0f, 2500.0f);
            Im::sliderFloat("Pilot Height", emitters[0].pilotHeight, 0.0f, 1.0f);
        }
    }

    if (Im::collapsingHeader("Combustion", true)) {
        float ambient = fluid.getAmbientTemperature();
        if (Im::sliderFloat("Ambient Temperature", ambient, 0.0f, 1000.0f)) fluid.setAmbientTemperature(ambient);

        float ignition = fluid.getIgnitionTemperature();
        if (Im::sliderFloat("Ignition Temperature", ignition, 500.0f, 3000.0f)) fluid.setIgnitionTemperature(ignition);

        float burnRate = fluid.getBurnRate();
        if (Im::sliderFloat("Burn Rate", burnRate, 0.0f, 30.0f)) fluid.setBurnRate(burnRate);

        float heatRelease = fluid.getHeatRelease();
        if (Im::sliderFloat("Heat Release", heatRelease, 0.0f, 5000.0f)) fluid.setHeatRelease(heatRelease);

        float coolRate = fluid.getCoolRate();
        if (Im::sliderFloat("Cool Rate", coolRate, 0.0f, 10.0f)) fluid.setCoolRate(coolRate);

        float sootYield = fluid.getSootYield();
        if (Im::sliderFloat("Soot Yield", sootYield, 0.0f, 1.0f)) fluid.setSootYield(sootYield);

        float lifeMax = fluid.getLifeMax();
        if (Im::sliderFloat("Life Max (s)", lifeMax, 0.5f, 15.0f)) fluid.setLifeMax(lifeMax);
    }

    if (Im::collapsingHeader("Combustion Model", true)) {
        using Model = Phantom::Physics::FlameFluid::CombustionModel;
        using RateModel = Phantom::Physics::FlameFluid::ReactionRateModel;
        int model = fluid.getCombustionModel() == Model::Physical ? 1 : 0;
        const char* models[] = { "Legacy (first-order, T-independent)", "Physical (k(T) * fuel * O2)" };
        if (Im::combo("Model", model, models, 2)) fluid.setCombustionModel(model == 1 ? Model::Physical : Model::Legacy);

        if (fluid.getCombustionModel() == Model::Physical) {
            int rate = fluid.getReactionRateModel() == RateModel::Arrhenius ? 1 : 0;
            const char* rates[] = { "Smoothstep ignition window", "Arrhenius exp(-Ta/T)" };
            if (Im::combo("k(T)", rate, rates, 2)) fluid.setReactionRateModel(rate == 1 ? RateModel::Arrhenius : RateModel::Smoothstep);

            float width = fluid.getIgnitionWidth();
            if (Im::sliderFloat("Ignition Width (K)", width, 0.0f, 600.0f)) fluid.setIgnitionWidth(width);
            float ta = fluid.getActivationTemperature();
            if (Im::sliderFloat("Activation Temperature (K)", ta, 1000.0f, 40000.0f)) fluid.setActivationTemperature(ta);
            float nu = fluid.getOxygenPerFuel();
            if (Im::sliderFloat("Oxygen per Fuel", nu, 0.0f, 4.0f)) fluid.setOxygenPerFuel(nu);
            float tmax = fluid.getMaxTemperature();
            if (Im::sliderFloat("Max Temperature (K)", tmax, 1000.0f, 4000.0f)) fluid.setMaxTemperature(tmax);
        }

        Im::textDisabled("SPH diffusion (length^2/s; clamped to kappa*dt/h^2 <= 0.1)");
        float kT = fluid.getThermalDiffusivity();
        if (Im::sliderFloat("Thermal Diffusivity", kT, 0.0f, 0.1f)) fluid.setThermalDiffusivity(kT);
        float kF = fluid.getFuelDiffusivity();
        if (Im::sliderFloat("Fuel Diffusivity", kF, 0.0f, 0.1f)) fluid.setFuelDiffusivity(kF);
        float kO = fluid.getOxygenDiffusivity();
        if (Im::sliderFloat("Oxygen Diffusivity", kO, 0.0f, 0.1f)) fluid.setOxygenDiffusivity(kO);
        float kS = fluid.getSootDiffusivity();
        if (Im::sliderFloat("Soot Diffusivity", kS, 0.0f, 0.1f)) fluid.setSootDiffusivity(kS);

        bool expansion = fluid.getThermalExpansionPressure();
        if (Im::checkbox("Thermal-Expansion Pressure (rho0*Tamb/T)", expansion)) fluid.setThermalExpansionPressure(expansion);
    }

    if (Im::collapsingHeader("Buoyancy / Vorticity / Noise", true)) {
        float buoyancyCoe = fluid.getBuoyancyCoe();
        if (Im::sliderFloat("Buoyancy Coe", buoyancyCoe, 0.0f, 20.0f)) fluid.setBuoyancyCoe(buoyancyCoe);

        float thermalExpansion = fluid.getThermalExpansion();
        if (Im::sliderFloat("Thermal Expansion", thermalExpansion, 0.0f, 0.2f)) fluid.setThermalExpansion(thermalExpansion);

        float vorticityEps = fluid.getVorticityEps();
        if (Im::sliderFloat("Vorticity Eps", vorticityEps, 0.0f, 10.0f)) fluid.setVorticityEps(vorticityEps);

        float curlStrength = fluid.getCurlNoiseStrength();
        if (Im::sliderFloat("Curl Noise Strength (m/s^2)", curlStrength, 0.0f, 180.0f)) fluid.setCurlNoiseStrength(curlStrength);

        float curlFrequency = fluid.getCurlNoiseFrequency();
        if (Im::sliderFloat("Curl Noise Frequency", curlFrequency, 0.0f, 2.0f)) fluid.setCurlNoiseFrequency(curlFrequency);

        float curlTimeScale = fluid.getCurlNoiseTimeScale();
        if (Im::sliderFloat("Curl Noise Time Scale", curlTimeScale, 0.0f, 3.0f)) fluid.setCurlNoiseTimeScale(curlTimeScale);

        float maxSpeed = fluid.getMaxSpeed();
        if (Im::sliderFloat("Max Speed (m/s)", maxSpeed, 0.5f, 10.0f)) fluid.setMaxSpeed(maxSpeed);
    }

    if (Im::collapsingHeader("Secondary Particles (Sparks / Smoke)", true)) {
        Im::text("Count: %d / %d",
                 static_cast<int>(fluid.getSecondaryParticles().size()),
                 fluid.getMaxSecondaryParticles());

        int maxSecondary = fluid.getMaxSecondaryParticles();
        if (Im::sliderInt("Max Secondary Particles", maxSecondary, 1000, 300000)) fluid.setMaxSecondaryParticles(maxSecondary);

        float sparkPerPrimary = fluid.getSparkCountPerPrimary();
        if (Im::sliderFloat("Sparks per Primary", sparkPerPrimary, 0.0f, 50.0f)) fluid.setSparkCountPerPrimary(sparkPerPrimary);

        float sparkLifeMax = fluid.getSparkLifeMax();
        if (Im::sliderFloat("Spark Life Max (s)", sparkLifeMax, 0.1f, 3.0f)) fluid.setSparkLifeMax(sparkLifeMax);

        float sparkSpeed = fluid.getSparkSpeed();
        if (Im::sliderFloat("Spark Speed", sparkSpeed, 0.0f, 5.0f)) fluid.setSparkSpeed(sparkSpeed);

        float sparkSize = fluid.getSparkSize();
        if (Im::sliderFloat("Spark Size", sparkSize, 0.05f, 1.5f)) fluid.setSparkSize(sparkSize);

        float smokePerPrimary = fluid.getSmokeCountPerPrimary();
        if (Im::sliderFloat("Smoke Puffs per Primary", smokePerPrimary, 0.0f, 50.0f)) fluid.setSmokeCountPerPrimary(smokePerPrimary);

        float smokeLifeMax = fluid.getSmokeLifeMax();
        if (Im::sliderFloat("Smoke Life Max (s)", smokeLifeMax, 0.5f, 10.0f)) fluid.setSmokeLifeMax(smokeLifeMax);

        float smokeRiseSpeed = fluid.getSmokeRiseSpeed();
        if (Im::sliderFloat("Smoke Rise Speed", smokeRiseSpeed, 0.0f, 2.0f)) fluid.setSmokeRiseSpeed(smokeRiseSpeed);

        float swirlStrength = fluid.getSecondarySwirlStrength();
        if (Im::sliderFloat("Vorticity Swirl Strength", swirlStrength, 0.0f, 5.0f)) fluid.setSecondarySwirlStrength(swirlStrength);
    }
}

} // namespace Phantom
