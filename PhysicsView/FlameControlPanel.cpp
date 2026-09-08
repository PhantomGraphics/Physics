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

    Im::sliderFloat("Point Size", render.pointSize, 2.0f, 40.0f);

    if (Im::collapsingHeader("Render Mode", true)) {
        int mode = render.pbvrMode ? 1 : 0;
        const char* modes[] = { "Normal", "PBVR (stochastic, no sort)" };
        if (Im::combo("Mode", mode, modes, 2)) render.pbvrMode = (mode == 1);

        Im::sliderFloat("Smoke Opacity", render.smokeOpacityScale, 0.0f, 2.0f);

        if (render.pbvrMode) {
            Im::textDisabled(
                "Flame/spark and smoke particles alike survive with probability =\n"
                "their (scaled) opacity, then draw fully opaque -- one shared depth\n"
                "buffer, so no back-to-front sort is needed.");
            Im::sliderFloat("Flame Opacity", render.flameOpacityScale, 0.0f, 1.0f);
            Im::sliderInt("Repeat Count##pbvr", render.pbvrRepeatCount, 1, 100);
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
        Im::sliderFloat("Smoke Point Size", render.smokePointSize, 2.0f, 60.0f);
    }

    if (Im::collapsingHeader("Emitter", true)) {
        auto& emitters = fluid.getEmittersMutable();
        if (!emitters.empty()) {
            Im::sliderFloat("Rate (particles/sec)", emitters[0].rate, 0.0f, 2000.0f);
            Im::sliderFloat("Air Rate (particles/sec)", emitters[0].airRate, 0.0f, 2000.0f);
            Im::sliderFloat("Emitter Radius", emitters[0].radius, 0.01f, 0.5f);
        }
    }

    if (Im::collapsingHeader("Combustion", true)) {
        float ambient = fluid.getAmbientTemperature();
        if (Im::sliderFloat("Ambient Temperature", ambient, 0.0f, 1000.0f)) fluid.setAmbientTemperature(ambient);

        float ignition = fluid.getIgnitionTemperature();
        if (Im::sliderFloat("Ignition Temperature", ignition, 500.0f, 3000.0f)) fluid.setIgnitionTemperature(ignition);

        float burnRate = fluid.getBurnRate();
        if (Im::sliderFloat("Burn Rate", burnRate, 0.0f, 5.0f)) fluid.setBurnRate(burnRate);

        float heatRelease = fluid.getHeatRelease();
        if (Im::sliderFloat("Heat Release", heatRelease, 0.0f, 5000.0f)) fluid.setHeatRelease(heatRelease);

        float coolRate = fluid.getCoolRate();
        if (Im::sliderFloat("Cool Rate", coolRate, 0.0f, 10.0f)) fluid.setCoolRate(coolRate);

        float sootYield = fluid.getSootYield();
        if (Im::sliderFloat("Soot Yield", sootYield, 0.0f, 1.0f)) fluid.setSootYield(sootYield);

        float lifeMax = fluid.getLifeMax();
        if (Im::sliderFloat("Life Max (s)", lifeMax, 0.5f, 15.0f)) fluid.setLifeMax(lifeMax);
    }

    if (Im::collapsingHeader("Buoyancy / Vorticity / Noise", true)) {
        float buoyancyCoe = fluid.getBuoyancyCoe();
        if (Im::sliderFloat("Buoyancy Coe", buoyancyCoe, 0.0f, 20.0f)) fluid.setBuoyancyCoe(buoyancyCoe);

        float thermalExpansion = fluid.getThermalExpansion();
        if (Im::sliderFloat("Thermal Expansion", thermalExpansion, 0.0f, 0.2f)) fluid.setThermalExpansion(thermalExpansion);

        float vorticityEps = fluid.getVorticityEps();
        if (Im::sliderFloat("Vorticity Eps", vorticityEps, 0.0f, 10.0f)) fluid.setVorticityEps(vorticityEps);

        float curlStrength = fluid.getCurlNoiseStrength();
        if (Im::sliderFloat("Curl Noise Strength", curlStrength, 0.0f, 3.0f)) fluid.setCurlNoiseStrength(curlStrength);

        float curlFrequency = fluid.getCurlNoiseFrequency();
        if (Im::sliderFloat("Curl Noise Frequency", curlFrequency, 0.0f, 2.0f)) fluid.setCurlNoiseFrequency(curlFrequency);

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
