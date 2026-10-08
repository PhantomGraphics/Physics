#include "pch.h"
#include "FlameControlPanel.h"
#include "RigidBodyWorld.h"
#include "../Physics/FlameStats.h"

namespace Phantom {

namespace Im = ::Phantom::UI::Immediate;

void FlameControlPanel::drawContents()
{
    if (!world_) return;

    Im::sliderFloat("New Sphere Radius",sphereRadius_,0.2f,1.2f);
    Im::sliderFloat("New Spacing",sphereSpacing_,0.04f,0.15f);
    if (Im::button("Spherical Convection")) { sphereSetupFailed_=!world_->sphericalPreset(false,sphereRadius_,sphereSpacing_); notifyWorldChanged(); return; }
    Im::sameLine();
    if (Im::button("Spherical Closed Flame")) { sphereSetupFailed_=!world_->sphericalPreset(true,sphereRadius_,sphereSpacing_); notifyWorldChanged(); return; }
    if(sphereSetupFailed_) Im::text("Invalid sphere setup: increase spacing (limit 12000 carriers).");
    if (Im::button("Continuous Closed Flame")) { world_->reset(); notifyWorldChanged(); return; }

    auto& fluid  = world_->fluid();
    auto& render = world_->render();

    Im::text("Particles: %d / %d", fluid.getNumParticles(), fluid.getMaxParticles());
	if (fluid.fixedCarriers) {
		if (const auto& sphere=world_->solver().getBoundarySphere())
			Im::text("Active sphere radius: %.3f",sphere->getRadius());
		if (const auto& cylinder=world_->solver().getBoundaryCylinder())
			Im::text("Active cylinder: radius %.3f / height %.3f",cylinder->getRadius(),2*cylinder->getHalfHeight());
		const char* debugModes[]={"Radiance","Temperature","Velocity"};
		Im::combo("Carrier Debug",render.carrierDebug,debugModes,3);
		Im::checkbox("Boundary Wire",render.sphereWire);
		Im::textDisabled("Fixed carriers / closed domain.\nPresets stop and rebuild the scene.");
		auto thermal = world_->solver().getThermalBoundary();
		bool changed = Im::sliderFloat("Source Power", thermal.sourcePower, 0, 100);
		changed |= Im::sliderFloat("Heat Time", thermal.sourceDuration, 0, 10);
		Im::textDisabled("Heat Time 0 = continuous heating.");
		changed |= Im::sliderFloat("Wall Temperature", thermal.wallTemperature, 250, 600);
		changed |= Im::sliderFloat("Wall Cooling Rate", thermal.wallRate, 0, 10);
		changed |= Im::sliderFloat("Outer Oxygen Recovery", thermal.oxygenRecoveryRate, 0, 10);
		changed |= Im::sliderFloat("Outer Smoke Decay", thermal.smokeDecayRate, 0, 10);
		changed |= Im::sliderFloat("Outer Velocity Damping", thermal.velocityDampingRate, 0, 10);
		if (changed) world_->solver().setThermalBoundary(thermal);
		const auto st = Physics::computeFlameStats(fluid, nullptr, &world_->solver());
		Im::text("Mean %.1f K / mass %.5f", st.allAvgT, st.carrierMass);
		Im::text("Core vy %.4f / outer vy %.4f", st.coreVelocityY, st.outerVelocityY);
		Im::text("Heat error %.6f / fuel error %.6f", st.heatBalanceError, st.fuelBalanceError);
	}

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
        return;
    }

    Im::sliderFloat("Particle Size", render.particleSize, 0.01f, 0.6f);

    if (Im::collapsingHeader("Object Combustion", true)) {
        if (Im::button("Finite Object Fuel Preset")) { world_->combustionPreset(); selectedBody_=0; notifyWorldChanged(); return; }
        Im::sameLine();
        if (Im::button("Stop Ignition Source")) { world_->stopSource(); notifyWorldChanged(); }
        Im::textDisabled("Visual material presets; positions / sizes in simulation units.");
        const char* shapes[]={"Box","Sphere"}; const char* materials[]={"Easy","Standard","Difficult","Inert"};
        Im::combo("New Shape",bodyShape_,shapes,2);
        Im::combo("New Material",materialPreset_,materials,4);
        Im::dragFloat3("New Center",bodyCenter_,0.01f,-1.0f,2.0f);
        Im::dragFloat3("Half Extent / Radius",bodyExtent_,0.005f,0.01f,0.4f);
        Im::sliderFloat("Initial Fuel Mass",bodyFuel_,0.0001f,0.03f,"%.4f");
        Im::sliderInt("Solid Particle Resolution",bodyResolution_,1,8);
        if (Im::button("Add Combustible Object")) {
            if(world_->addBody(bodyShape_==0?Physics::CombustibleBody::Shape::Box:Physics::CombustibleBody::Shape::Sphere,
                {bodyCenter_[0],bodyCenter_[1],bodyCenter_[2]}, {bodyExtent_[0],bodyExtent_[1],bodyExtent_[2]},bodyFuel_,materialPreset_,bodyResolution_))
                selectedBody_=static_cast<int>(world_->bodies().size())-1;
            notifyWorldChanged();
        }
        const int count=static_cast<int>(world_->bodies().size());
        if(count) {
            selectedBody_=std::clamp(selectedBody_,0,count-1);
            Im::sliderInt("Selected Object",selectedBody_,0,count-1);
            auto& b=*world_->bodies()[selectedBody_]; const auto s=b.stats();
            Im::text("Solid SPH particles: %d",static_cast<int>(b.samples().size()));
            Im::text("ID %llu: %s, %.1f K",static_cast<unsigned long long>(b.id()),Physics::combustionStateName(s.state),s.temperature);
            Im::text("Fuel %.1f%% / pending %.5f / residue %.5f",100*s.fuel/s.initialFuel,s.pending,s.residue);
            auto m=b.material(); bool changed=Im::checkbox("Combustible",m.combustible);
            if(Im::button("Apply New Material Preset to Selected")) { m=Physics::CombustibleMaterial::preset(materialPreset_); changed=true; }
            changed|=Im::sliderFloat("Pyrolysis Temperature",m.pyrolysisTemperature,350,1200);
            changed|=Im::sliderFloat("Pyrolysis Rate",m.pyrolysisRate,0,2);
            if(changed) b.setMaterial(m);
            const auto rigidId=world_->rigidBodyId(b.id());
            if(rigidId) {
                Im::text("Rigid ID %llu / shared Flame clock",static_cast<unsigned long long>(rigidId));
                if(Im::button("Unbind Rigid Body")) { world_->unbindRigidBody(b.id()); notifyWorldChanged(); }
            } else {
                auto p=b.center(); float position[]={p.x,p.y,p.z};
                if(Im::dragFloat3("Object Position",position,0.01f,-1.0f,2.0f)) b.setCenter({position[0],position[1],position[2]});
                auto angles=glm::degrees(glm::eulerAngles(b.orientation())); float rotation[]={angles.x,angles.y,angles.z};
                if(Im::dragFloat3("Object Rotation (degrees)",rotation,1,-180,180)) {
                    b.setMotion(b.center(),Math::Quaternion(glm::radians(glm::vec3(rotation[0],rotation[1],rotation[2]))),{0,0,0},{0,0,0});
                    b.beginMotionStep(); notifyWorldChanged();
                }
                if(auto* rigid=world_->rigidWorld()) {
                    const int count=static_cast<int>(rigid->getWorld().getBodies().size());
                    if(count>0) {
                        Im::sliderInt("Rigid Body Index",rigidIndex_,0,count-1);
                        if(Im::button("Bind to Rigid Body")) {
                            bindingFailed_=!world_->bindRigidBody(b.id(),rigidIndex_); notifyWorldChanged();
                        }
                        Im::textDisabled("Requires matching shape / scene size (simulation size x 12).\n"
                            "Rigid-fluid coupling must be off. Play/Pause/Step share the Flame clock.");
                        if(bindingFailed_) Im::text("Binding failed: check shape, size and coupling.");
                    }
                }
            }
            if(Im::button("Remove Selected Object")) { world_->removeBody(b.id()); notifyWorldChanged(); }
        }
        const char* colors[]={"Char / Residue","Temperature","Remaining Fuel"};
        Im::combo("Solid Color",world_->solidDebugColor,colors,3);
    }

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
        Im::sliderFloat("Smoke Density Profile", render.smokeDensityProfile, 0.0f, 1.0f);
        Im::textDisabled("Density: 0 uniform, 1 smooth poly6; soot amount is preserved.");

        if (render.pbvrMode) {
            Im::sliderFloat("Smoke Self Shadow", render.smokeShadowStrength, 0.0f, 4.0f);
            Im::sliderFloat("Smoke Shadow Fill", render.smokeShadowAmbient, 0.0f, 1.0f);
            Im::sliderFloat("Smoke Flame Light", render.smokeFlameLight, 0.0f, 20.0f);
            Im::sliderFloat("Object Flame Light", render.objectFlameLight, 0.0f, 20.0f);
            const auto& st = pbvrStats_ ? pbvrStats_() : FlamePBVRPass::Stats{};
            Im::text("Ensembles: %u shown, %u this frame, %.2f ms GPU",
                     st.displayedEnsembles, st.ensemblesThisFrame, st.gpuMs);
            Im::text("Sub-particles: %u (%u dropped)", st.generated, st.overflowed);
            Im::text("Light-space particles: %u (%u dropped)", st.shadowGenerated, st.shadowOverflowed);
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

    if (Im::collapsingHeader("Heat Haze", true)) {
        Im::textDisabled("Refracts the scene behind the flame (screen space).");
        Im::checkbox("Enabled", render.hazeEnabled);
        Im::sliderFloat("Strength (px)", render.hazeStrength, 0.0f, 24.0f);
        Im::sliderFloat("Extent", render.hazeExtent, 1.0f, 6.0f);
        Im::sliderFloat("Frequency", render.hazeFrequency, 2.0f, 40.0f);
        Im::sliderFloat("Rise Speed", render.hazeRiseSpeed, 0.0f, 2.0f);
    }

    if (Im::collapsingHeader("Display Transform", true)) {
        Im::textDisabled(
            "The flame sim runs at its native ~3-unit scale; these place it in the\n"
            "shared scene (origin = rigid / soft floor, 1 unit = 1 m at scale 1).");
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
            Im::sliderFloat("Fuel Supply", emitters[0].rate, 0.0f, 2000.0f);
            Im::sliderFloat("Local Oxygen Recovery", emitters[0].airRate, 0.0f, 2000.0f);
            Im::sliderFloat("Emitter Radius", emitters[0].radius, 0.01f, 0.5f);
            Im::sliderFloat("Fuel Heating Temperature (<0: ignition)", emitters[0].fuelTemperature, -1.0f, 2500.0f);
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

        Im::textDisabled("SPH carriers are retained after cooling and burnout.");
    }

    if (Im::collapsingHeader("Combustion Model", true)) {
        using Model = Phantom::Physics::FlameFluid::CombustionModel;
        using RateModel = Phantom::Physics::FlameFluid::ReactionRateModel;
        int model = fluid.getCombustionModel() == Model::Physical ? 1 : 0;
        const char* models[] = { "Legacy (first-order, T-independent)", "Physical (k(T) * fuel * O2)" };
        if(world_->bodies().empty()) {
            if (Im::combo("Model", model, models, 2)) fluid.setCombustionModel(model == 1 ? Model::Physical : Model::Legacy);
        } else Im::textDisabled("Object combustion uses Physical chemistry.");

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

        Im::textDisabled(fluid.fixedCarriers ? "SPH diffusion (length^2/s; stable substeps)" :
            "SPH diffusion (length^2/s; clamped to kappa*dt/h^2 <= 0.1)");
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
