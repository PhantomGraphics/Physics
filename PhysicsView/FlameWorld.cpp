#include "pch.h"
#include "FlameWorld.h"

#include "../../CGLib/Math/Box3d.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace Phantom {

FlameWorld::FlameWorld()
{
    reset();
}

void FlameWorld::reset()
{
    fluid_  = std::make_unique<FlameFluid>();
    solver_ = std::make_unique<FlameSolver>();
    simTime_ = 0.0f;
    buildScene();
}

void FlameWorld::buildScene()
{
    // SPH numbers from the former FlameView::FlameApp::setupInitialScene();
    // combustion/emitter/noise re-tuned for the Physical model (plan Phase 2).
    fluid_->setEffectLength(0.12f);
    fluid_->setDensity(1.0f);
    fluid_->setPressureCoe(20.0f);
    fluid_->setVicosityCoe(0.001f);
    fluid_->setMaxParticles(4000);
    fluid_->setMaxSpeed(3.0f);

    // Gentle flicker: an 8 m/s^2 curl-noise acceleration. (The historical
    // value was 0.5 per step without dt = 30 m/s^2 at 1/60 s, which kept
    // nearly every particle pinned at the maxSpeed cap -- plan A1 / Phase 2.)
    fluid_->setCurlNoiseStrength(8.0f);

    FlameFluid::Emitter e;
    e.center = Vector3df(0.0f, 0.0f, 0.0f);
    e.radius = 0.12f;
    e.rate   = 150.0f;
    // Co-emit ambient "air" particles so freshly ignited particles have SPH
    // neighbours from frame one instead of spawning into a near-vacuum -- and,
    // under the Physical model, so the fuel has oxygen to mix with. 2:1 air
    // keeps the mixture lean enough to burn out instead of leaving an
    // oxygen-starved fuel plume.
    e.airRate = 300.0f;
    // Physical combustion model (the FlameFluid default): fuel vapor leaves
    // the emitter preheated but below ignition and with no oxygen, so it only
    // burns where it has mixed with the air carriers. The wick-like pilot
    // keeps the base of that mixing layer above ignition so the flame stays
    // lit and anchored; above it the reaction's own heat release (spread by
    // the thermal diffusion pass) is what keeps the flame burning.
    e.fuelTemperature  = 700.0f;
    e.pilotTemperature = 1500.0f;
    e.pilotHeight      = 0.15f;
    fluid_->addEmitter(e);

    // Cosmetic secondary particles (non-SPH): population targets, not flat rates.
    fluid_->setSparkCountPerPrimary(6.0f);
    // 5 puffs per primary (was 14): at 14 the closed domain filled with smoke
    // thick enough that the order-independent PBVR mode (correctly) hid the
    // flame inside it -- Normal mode had only masked that by drawing the flame
    // on top of the smoke regardless of depth.
    fluid_->setSmokeCountPerPrimary(5.0f);

    solver_->add(fluid_.get());
    solver_->setEffectLength(fluid_->getEffectLength());
    solver_->setBoundary(Box3df(Vector3df(-1.5f, -0.05f, -1.5f),
                                Vector3df(1.5f, 4.0f, 1.5f)), 0.01f);
}

void FlameWorld::step()
{
    if (!running_) return;
    stepOnce();
}

void FlameWorld::stepOnce()
{
    solver_->simulate(timeStep_);
    simTime_ += timeStep_;
}

} // namespace Phantom
