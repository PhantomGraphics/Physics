#include "pch.h"
#include "FlameWorld.h"

#include "../../CGLib/Math/Box3d.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace Phantom {

namespace {
constexpr float kFlameStep = 1.0f / 60.0f;
}

FlameWorld::FlameWorld()
{
    reset();
}

void FlameWorld::reset()
{
    fluid_  = std::make_unique<FlameFluid>();
    solver_ = std::make_unique<FlameSolver>();
    buildScene();
}

void FlameWorld::buildScene()
{
    // Identical numbers to the former FlameView::FlameApp::setupInitialScene().
    fluid_->setEffectLength(0.12f);
    fluid_->setDensity(1.0f);
    fluid_->setPressureCoe(20.0f);
    fluid_->setVicosityCoe(0.001f);
    fluid_->setMaxParticles(4000);
    fluid_->setMaxSpeed(3.0f);

    FlameFluid::Emitter e;
    e.center = Vector3df(0.0f, 0.0f, 0.0f);
    e.radius = 0.12f;
    e.rate   = 250.0f;
    // Co-emit ambient "air" particles so freshly ignited particles have SPH
    // neighbours from frame one instead of spawning into a near-vacuum.
    e.airRate = 150.0f;
    fluid_->addEmitter(e);

    // Cosmetic secondary particles (non-SPH): population targets, not flat rates.
    fluid_->setSparkCountPerPrimary(6.0f);
    fluid_->setSmokeCountPerPrimary(14.0f);

    solver_->add(fluid_.get());
    solver_->setEffectLength(fluid_->getEffectLength());
    solver_->setBoundary(Box3df(Vector3df(-1.5f, -0.05f, -1.5f),
                                Vector3df(1.5f, 4.0f, 1.5f)), 0.01f);
}

void FlameWorld::step()
{
    if (!running_) return;
    solver_->simulate(kFlameStep);
}

void FlameWorld::stepOnce()
{
    solver_->simulate(kFlameStep);
}

} // namespace Phantom
