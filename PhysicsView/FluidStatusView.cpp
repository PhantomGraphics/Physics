#include "pch.h"
#include "FluidStatusView.h"

#include "FluidWorld.h"
#include "SoftBodyWorld.h"
#include "RigidBodyWorld.h"
#include "../../CGLib/VkAppBase/ScenarioRunner/ScenarioRunner.h"

namespace Phantom {

namespace {
const char* runState(bool running) { return running ? "Running" : "Paused"; }
}

FluidStatusView::FluidStatusView() : UI::IView("FluidStatus")
{
    add(&methodLabel_);
    add(&particleLabel_);
    add(&couplingLabel_);
    add(&scenarioRunningLabel_);
    add(&scenarioFailedLabel_);

    scenarioRunningLabel_.setVisibleWhen([this] { return runner_ && runner_->isActive(); });
    scenarioFailedLabel_.setVisibleWhen([this] { return runner_ && runner_->hasFailed(); });
}

void FluidStatusView::bind(const FluidWorld* world, const SoftBodyWorld* softWorld,
                           const ScenarioRunner* runner)
{
    world_ = world;
    softWorld_ = softWorld;
    runner_ = runner;
}

std::string FluidStatusView::methodLine() const
{
    if (!world_) return {};
    static const char* const kMethodNames[] = { "DFSPH", "PBSPH", "WCSPH", "GPU CSPH" };
    const int mi = static_cast<int>(world_->getSimulationType());
    const char* method = (mi >= 0 && mi < 4) ? kMethodNames[mi] : "?";

    char buf[192];
    std::snprintf(buf, sizeof(buf), "%s  |  Fluid: %s  |  Rigid: %s  |  Soft: %s",
        method,
        runState(world_->isRunning()),
        runState(world_->rigid().isRunning()),
        runState(softWorld_ && softWorld_->isRunning()));
    return buf;
}

std::string FluidStatusView::particleLine() const
{
    if (!world_) return {};
    char buf[160];
    std::snprintf(buf, sizeof(buf), "Particles: %llu   (spray %llu, foam %llu)",
        static_cast<unsigned long long>(world_->getParticleCount()),
        static_cast<unsigned long long>(world_->getSprayPositions().size()),
        static_cast<unsigned long long>(world_->getFoamPositions().size()));
    return buf;
}

std::string FluidStatusView::couplingLine() const
{
    if (!world_) return {};
    const char* rigidCoupling = !world_->isCouplingEnabled()
        ? "off"
        : (world_->activeCouplingMode() == Phantom::Physics::CouplingMode::TwoWay
               ? "Two-Way" : "One-Way");
    const char* softCoupling = world_->isSoftCouplingEnabled() ? "on" : "off";
    char buf[128];
    std::snprintf(buf, sizeof(buf), "Coupling  Rigid-Fluid: %s   Soft-Fluid: %s",
        rigidCoupling, softCoupling);
    return buf;
}

std::string FluidStatusView::scenarioRunningLine() const
{
    if (!runner_) return {};
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Scenario: running (%llu steps)",
        static_cast<unsigned long long>(runner_->stepCount()));
    return buf;
}

std::string FluidStatusView::scenarioFailedLine() const
{
    if (!runner_) return {};
    return "Scenario FAILED: " + runner_->failMessage();
}

} // namespace Phantom
