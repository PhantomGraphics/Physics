#include "CloudWorld.h"

#include "../Physics/CloudThermodynamics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

namespace Phantom {

using Physics::CloudSourceParams;
using Math::Vector3dd;
namespace CT = Physics::CloudThermodynamics;

CloudWorld::CloudWorld() = default;

void CloudWorld::syncSolver()
{
    solver_.params = config_.solver;
    solver_.thermo = config_.thermo;
    solver_.environment = config_.environment;
}

void CloudWorld::reset(int64_t seed)
{
    built_ = true;
    if (seed >= 0) config_.seed = static_cast<uint32_t>(seed);
    syncSolver();
    soa_.clear();
    ledger_ = Physics::CloudWaterLedger();
    Physics::CloudOps::fillEnvironmentLattice(soa_, config_.thermo, config_.environment,
                                              config_.solver.domainMin, config_.solver.domainMax,
                                              config_.solver.spacing, ledger_);
    if (config_.jitter > 0.0) {
        std::mt19937 rng(config_.seed);
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        const double amp = config_.jitter * config_.solver.spacing;
        for (Vector3dd& p : soa_.positions) {
            // Bounded so jittered particles stay inside the domain.
            p.x = std::clamp(p.x + amp * u(rng), config_.solver.domainMin.x, config_.solver.domainMax.x);
            p.y = std::clamp(p.y + amp * u(rng), config_.solver.domainMin.y, config_.solver.domainMax.y);
            p.z = std::clamp(p.z + amp * u(rng), config_.solver.domainMin.z, config_.solver.domainMax.z);
        }
    }
    solver_.initialize(soa_);
    solver_.resetDiagnostics();
    simTime_ = 0.0;
    stepCount_ = 0;
    overloadedFrames_ = 0;
    droppedSimTime_ = 0.0;
}

double CloudWorld::chooseDt(double remaining) const
{
    double dt = solver_.stableTimeStep(soa_);
    if (config_.fixedTimeStep > 0.0) dt = std::min(dt, config_.fixedTimeStep);
    return std::min(dt, remaining);
}

double CloudWorld::run(double simSeconds, int maxSubsteps)
{
    syncSolver();
    double remaining = simSeconds;
    int steps = 0;
    while (remaining > 1.0e-12 && steps < maxSubsteps) {
        // A forced fixed dt is honoured exactly (stableTimeStep only caps it).
        const double dt = chooseDt(remaining);
        if (!(dt > 0.0) || !solver_.step(soa_, sources_, simTime_, dt, ledger_)) break;
        simTime_ += dt;
        remaining -= dt;
        ++stepCount_;
        ++steps;
    }
    return remaining > 1.0e-12 ? remaining : 0.0;
}

void CloudWorld::update(double realDt)
{
    ensureBuilt();
    if (!running_ || !(realDt > 0.0)) return;
    const double left = run(realDt * config_.timeScale, std::max(1, config_.maxSubstepsPerFrame));
    if (left > 0.0) {
        ++overloadedFrames_;
        droppedSimTime_ += left;
    }
}

void CloudWorld::advance(double simSeconds)
{
    ensureBuilt();
    if (!(simSeconds > 0.0)) return;
    run(simSeconds, 1 << 30);
}

void CloudWorld::stepOnce()
{
    ensureBuilt();
    run(chooseDt(1.0e30), 1);
}

size_t CloudWorld::addSource(const CloudSourceParams& s)
{
    sources_.push_back(s);
    return sources_.size() - 1;
}

bool CloudWorld::setSource(size_t index, const CloudSourceParams& s)
{
    if (index >= sources_.size()) return false;
    sources_[index] = s;
    return true;
}

bool CloudWorld::removeSource(size_t index)
{
    if (index >= sources_.size()) return false;
    sources_.erase(sources_.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
}

void CloudWorld::setRelativeHumidity(double rh)
{
    ensureBuilt();
    rh = std::clamp(rh, 0.0, 1.0);
    const double old = config_.environment.relativeHumidity;
    config_.environment.relativeHumidity = rh;
    syncSolver();
    if (old <= 0.0 && rh <= 0.0) return;
    // Rescale the existing vapour: qv' = qv + (rh - old) * qvs(env) at each particle's height.
    double delta = 0.0;
    for (size_t i = 0; i < soa_.size(); ++i) {
        const double z = soa_.positions[i].z;
        const double qs = CT::saturationMixingRatio(config_.thermo, CT::environmentTemperature(config_.environment, z),
                                                    CT::environmentPressure(config_.thermo, config_.environment, z));
        const double before = soa_.qv[i];
        soa_.qv[i] = std::max(0.0, before + (rh - old) * qs);
        delta += soa_.mDry[i] * (soa_.qv[i] - before);
    }
    ledger_.suppliedWater += delta;   // external supply (or removal when negative)
}

Physics::CloudStats CloudWorld::stats() const
{
    Physics::CloudStats s = Physics::CloudOps::computeStats(soa_, ledger_, config_.qcThreshold);
    s.adjustFailures = solver_.diagnostics().adjustFailures;
    return s;
}

std::string CloudWorld::csvHeader()
{
    return "simTime,particles,totalWater,vapor,cloudWater,cloudBase,cloudTop,cloudyParticles,"
           "balanceError,maxSpeed,maxDensityRatio,nonFinite,negative,adjustFailures,overloadedFrames";
}

std::string CloudWorld::csvRow() const
{
    const Physics::CloudStats s = stats();
    const Physics::CloudSolverDiagnostics& d = solver_.diagnostics();
    char buf[512];
    std::snprintf(buf, sizeof(buf), "%.6f,%zu,%.9g,%.9g,%.9g,%.6g,%.6g,%zu,%.6g,%.6g,%.6g,%zu,%zu,%zu,%llu",
                  simTime_, s.particleCount, s.totalWater, s.totalVapor, s.totalCloudWater, s.cloudBase,
                  s.cloudTop, s.cloudyParticles, s.balanceError, d.maxSpeed, d.maxDensityRatio,
                  s.nonFiniteCount, s.negativeCount, s.adjustFailures,
                  static_cast<unsigned long long>(overloadedFrames_));
    return buf;
}

}
