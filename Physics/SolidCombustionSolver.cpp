#include "pch.h"
#include "SolidCombustionSolver.h"
#include <algorithm>
#include <cmath>
using namespace Phantom::Physics;

double SolidCombustionSolver::exchange(float& a, double ca, float& b, double cb, double g, float dt)
{
    if (ca <= 0 || cb <= 0 || g <= 0 || dt <= 0) return 0;
    const double q = (a-b) * (-std::expm1(-g * (1/ca+1/cb) * dt)) / (1/ca+1/cb);
    a -= static_cast<float>(q/ca); b += static_cast<float>(q/cb);
    return q;
}

void SolidCombustionSolver::update(CombustibleBody& body, float dt) const
{
    if (!std::isfinite(dt) || dt <= 0) return;
    auto& samples = body.samples(); const auto& m = body.material();
    for (size_t i = 0; i < samples.size(); ++i) for (const auto j : samples[i].neighbors) if (j > i) {
        auto& a = samples[i]; auto& b = samples[j];
        const float distance = glm::length(a.position-b.position);
        // Surface-cell conduction: edge width / distance. Degree-normalized
        // areas prevent extra graph edges from increasing the effective flux.
        const double g = m.conductivity * std::sqrt(std::min(a.area,b.area)) /
            std::max(distance,1e-6f) * 4.0 / std::max(a.neighbors.size(),b.neighbors.size());
        exchange(a.temperature,body.heatCapacity(a),b.temperature,body.heatCapacity(b),g,dt);
    }
    for (auto& s : samples) {
        s.temperature = ambientTemperature + (s.temperature-ambientTemperature) * std::exp(-cooling*dt);
        if (!m.combustible || s.fuel <= 0 || s.temperature <= m.pyrolysisTemperature) continue;
        const double volatileFraction = 1.0-m.residueFraction;
        if (volatileFraction <= 0) continue;
        const double room = std::max(0.0, s.initialFuel * pendingFractionLimit - s.pending);
        double amount = std::min(s.fuel, s.area * 0.05 * m.pyrolysisRate * dt *
            std::clamp((s.temperature-m.pyrolysisTemperature)/100.0f,0.0f,1.0f));
        amount = std::min(amount, room/volatileFraction);
        const double capacity = body.heatCapacity(s);
        if (m.latentHeat > 0) { const double excess=s.temperature-m.pyrolysisTemperature;
            amount = std::min(amount, capacity*excess/(m.latentHeat+volatileFraction*m.specificHeat*excess)); }
        const double remainingCapacity=capacity-amount*volatileFraction*m.specificHeat;
        s.pendingHeat+=amount*volatileFraction*m.specificHeat*(s.temperature-ambientTemperature);
        s.temperature -= static_cast<float>(amount*m.latentHeat/remainingCapacity);
        s.fuel = std::max(0.0,s.fuel-amount); s.residue += amount*m.residueFraction;
        s.pending += amount*volatileFraction; body.pyrolyzed += amount;
    }
}
