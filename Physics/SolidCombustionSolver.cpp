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
        double g = m.conductivity * std::sqrt(std::min(a.area,b.area)) /
            std::max(distance,1e-6f) * 4.0 / std::max(a.neighbors.size(),b.neighbors.size());
        if(body.usesSolidParticles()) {
            // Brookshaw SPH Laplacian with the compact-support spiky kernel.
            // Volumes are m/rho. Each unordered pair transfers equal/opposite heat.
            const double h=body.smoothingLength(), r=distance;
            constexpr double pi=3.141592653589793;
            const double gradient=45.0/(pi*std::pow(h,6))*std::pow(h-r,2);
            g=2*m.conductivity*a.volume*b.volume*r*gradient/(r*r+0.01*h*h);
        }
        exchange(a.temperature,body.heatCapacity(a),b.temperature,body.heatCapacity(b),g,dt);
    }
    for (auto& s : samples) {
        s.temperature = ambientTemperature + (s.temperature-ambientTemperature) * std::exp(-cooling*dt);
        if (!m.combustible || s.fuel <= 0 || s.temperature <= m.pyrolysisTemperature) continue;
        const double volatileFraction = 1.0-m.residueFraction;
        if (volatileFraction <= 0) continue;
        const double room = std::max(0.0, s.initialFuel * pendingFractionLimit - s.pending);
        const auto e=body.halfExtent();
        const double surfaceToVolume=body.shape()==CombustibleBody::Shape::Box?
            1.0/e.x+1.0/e.y+1.0/e.z:3.0/e.x;
        const double reactiveArea=body.usesSolidParticles()?s.volume*surfaceToVolume:s.area;
        double amount = std::min(s.fuel, reactiveArea * 0.05 * m.pyrolysisRate * dt *
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
    if(body.usesSolidParticles()) {
        // Vapor migrates through the porous solid to a neighboring exposed
        // particle. Limited pair flux retains mass and sensible heat when full.
        for(size_t i=0;i<samples.size();++i) for(auto j:samples[i].neighbors) if(j>i) {
            auto* a=&samples[i]; auto* b=&samples[j];
            if(a->pending/a->volume < b->pending/b->volume) std::swap(a,b);
            const double difference=a->pending/a->volume-b->pending/b->volume;
            const double h=body.smoothingLength(), r=glm::length(a->position-b->position);
            const double gradient=45.0/(3.141592653589793*std::pow(h,6))*std::pow(h-r,2);
            const double conductance=2*0.01*a->volume*b->volume*r*gradient/(r*r+0.01*h*h);
            const double inverseVolume=1.0/a->volume+1.0/b->volume;
            const double amount=std::min({a->pending, std::max(0.0,b->initialFuel*pendingFractionLimit-b->pending),
                difference*(-std::expm1(-conductance*inverseVolume*dt))/inverseVolume});
            if(amount<=0) continue;
            const double heat=a->pendingHeat*amount/a->pending;
            a->pending-=amount; b->pending+=amount; a->pendingHeat-=heat; b->pendingHeat+=heat;
        }
    }
}
