#include "pch.h"
#include "FlameSolidCoupler.h"
#include "FlameFluid.h"
#include <algorithm>
#include <cmath>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

bool FlameSolidCoupler::add(CombustibleBody* body)
{
    if (!body || !body->id()) return false;
    for (const auto* b : bodies_) if (b->id() == body->id()) return false;
    bodies_.push_back(body); initialFuelMass+=body->stats().initialFuel; return true;
}
void FlameSolidCoupler::remove(uint64_t id)
{
    for(const auto* b:bodies_) if(b->id()==id) { const auto s=b->stats(); removedMass+=s.fuel+s.pending+s.residue; }
    bodies_.erase(std::remove_if(bodies_.begin(),bodies_.end(),[id](const auto* b){return b->id()==id;}),bodies_.end());
}
bool FlameSolidCoupler::occluded(const Vector3df& a, const Vector3df& b) const
{
    for (const auto* body : bodies_) if (body->blocksSegment(a,b)) return true;
    return false;
}

void FlameSolidCoupler::update(FlameFluid& fluid, float dt, double time)
{
    if (!std::isfinite(dt) || dt <= 0 || exchangeDistance <= 0) return;
    auto& gas = fluid.getParticles();
    // Surface-area-weighted, normalized kernel for each cell. Increasing gas
    // count does not multiply surface conductance. Only the outward hemisphere
    // is visible; segment tests also reject thin-wall and third-body occlusion.
    for (auto* body : bodies_) {
        body->reactionRate = 0;
        for (auto& s : body->samples()) {
            const Vector3df surface = body->surfacePosition(s);
            std::vector<std::pair<size_t,float>> near;
            float sum = 0;
            for (size_t i = 0; i < gas.size(); ++i) {
                const Vector3df d = gas.positions[i]-surface; const float distance = glm::length(d);
                if (distance >= exchangeDistance || glm::dot(d,s.normal) < 0 || occluded(surface,gas.positions[i])) continue;
                const float weight = 1-distance/exchangeDistance;
                near.emplace_back(i,weight); sum += weight;
                body->reactionRate += fluid.computeReactionRate(gas.temperatures[i],gas.fuels[i],gas.oxygens[i]) *
                    FlameParticle(gas,i,&fluid).getMass() * weight * s.area;
            }
            for (const auto& [i,weight] : near) {
                const double gasCapacity = FlameParticle(gas,i,&fluid).getMass()*gasSpecificHeat;
                body->heatExchange += SolidCombustionSolver::exchange(gas.temperatures[i],gasCapacity,
                    s.temperature,body->heatCapacity(s),heatTransfer*s.area*weight/sum,dt);
            }
        }
        if (body->reactionRate > 1e-8f && body->material().combustible && body->pyrolyzed > 1e-12) {
            body->wasBurning = true;
            if (body->firstIgnitionTime < 0) body->firstIgnitionTime = time;
        }
        solidSolver.ambientTemperature = fluid.getAmbientTemperature();
        solidSolver.update(*body,dt);
    }
    // Fixed-size carriers: fraction = emitted mass / carrier mass. No pilot,
    // no injected oxidizer. Gas receives only the current solid temperature.
    for (auto* body : bodies_) for (auto& s : body->samples()) {
        const double mass=std::min(static_cast<double>(fluid.getDensity())*8*0.0125*0.0125*0.0125,
            s.initialFuel*solidSolver.pendingFractionLimit*0.5);
        const float radius=static_cast<float>(std::cbrt(mass/(8*fluid.getDensity())));
        if (s.pending <= 1e-14 || mass <= 0 || fluid.getMaxParticles()<=0 || gas.size() >= static_cast<size_t>(fluid.getMaxParticles())) continue;
        const Vector3df position = body->surfacePosition(s)+s.normal*(radius+1e-4f);
        bool blocked = false;
        for (const auto* other : bodies_) if (other->signedDistance(position) < radius) { blocked = true; break; }
        if (blocked) continue;
        // Avoid spending a whole carrier on a tiny cell every substep. Flush
        // the final fraction once exhausted, otherwise accumulate to 1/4 mass.
        if (s.pending < mass && s.fuel > s.initialFuel*1e-6) continue;
        const double amount = std::min(mass,s.pending);
        const double heat=s.pendingHeat*amount/s.pending;
        const float temperature=fluid.getAmbientTemperature()+static_cast<float>(heat/(mass*gasSpecificHeat));
        gas.push_back(position,radius,fluid.getDensity(),temperature);
        gas.fuels.back() = static_cast<float>(amount/mass); gas.oxygens.back() = 0;
        gas.velocities.back() = s.normal*emissionSpeed;
        s.pending -= amount; s.pendingHeat-=heat; body->emitted += amount;
    }
}

void FlameSolidCoupler::constrain(FlameFluid& fluid, const std::vector<Vector3df>& previous) const
{
    auto& gas = fluid.getParticles();
    for (size_t i = 0; i < gas.size(); ++i) for (const auto* body : bodies_) {
        Vector3df normal;
        const float distance = body->signedDistance(gas.positions[i],&normal);
        if (i < previous.size() && body->blocksSegment(previous[i],gas.positions[i]) && body->signedDistance(previous[i]) >= gas.radii[i]) {
            // Swept collision prevents crossing a plate in one integration.
            float lo=0,hi=1;
            for (int n=0;n<24;++n) { const float mid=(lo+hi)*0.5f;
                if (body->blocksSegment(previous[i],glm::mix(previous[i],gas.positions[i],mid))) hi=mid; else lo=mid; }
            gas.positions[i]=glm::mix(previous[i],gas.positions[i],lo);
            const float d=body->signedDistance(gas.positions[i],&normal);
            gas.positions[i]+=normal*std::max(0.0f,gas.radii[i]-d);
        } else if (distance < gas.radii[i]) gas.positions[i]+=normal*(gas.radii[i]-distance);
        else continue;
        const float inward = glm::dot(gas.velocities[i],normal);
        if (inward < 0) gas.velocities[i]-=normal*inward;
    }
}
