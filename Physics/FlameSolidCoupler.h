#pragma once
#include "SolidCombustionSolver.h"

namespace Phantom::Physics {
class FlameFluid;
// Non-owning registrations. World clears these before deleting bodies.
class FlameSolidCoupler {
public:
    bool add(CombustibleBody* body);
    void remove(uint64_t id);
    void clear() { bodies_.clear(); }
    const std::vector<CombustibleBody*>& bodies() const { return bodies_; }
    double initialFuelMass = 0.0, removedMass = 0.0;
    float exchangeDistance = 0.16f;
    float heatTransfer = 0.04f;
    float gasSpecificHeat = 1.0f;
    float emissionSpeed = 0.15f;
    SolidCombustionSolver solidSolver;
    void update(FlameFluid& fluid, float dt, double time);
    void constrain(FlameFluid& fluid, const std::vector<Math::Vector3df>& previous) const;
    bool occluded(const Math::Vector3df& a, const Math::Vector3df& b) const;
private:
    std::vector<CombustibleBody*> bodies_;
};
}
