#pragma once
#include "CombustibleBody.h"

namespace Phantom::Physics {
class SolidCombustionSolver {
public:
    float ambientTemperature = 300.0f;
    float cooling = 0.03f;
    float pendingFractionLimit = 0.1f;
    void update(CombustibleBody& body, float dt) const;
    // Exact two-capacity relaxation; conserves energy and cannot overshoot.
    static double exchange(float& a, double ca, float& b, double cb, double conductance, float dt);
};
}
