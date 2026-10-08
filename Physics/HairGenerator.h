#pragma once
#include "HairStrands.h"

namespace Phantom::Physics {

struct HairGeneratorParams {
    int strands = 48;
    int particlesPerStrand = 24;
    float length = 1.2f;
    float spacing = 0.08f;
    float curvature = 0.3f;
    HairRootPose root{{0.f, 1.5f, 0.f}, {1.f, 0.f, 0.f, 0.f}};
};

// Deterministic synthetic guides. No external assets or random state required.
bool generateHairBundle(HairStrands& output, const HairGeneratorParams& params);

} // namespace Phantom::Physics
