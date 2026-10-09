#pragma once
#include "HairStrands.h"
#include <cstdint>

namespace Phantom::Physics {

struct HairVariationParams {
    float rootJitter = 0.f; // 0..1, fraction of root sampling cell
    float shapeVariation = 0.f; // 0..1, lateral tip offset / nominal length
    float lengthVariation = 0.f; // 0..0.9, symmetric relative arc-length range
    uint32_t seed = 0;
};
bool validateHairVariation(const HairVariationParams& params);

struct HairGeneratorParams {
    int strands = 48;
    int particlesPerStrand = 24;
    float length = 1.2f;
    float spacing = 0.08f;
    float curvature = 0.3f;
    HairVariationParams variation;
    HairRootPose root{{0.f, 1.5f, 0.f}, {1.f, 0.f, 0.f, 0.f}};
};

// Deterministic synthetic guides. No external assets or random state required.
bool generateHairBundle(HairStrands& output, const HairGeneratorParams& params);

enum class HairSurface { Scalp, Capsule };

struct HairSurfaceParams {
    HairSurface surface = HairSurface::Scalp;
    int strands = 48;
    int particlesPerStrand = 24;
    float length = 1.2f; // discrete strand arc length, metres
    float radius = 0.32f;
    float capsuleHalfLength = 0.4f; // segment along local Z; zero gives a sphere
    float scalpMinAngle = 0.35f; // polar angle from local +Y, radians
    float scalpMaxAngle = 1.3f;
    HairVariationParams variation;
    HairRootPose pose{{0.f, 1.05f, 0.f}, {1.f, 0.f, 0.f, 0.f}};
};

// Deterministic area-uniform roots on a spherical scalp band or capsule surface.
// Capsule guides grow along the outward normal; scalp guides follow a meridian
// toward the equator, then hang down. Invalid input preserves output.
bool generateHairSurface(HairStrands& output, const HairSurfaceParams& params);

} // namespace Phantom::Physics
