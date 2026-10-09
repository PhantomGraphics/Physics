#include "HairGenerator.h"
#include <cmath>
#include <algorithm>

namespace Phantom::Physics {
namespace {
// Integer hashing makes each sample independent of previous generator calls.
float sample(uint32_t seed, uint32_t strand, uint32_t channel) {
    uint32_t x = seed ^ (strand*0x9e3779b9u) ^ (channel*0x85ebca6bu);
    x ^= x >> 16; x *= 0x7feb352du;
    x ^= x >> 15; x *= 0x846ca68bu; x ^= x >> 16;
    return static_cast<float>(x >> 8)*(2.f/16777216.f)-1.f;
}
float strandLength(float length, const HairVariationParams& v, int strand) {
    return length*(1.f+v.lengthVariation*sample(v.seed,strand,1));
}
void varyShape(HairStrandInput& in, float length, const HairVariationParams& v, int strand) {
    const Math::Vector3df offset(v.shapeVariation*length*sample(v.seed,strand,2),0.f,
                               v.shapeVariation*length*sample(v.seed,strand,3));
    for (size_t j = 1; j < in.restPositions.size(); ++j) {
        const float t = static_cast<float>(j)/(in.restPositions.size()-1);
        in.restPositions[j] += offset*t*t;
    }
}
}
bool validateHairVariation(const HairVariationParams& v) {
    return std::isfinite(v.rootJitter) && v.rootJitter >= 0.f && v.rootJitter <= 1.f &&
           std::isfinite(v.shapeVariation) && v.shapeVariation >= 0.f && v.shapeVariation <= 1.f &&
           std::isfinite(v.lengthVariation) && v.lengthVariation >= 0.f && v.lengthVariation <= 0.9f;
}
bool generateHairBundle(HairStrands& output, const HairGeneratorParams& p) {
    if (!validateHairVariation(p.variation) || p.strands < 1 || p.strands > 10000 || p.particlesPerStrand < 2 ||
        p.particlesPerStrand > 4096 || p.strands > 1000000 / p.particlesPerStrand ||
        !std::isfinite(p.length) || p.length < 1.e-4f || p.length > 100.f ||
        !std::isfinite(p.spacing) || p.spacing < 0.f || p.spacing > 10.f ||
        !std::isfinite(p.curvature) || std::abs(p.curvature) > 10.f) return false;
    const int columns = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(p.strands))));
    const int rows = (p.strands + columns - 1) / columns;
    const float norm = glm::dot(p.root.rotation, p.root.rotation);
    if (!std::isfinite(norm) || norm < 1.e-12f) return false;
    const auto rotation = glm::normalize(p.root.rotation);
    std::vector<HairStrandInput> inputs;
    inputs.reserve(p.strands);
    for (int s = 0; s < p.strands; ++s) {
        HairStrandInput in;
        in.root = p.root;
        in.root.position += rotation * Math::Vector3df(
            (s % columns - (columns - 1) * 0.5f + 0.5f*p.variation.rootJitter*sample(p.variation.seed,s,4)) * p.spacing, 0.f,
            (s / columns - (rows - 1) * 0.5f + 0.5f*p.variation.rootJitter*sample(p.variation.seed,s,5)) * p.spacing);
        for (int j = 0; j < p.particlesPerStrand; ++j) {
            const float t = static_cast<float>(j) / (p.particlesPerStrand - 1);
            in.restPositions.push_back({p.curvature * p.length * t * t, -p.length * t, 0.f});
        }
        varyShape(in,p.length,p.variation,s);
        // length is the discrete arc length, rather than the vertical extent.
        float arcLength = 0.f;
        for (size_t j = 1; j < in.restPositions.size(); ++j)
            arcLength += glm::length(in.restPositions[j] - in.restPositions[j-1]);
        for (auto& point : in.restPositions) point *= strandLength(p.length,p.variation,s) / arcLength;
        inputs.push_back(std::move(in));
    }
    return output.initialize(inputs);
}
namespace {
constexpr float pi = 3.14159265359f;
constexpr float goldenAngle = 2.39996322973f;

Math::Quaternion normalRotation(const Math::Vector3df& normal) {
    // Rotate local +Y onto the normal, including both poles.
    const Math::Vector3df axis(normal.z,0.f,-normal.x);
    const float sine = glm::length(axis);
    if (sine < 1.e-7f)
        return normal.y < 0.f ? Math::Quaternion(0.f,1.f,0.f,0.f) : Math::Quaternion(1.f,0.f,0.f,0.f);
    return glm::angleAxis(std::atan2(sine,normal.y),axis/sine);
}
}

bool generateHairSurface(HairStrands& output, const HairSurfaceParams& p) {
    if (!validateHairVariation(p.variation) || (p.surface != HairSurface::Scalp && p.surface != HairSurface::Capsule) ||
        p.strands < 1 || p.strands > 10000 || p.particlesPerStrand < 2 ||
        p.particlesPerStrand > 4096 || p.strands > 1000000 / p.particlesPerStrand ||
        !std::isfinite(p.length) || p.length < 1.e-4f || p.length > 100.f ||
        !std::isfinite(p.radius) || p.radius < 1.e-4f || p.radius > 100.f ||
        !std::isfinite(p.capsuleHalfLength) || p.capsuleHalfLength < 0.f || p.capsuleHalfLength > 100.f ||
        !std::isfinite(p.scalpMinAngle) || !std::isfinite(p.scalpMaxAngle) ||
        p.scalpMinAngle < 0.f || p.scalpMaxAngle > pi*0.5f || p.scalpMinAngle >= p.scalpMaxAngle ||
        !std::isfinite(p.pose.position.x) || !std::isfinite(p.pose.position.y) || !std::isfinite(p.pose.position.z))
        return false;
    const float norm = glm::dot(p.pose.rotation, p.pose.rotation);
    if (!std::isfinite(norm) || norm < 1.e-12f) return false;
    const auto rotation = glm::normalize(p.pose.rotation);
    std::vector<HairStrandInput> inputs;
    inputs.reserve(p.strands);
    for (int s = 0; s < p.strands; ++s) {
        // Jitter stays inside each area stratum and never leaves the surface.
        const float u = (s+0.5f+0.49f*p.variation.rootJitter*sample(p.variation.seed,s,4))/p.strands;
        const float phi = s*goldenAngle+pi*p.variation.rootJitter*sample(p.variation.seed,s,5);
        const float c = std::cos(phi), si = std::sin(phi);
        Math::Vector3df normal, root;
        float theta = 0.f;
        if (p.surface == HairSurface::Scalp) {
            const float y = std::cos(p.scalpMinAngle)*(1.f-u)+std::cos(p.scalpMaxAngle)*u;
            theta = std::acos(std::clamp(y,-1.f,1.f));
            const float radial = std::sqrt(std::max(0.f,1.f-y*y));
            normal = {radial*c, y, radial*si};
            root = p.radius*normal;
        } else {
            // Cylinder area = 4*pi*r*h, total cap area = 4*pi*r*r.
            // Sweep area from the negative cap, along the cylinder, to the positive cap.
            const float areaCoordinate = u*(2.f*p.radius+2.f*p.capsuleHalfLength);
            float z;
            if (areaCoordinate < p.radius) {
                z = areaCoordinate/p.radius-1.f;
                const float radial = std::sqrt(std::max(0.f,1.f-z*z));
                normal = {radial*c, radial*si, z};
                root = p.radius*normal+Math::Vector3df(0.f,0.f,-p.capsuleHalfLength);
            } else if (areaCoordinate <= p.radius+2.f*p.capsuleHalfLength) {
                normal = {c,si,0.f};
                root = p.radius*normal+Math::Vector3df(0.f,0.f,areaCoordinate-p.radius-p.capsuleHalfLength);
            } else {
                z = (areaCoordinate-p.radius-2.f*p.capsuleHalfLength)/p.radius;
                const float radial = std::sqrt(std::max(0.f,1.f-z*z));
                normal = {radial*c,radial*si,z};
                root = p.radius*normal+Math::Vector3df(0.f,0.f,p.capsuleHalfLength);
            }
        }
        HairStrandInput in;
        const auto frame = normalRotation(normal);
        in.root.position = p.pose.position+rotation*root;
        in.root.rotation = rotation*frame;
        const float targetLength = strandLength(p.length,p.variation,s);
        const float arc = std::min(targetLength, p.radius*(pi*0.5f-theta));
        for (int j = 0; j < p.particlesPerStrand; ++j) {
            const float distance = targetLength*j/(p.particlesPerStrand-1);
            Math::Vector3df point;
            if (p.surface == HairSurface::Scalp) {
                const float angle = theta+std::min(distance,arc)/p.radius;
                point = p.radius*Math::Vector3df(std::sin(angle)*c,std::cos(angle),std::sin(angle)*si);
                point.y -= std::max(0.f,distance-arc);
                in.restPositions.push_back(j == 0 ? Math::Vector3df(0.f) : glm::conjugate(frame)*(point-root));
            } else in.restPositions.push_back({0.f,distance,0.f});
        }
        varyShape(in,targetLength,p.variation,s);
        float length = 0.f;
        for (size_t j = 1; j < in.restPositions.size(); ++j)
            length += glm::length(in.restPositions[j]-in.restPositions[j-1]);
        if (!std::isfinite(length) || length < 1.e-6f) return false;
        for (auto& point : in.restPositions) point *= targetLength/length;
        inputs.push_back(std::move(in));
    }
    return output.initialize(inputs);
}
} // namespace Phantom::Physics
