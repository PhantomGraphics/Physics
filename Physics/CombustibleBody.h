#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Quaternion.h"
#include <cstdint>
#include <vector>

namespace Phantom::Physics {

// Visual simulation units, not measured material data. Mass uses the same
// density * (2 radius)^3 convention as FlameParticle. Energy = mass * cp * K.
struct CombustibleMaterial {
    float density = 1.0f;
    float specificHeat = 1.0f;
    float conductivity = 0.02f;
    float pyrolysisTemperature = 650.0f;
    float pyrolysisRate = 0.8f; // multiplier of 0.05 mass/(area*s), independent of fuel stock
    float latentHeat = 100.0f;
    float residueFraction = 0.15f;
    bool combustible = true;
    bool valid() const;
    static CombustibleMaterial preset(int index);
};

enum class CombustionState { Unburned, Heating, Burning, Extinguished, Exhausted };
const char* combustionStateName(CombustionState state);

struct CombustibleParticle {
    Math::Vector3df position{0.0f}, normal{0.0f}; // local simulation coordinates
    float area = 0.0f, volume = 0.0f;
    float temperature = 300.0f;
    float radius = 0.0f; // positive for volumetric solid SPH particles
    Math::Vector3df glyphHalfExtent{0.0f}; // cell dimensions, including thin plates
    double initialFuel = 0.0, fuel = 0.0, residue = 0.0, pending = 0.0;
    double pendingHeat = 0.0; // sensible energy above ambient held by pending vapor
    std::vector<size_t> neighbors;
};

// Compatibility name for the original surface-cell model and its API clients.
using CombustibleSample = CombustibleParticle;

struct CombustibleBodyStats {
    double initialFuel = 0.0, fuel = 0.0, residue = 0.0, pending = 0.0;
    double pyrolyzed = 0.0, emitted = 0.0, heatExchange = 0.0;
    float temperature = 300.0f, reactionRate = 0.0f;
    double firstIgnitionTime = -1.0;
    CombustionState state = CombustionState::Unburned;
    bool get(const char* name, double& value) const;
};

class CombustibleBody {
public:
    enum class Shape { Box, Sphere };
    bool initialize(uint64_t id, Shape shape, const Math::Vector3df& center,
                    const Math::Vector3df& halfExtent, int resolution,
                    double fuelMass, const CombustibleMaterial& material);
    bool initializeParticles(uint64_t id, Shape shape, const Math::Vector3df& center,
                    const Math::Vector3df& halfExtent, int resolution,
                    double fuelMass, const CombustibleMaterial& material);
    bool usesSolidParticles() const { return smoothingLength_ > 0; }
    float smoothingLength() const { return smoothingLength_; }
    int resolution() const { return resolution_; }
    uint64_t id() const { return id_; }
    Shape shape() const { return shape_; }
    const Math::Vector3df& center() const { return center_; }
    const Math::Vector3df& halfExtent() const { return halfExtent_; }
    bool setCenter(const Math::Vector3df& center);
    bool setMotion(const Math::Vector3df& center, const Math::Quaternion& orientation,
                   const Math::Vector3df& linearVelocity, const Math::Vector3df& angularVelocity);
    const Math::Quaternion& orientation() const { return orientation_; }
    Math::Vector3df worldPosition(const Math::Vector3df& local) const { return center_ + orientation_ * local; }
    Math::Vector3df surfaceNormal(const CombustibleSample& sample) const { return orientation_ * sample.normal; }
    Math::Vector3df velocityAt(const Math::Vector3df& point) const { return linearVelocity_ + glm::cross(angularVelocity_, point-center_); }
    void beginMotionStep() { previousCenter_=center_; previousOrientation_=orientation_; }
    Math::Vector3df sweepStart(const Math::Vector3df& point) const {
        return worldPosition(glm::conjugate(previousOrientation_) * (point-previousCenter_));
    }
    bool setMaterial(const CombustibleMaterial& material);
    const CombustibleMaterial& material() const { return material_; }
    std::vector<CombustibleSample>& samples() { return samples_; }
    const std::vector<CombustibleSample>& samples() const { return samples_; }
    std::vector<CombustibleParticle>& particles() { return samples_; }
    const std::vector<CombustibleParticle>& particles() const { return samples_; }
    Math::Vector3df surfacePosition(const CombustibleSample& sample) const;
    float signedDistance(const Math::Vector3df& point, Math::Vector3df* normal = nullptr) const;
    bool blocksSegment(const Math::Vector3df& a, const Math::Vector3df& b) const;
    CombustibleBodyStats stats() const;
    double heatCapacity(const CombustibleSample& sample) const;

    double pyrolyzed = 0.0, emitted = 0.0, heatExchange = 0.0;
    float reactionRate = 0.0f;
    double firstIgnitionTime = -1.0;
    bool wasBurning = false;
private:
    uint64_t id_ = 0;
    Shape shape_ = Shape::Box;
    Math::Vector3df center_{0.0f}, halfExtent_{1.0f};
    Math::Quaternion orientation_{1,0,0,0}, previousOrientation_{1,0,0,0};
    Math::Vector3df linearVelocity_{0}, angularVelocity_{0}, previousCenter_{0};
    CombustibleMaterial material_;
    std::vector<CombustibleSample> samples_;
    float smoothingLength_ = 0.0f;
    int resolution_ = 2;
};

// Independent of RenderParams. Bodies are stored in simulation space; commands
// accepting scene coordinates pass through this transform once at creation.
struct FlamePhysicalTransform {
    float scale = 12.0f;
    Math::Vector3df offset{0.0f};
    Math::Vector3df toScene(const Math::Vector3df& p) const { return p * scale + offset; }
    Math::Vector3df toSimulation(const Math::Vector3df& p) const { return (p - offset) / scale; }
    Math::Vector3df velocityToScene(const Math::Vector3df& v) const { return v * scale; }
    Math::Vector3df velocityToSimulation(const Math::Vector3df& v) const { return v / scale; }
    float lengthToSimulation(float v) const { return v / scale; }
    float areaToScene(float v) const { return v * scale * scale; }
    float areaToSimulation(float v) const { return v / (scale * scale); }
    float volumeToScene(float v) const { return v * scale * scale * scale; }
    float volumeToSimulation(float v) const { return v / (scale * scale * scale); }
    Math::Vector3df normalToScene(const Math::Vector3df& v) const { return v; }
};
}
