#pragma once

#include "../../CGLib/Math/Vector3d.h"
#include "RandomSeed.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

namespace Phantom {
namespace Physics {

class WhiteWaterSystem {
public:
    struct SprayParticle {
        Math::Vector3df pos{ 0.0f, 0.0f, 0.0f };
        Math::Vector3df vel{ 0.0f, 0.0f, 0.0f };
        float life = 0.0f;
        float lifeMax = 0.0f;
    };

    struct FoamParticle {
        Math::Vector3df pos{ 0.0f, 0.0f, 0.0f };
        Math::Vector3df vel{ 0.0f, 0.0f, 0.0f };
        float life = 0.0f;
        float lifeMax = 0.0f;
    };

    struct Params {
        bool enableSpray = true;
        bool enableFoam = false;
        float sprayVelThreshold = 3.0f;
        float sprayDensityRatio = 0.6f;
        float foamCurvThreshold = 0.5f;
        float foamNeighborCount = 20.0f;
        int maxSprayParticles = 50000;
        int maxFoamParticles = 30000;
        float gravity = -9.8f;
        float foamBuoyancy = 2.0f;
        float foamDrag = 0.95f;
        float sprayLifeMin = 0.5f;
        float sprayLifeMax = 2.0f;
        float foamLifeMin = 1.0f;
        float foamLifeMax = 5.0f;
    };

    WhiteWaterSystem()
        : rng_(kDefaultRandomSeed)
    {
    }

    void clear()
    {
        spray_.clear();
        foam_.clear();
        estimatedSurfaceY_ = 0.0f;
    }

    /**
     * @brief Reseeds the RNG the spray/foam lifetimes are drawn from.
     * @param seed New seed. The default (kDefaultRandomSeed) makes a scene
     *        reproducible run to run; pass std::random_device{}() to get a
     *        different draw each run. See RandomSeed.h.
     */
    void setRandomSeed(const unsigned int seed) { rng_.seed(seed); }

    void generate(const std::vector<Math::Vector3df>& pos,
                  const std::vector<Math::Vector3df>& vel,
                  const std::vector<float>& density,
                  float restDensity,
                  float)
    {
        if (!params.enableSpray) {
            spray_.clear();
        }
        if (!params.enableFoam) {
            foam_.clear();
        }

        const size_t n = std::min({ pos.size(), vel.size(), density.size() });
        if (n == 0 || restDensity <= 1.0e-6f) {
            return;
        }

        Math::Vector3df center(0.0f, 0.0f, 0.0f);
        float sumY = 0.0f;
        for (size_t i = 0; i < n; ++i) {
            center += pos[i];
            sumY += pos[i].y;
        }
        center /= static_cast<float>(n);
        estimatedSurfaceY_ = sumY / static_cast<float>(n);

        for (size_t i = 0; i < n; ++i) {
            const float speed = Math::getLength(vel[i]);
            const float densityRatio = density[i] / restDensity;
            const Math::Vector3df normal = safeNormalize(pos[i] - center);

            const bool sprayA = speed > params.sprayVelThreshold;
            const bool sprayB = densityRatio < params.sprayDensityRatio;
            const bool sprayC = glm::dot(vel[i], normal) > 0.0f;

            if (params.enableSpray && sprayA && sprayB && sprayC) {
                const int emitCount = static_cast<int>(randomRange(1.0f, 4.999f));
                for (int e = 0; e < emitCount; ++e) {
                    const Math::Vector3df jitter(
                        randomRange(-0.1f, 0.1f),
                        randomRange(-0.1f, 0.1f),
                        randomRange(-0.1f, 0.1f));
                    emitSpray(pos[i] + jitter, vel[i] + 0.5f * jitter);
                }
            }

            const float curvatureMetric = std::max(0.0f, 1.0f - densityRatio) + speed * 0.05f;
            const float neighborMetric = densityRatio * 30.0f;
            const float convergence = -glm::dot(vel[i], normal);

            const bool foamA = curvatureMetric > params.foamCurvThreshold;
            const bool foamB = neighborMetric > params.foamNeighborCount;
            const bool foamC = convergence > 0.0f;

            if (params.enableFoam && foamA && foamB && foamC) {
                const Math::Vector3df jitter(
                    randomRange(-0.05f, 0.05f),
                    randomRange(0.0f, 0.08f),
                    randomRange(-0.05f, 0.05f));
                emitFoam(pos[i] + jitter, vel[i] * 0.2f);
            }
        }
    }

    void advance(float dt)
    {
        if (params.enableSpray) {
            for (auto& s : spray_) {
                s.vel.y += params.gravity * dt;
                s.pos += s.vel * dt;
                s.life -= dt;
            }
        }
        else {
            spray_.clear();
        }

        if (params.enableFoam) {
            for (auto& f : foam_) {
                f.vel.y += params.foamBuoyancy * dt;
                f.vel *= params.foamDrag;
                f.pos += f.vel * dt;
                f.life -= dt;
            }
        }
        else {
            foam_.clear();
        }

        if (params.enableSpray && params.enableFoam) {
            transitionSprayToFoam();
        }
        removeDead();
    }

    const std::vector<SprayParticle>& getSpray() const { return spray_; }
    const std::vector<FoamParticle>& getFoam() const { return foam_; }

    Params params;

private:
    static Math::Vector3df safeNormalize(const Math::Vector3df& v)
    {
        const float len2 = Math::getLengthSquared(v);
        if (len2 <= 1.0e-8f) {
            return Math::Vector3df(0.0f, 1.0f, 0.0f);
        }
        return v / std::sqrt(len2);
    }

    void emitSpray(const Math::Vector3df& pos, const Math::Vector3df& vel)
    {
        if (static_cast<int>(spray_.size()) >= params.maxSprayParticles) {
            return;
        }

        SprayParticle p;
        p.pos = pos;
        p.vel = vel;
        p.lifeMax = randomRange(params.sprayLifeMin, params.sprayLifeMax);
        p.life = p.lifeMax;
        spray_.push_back(p);
    }

    void emitFoam(const Math::Vector3df& pos, const Math::Vector3df& vel)
    {
        if (static_cast<int>(foam_.size()) >= params.maxFoamParticles) {
            return;
        }

        FoamParticle p;
        p.pos = pos;
        p.vel = vel;
        p.lifeMax = randomRange(params.foamLifeMin, params.foamLifeMax);
        p.life = p.lifeMax;
        foam_.push_back(p);
    }

    void transitionSprayToFoam()
    {
        for (size_t i = 0; i < spray_.size();) {
            auto& s = spray_[i];
            const bool hitSurface = s.vel.y < 0.0f && s.pos.y <= estimatedSurfaceY_;

            if (hitSurface && static_cast<int>(foam_.size()) < params.maxFoamParticles) {
                emitFoam(s.pos, s.vel * 0.15f);
                spray_[i] = spray_.back();
                spray_.pop_back();
                continue;
            }

            ++i;
        }
    }

    void removeDead()
    {
        spray_.erase(
            std::remove_if(spray_.begin(), spray_.end(),
                           [](const SprayParticle& p) { return p.life <= 0.0f; }),
            spray_.end());

        foam_.erase(
            std::remove_if(foam_.begin(), foam_.end(),
                           [](const FoamParticle& p) { return p.life <= 0.0f; }),
            foam_.end());
    }

    float randomRange(float minValue, float maxValue)
    {
        std::uniform_real_distribution<float> dist(minValue, maxValue);
        return dist(rng_);
    }

private:
    std::vector<SprayParticle> spray_;
    std::vector<FoamParticle> foam_;
    std::mt19937 rng_;
    float estimatedSurfaceY_ = 0.0f;
};

} // namespace Physics
} // namespace Phantom
