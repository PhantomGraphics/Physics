#include "HairSolver.h"
#include <algorithm>
#include <cmath>

namespace Phantom::Physics {
namespace {
bool finite(const Math::Vector3df& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool compliance(float v) { return std::isfinite(v) && v >= 0.f && v <= 1.e6f; }
}

bool HairSolver::setParams(const Params& p) {
    if (!std::isfinite(p.timeStep) || p.timeStep < 1.e-4f || p.timeStep > 1.f/15.f ||
        p.numSubsteps < 1 || p.numSubsteps > 64 || p.numIterations < 1 || p.numIterations > 128 ||
        !compliance(p.stretchCompliance) || !compliance(p.bendCompliance) || !compliance(p.shapeCompliance) ||
        !std::isfinite(p.dampingRate) || p.dampingRate < 0.f || p.dampingRate > 100.f ||
        !finite(p.gravity) || glm::length(p.gravity) > 10000.f) return false;
    params_ = p;
    return true;
}

bool HairSolver::setStrands(HairStrands* strands) {
    strands_ = strands;
    revision_ = strands ? strands->revision() : 0;
    stretch_.clear();
    bend_.clear();
    targets_.clear();
    shapeLambda_.clear();
    stats_ = {};
    if (!strands) return true;
    targets_.resize(strands->particleCount());
    shapeLambda_.resize(strands->particleCount(), Math::Vector3df(0.f));
    for (const auto& range : strands->ranges()) {
        for (size_t j = 0; j < range.count; ++j) {
            const size_t i = range.offset + j;
            targets_[i] = range.root.position + range.root.rotation * strands->restPositions_[i];
            if (j) {
                DistanceConstraint c;
                c.a = static_cast<int>(i-1); c.b = static_cast<int>(i);
                c.restLength = glm::length(strands->restPositions_[i] - strands->restPositions_[i-1]);
                stretch_.push_back(c);
            }
            if (j >= 2) {
                DistanceConstraint c;
                c.a = static_cast<int>(i-2); c.b = static_cast<int>(i);
                c.restLength = glm::length(strands->restPositions_[i] - strands->restPositions_[i-2]);
                bend_.push_back(c);
            }
        }
    }
    measure();
    return true;
}

void HairSolver::reset() {
    if (!strands_ || revision_ != strands_->revision()) return;
    auto& p = strands_->particles_;
    p.positions = targets_;
    p.predicted = targets_;
    std::fill(p.velocities.begin(), p.velocities.end(), Math::Vector3df(0.f));
    std::fill(p.forces.begin(), p.forces.end(), Math::Vector3df(0.f));
    for (auto& c : stretch_) c.resetLambda();
    for (auto& c : bend_) c.resetLambda();
    std::fill(shapeLambda_.begin(), shapeLambda_.end(), Math::Vector3df(0.f));
    stats_ = {};
    measure();
}

bool HairSolver::step() {
    if (!strands_ || revision_ != strands_->revision() || strands_->particleCount() == 0) return false;
    auto& p = strands_->particles_;
    const float dt = params_.timeStep / params_.numSubsteps;
    const float stretchAlpha = params_.stretchCompliance / (dt*dt);
    const float bendAlpha = params_.bendCompliance / (dt*dt);
    const float shapeAlpha = params_.shapeCompliance / (dt*dt);
    const float decay = std::exp(-params_.dampingRate * dt);
    for (int s = 0; s < params_.numSubsteps; ++s) {
        for (size_t i = 0; i < p.size(); ++i) {
            if (p.isPinned(i)) {
                p.positions[i] = targets_[i];
                p.predicted[i] = targets_[i];
                p.velocities[i] = Math::Vector3df(0.f);
            } else {
                p.velocities[i] += dt * params_.gravity;
                p.predicted[i] = p.positions[i] + dt * p.velocities[i];
            }
        }
        for (auto& c : stretch_) c.resetLambda();
        for (auto& c : bend_) c.resetLambda();
        std::fill(shapeLambda_.begin(), shapeLambda_.end(), Math::Vector3df(0.f));
        for (int iter = 0; iter < params_.numIterations; ++iter) {
            if (params_.shapeEnabled) {
                for (size_t i = 0; i < p.size(); ++i) {
                    const float w = p.inverseMasses[i];
                    if (w == 0.f) continue;
                    const auto dl = -(p.predicted[i] - targets_[i] + shapeAlpha * shapeLambda_[i]) / (w + shapeAlpha);
                    shapeLambda_[i] += dl;
                    p.predicted[i] += w * dl;
                }
            }
            for (auto& c : bend_) c.project(p, bendAlpha);
            // Alternate sweep direction, finishing each iteration with stretch.
            if (iter % 2 == 0) {
                for (auto& c : stretch_) c.project(p, stretchAlpha);
            } else {
                for (auto c = stretch_.rbegin(); c != stretch_.rend(); ++c) c->project(p, stretchAlpha);
            }
        }
        for (size_t i = 0; i < p.size(); ++i) {
            if (!p.isPinned(i)) p.velocities[i] = (p.predicted[i] - p.positions[i]) * (decay / dt);
            p.positions[i] = p.predicted[i];
        }
    }
    stats_.simulatedTime += params_.timeStep;
    ++stats_.steps;
    measure();
    return stats_.nonFiniteCount == 0;
}

void HairSolver::measure() {
    stats_.particleCount = strands_ ? strands_->particleCount() : 0;
    stats_.nonFiniteCount = 0;
    stats_.maxSpeed = 0.f;
    stats_.maxRelativeLengthError = 0.f;
    if (!strands_) return;
    const auto& p = strands_->particles_;
    for (size_t i = 0; i < p.size(); ++i) {
        if (!finite(p.positions[i]) || !finite(p.predicted[i]) || !finite(p.velocities[i])) ++stats_.nonFiniteCount;
        else stats_.maxSpeed = std::max(stats_.maxSpeed, glm::length(p.velocities[i]));
    }
    for (const auto& c : stretch_) {
        const float error = std::abs(glm::length(p.positions[c.a] - p.positions[c.b]) - c.restLength) / c.restLength;
        if (std::isfinite(error)) stats_.maxRelativeLengthError = std::max(stats_.maxRelativeLengthError, error);
    }
}
} // namespace Phantom::Physics
