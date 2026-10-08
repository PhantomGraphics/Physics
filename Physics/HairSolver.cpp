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
        !finite(p.gravity) || glm::length(p.gravity) > 10000.f ||
        !finite(p.windVelocity) || glm::length(p.windVelocity) > 1000.f ||
        !std::isfinite(p.windDrag) || p.windDrag < 0.f || p.windDrag > 100.f ||
        !std::isfinite(p.collisionRadius) || p.collisionRadius < 0.f || p.collisionRadius > 1.f ||
        !std::isfinite(p.teleportDistance) || p.teleportDistance <= 0.f || p.teleportDistance > 100.f ||
        !std::isfinite(p.teleportAngle) || p.teleportAngle <= 0.f || p.teleportAngle > 3.141593f) return false;
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
    previousRoots_.clear();
    contacts_.clear();
    stats_ = {};
    if (!strands) return true;
    targets_.resize(strands->particleCount());
    shapeLambda_.resize(strands->particleCount(), Math::Vector3df(0.f));
    contacts_.resize(strands->particleCount());
    for (const auto& range : strands->ranges()) {
        previousRoots_.push_back(range.root);
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

void HairSolver::updateTargets(float fraction) {
    for (size_t s = 0; s < strands_->strandCount(); ++s) {
        const auto& r = strands_->ranges_[s];
        const auto& previous = previousRoots_[s];
        const auto position = glm::mix(previous.position, r.root.position, fraction);
        const auto rotation = glm::slerp(previous.rotation, r.root.rotation, fraction);
        for (size_t j = 0; j < r.count; ++j) {
            const size_t i = r.offset+j;
            targets_[i] = position + rotation * strands_->restPositions_[i];
        }
    }
}

void HairSolver::resetStrand(size_t strand) {
    const auto& r = strands_->ranges_[strand];
    auto& p = strands_->particles_;
    for (size_t j = 0; j < r.count; ++j) {
        const size_t i = r.offset+j;
        targets_[i] = r.root.position + r.root.rotation * strands_->restPositions_[i];
        p.positions[i] = p.predicted[i] = targets_[i];
        p.velocities[i] = p.forces[i] = shapeLambda_[i] = Math::Vector3df(0.f);
        contacts_[i] = {};
    }
    previousRoots_[strand] = r.root;
    for (auto& c : stretch_) c.resetLambda();
    for (auto& c : bend_) c.resetLambda();
}

bool HairSolver::setRootPose(size_t strand, const HairRootPose& pose, bool teleport) {
    if (!strands_ || revision_ != strands_->revision() || strand >= strands_->strandCount() ||
        !finite(pose.position) || glm::length(pose.position) > 10000.f) return false;
    const float norm = glm::dot(pose.rotation, pose.rotation);
    if (!std::isfinite(norm) || norm < 1.e-12f) return false;
    HairRootPose next = pose;
    next.rotation = glm::normalize(pose.rotation);
    const auto& r = strands_->ranges_[strand];
    Math::Vector3df previousPoint = next.position;
    for (size_t j = 1; j < r.count; ++j) {
        const auto point = next.position + next.rotation * strands_->restPositions_[r.offset+j];
        if (!finite(point) || glm::length(point-previousPoint) < 1.e-6f) return false;
        previousPoint = point;
    }
    const auto& old = previousRoots_[strand];
    const float angle = 2.f*std::acos(std::clamp(std::abs(glm::dot(old.rotation, next.rotation)), 0.f, 1.f));
    const bool jumped = teleport || glm::length(next.position-old.position) > params_.teleportDistance || angle > params_.teleportAngle;
    strands_->ranges_[strand].root = next;
    if (jumped) {
        resetStrand(strand);
        ++stats_.rootResets;
        measure();
    }
    return true;
}

bool HairSolver::setColliders(const std::vector<HairCollider>& colliders, bool teleport) {
    if (colliders.size() > 64) return false;
    for (const auto& c : colliders) if (!validHairCollider(c)) return false;
    if (teleport || colliders.size() != colliders_.size()) previousColliders_ = colliders;
    colliders_ = colliders;
    measure();
    return true;
}

void HairSolver::resolveCollisions(float dt) {
    auto& p = strands_->particles_;
    auto record = [&](size_t i, const HairContact& hit, size_t c, float correction) {
        const auto& now = stepColliders_[c];
        const auto& before = beforeColliders_[c];
        auto& contact = contacts_[i];
        contact.active = true;
        contact.normal = hit.normal;
        contact.velocity = (glm::mix(now.a, now.b, hit.colliderFraction) -
                            glm::mix(before.a, before.b, hit.colliderFraction)) / dt;
        contact.correction += correction;
        contact.friction = now.friction;
    };
    for (size_t c = 0; c < stepColliders_.size(); ++c) {
        const auto& collider = stepColliders_[c];
        for (size_t i = 0; i < p.size(); ++i) {
            if (p.isPinned(i)) continue;
            const auto hit = sampleHairContact(p.predicted[i], p.predicted[i], collider, params_.collisionRadius);
            if (hit.penetration > 0.f) {
                p.predicted[i] += hit.penetration*hit.normal;
                record(i, hit, c, hit.penetration);
            }
        }
        for (const auto& edge : stretch_) {
            const auto hit = sampleHairContact(p.predicted[edge.a], p.predicted[edge.b], collider, params_.collisionRadius);
            if (hit.penetration <= 1.e-7f) continue;
            const float a = 1.f-hit.strandFraction, b = hit.strandFraction;
            const float wa = p.inverseMasses[edge.a], wb = p.inverseMasses[edge.b];
            const float weight = wa*a*a + wb*b*b;
            if (weight < 1.e-8f) continue; // an embedded fixed root cannot be projected
            const float da = wa*a*hit.penetration/weight;
            const float db = wb*b*hit.penetration/weight;
            p.predicted[edge.a] += da*hit.normal;
            p.predicted[edge.b] += db*hit.normal;
            if (wa > 0.f) record(edge.a, hit, c, da);
            if (wb > 0.f) record(edge.b, hit, c, db);
        }
    }
}

void HairSolver::reset() {
    if (!strands_ || revision_ != strands_->revision()) return;
    updateTargets(1.f);
    for (size_t s = 0; s < strands_->strandCount(); ++s) previousRoots_[s] = strands_->ranges_[s].root;
    previousColliders_ = colliders_;
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
    const float windBlend = 1.f-std::exp(-params_.windDrag * dt);
    stepColliders_.resize(colliders_.size());
    beforeColliders_.resize(colliders_.size());
    for (int s = 0; s < params_.numSubsteps; ++s) {
        updateTargets(static_cast<float>(s+1)/params_.numSubsteps);
        std::fill(contacts_.begin(), contacts_.end(), ContactVelocity{});
        for (size_t c = 0; c < colliders_.size(); ++c) {
            stepColliders_[c] = colliders_[c];
            beforeColliders_[c] = colliders_[c];
            const float now = static_cast<float>(s+1)/params_.numSubsteps;
            const float before = static_cast<float>(s)/params_.numSubsteps;
            stepColliders_[c].a = glm::mix(previousColliders_[c].a, colliders_[c].a, now);
            stepColliders_[c].b = glm::mix(previousColliders_[c].b, colliders_[c].b, now);
            beforeColliders_[c].a = glm::mix(previousColliders_[c].a, colliders_[c].a, before);
            beforeColliders_[c].b = glm::mix(previousColliders_[c].b, colliders_[c].b, before);
        }
        for (size_t i = 0; i < p.size(); ++i) {
            if (p.isPinned(i)) {
                p.predicted[i] = targets_[i];
            } else {
                p.velocities[i] += dt * params_.gravity;
                p.velocities[i] += windBlend * (params_.windVelocity-p.velocities[i]);
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
            resolveCollisions(dt);
        }
        // End on collision projection; length error after collision is measured.
        resolveCollisions(dt);
        for (size_t i = 0; i < p.size(); ++i) {
            p.velocities[i] = (p.predicted[i]-p.positions[i])/dt;
            if (!p.isPinned(i)) {
                p.velocities[i] *= decay;
                const auto& contact = contacts_[i];
                if (contact.active) p.velocities[i] = hairContactVelocity(p.velocities[i], contact.normal,
                    contact.velocity, contact.correction, contact.friction, dt);
            }
            p.positions[i] = p.predicted[i];
        }
    }
    for (size_t s = 0; s < strands_->strandCount(); ++s) previousRoots_[s] = strands_->ranges_[s].root;
    previousColliders_ = colliders_;
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
    stats_.maxPenetration = 0.f;
    stats_.pinnedPenetration = 0.f;
    if (!strands_ || revision_ != strands_->revision()) return;
    const auto& p = strands_->particles_;
    for (size_t i = 0; i < p.size(); ++i) {
        if (!finite(p.positions[i]) || !finite(p.predicted[i]) || !finite(p.velocities[i])) ++stats_.nonFiniteCount;
        else stats_.maxSpeed = std::max(stats_.maxSpeed, glm::length(p.velocities[i]));
    }
    for (const auto& c : stretch_) {
        const float error = std::abs(glm::length(p.positions[c.a] - p.positions[c.b]) - c.restLength) / c.restLength;
        if (std::isfinite(error)) stats_.maxRelativeLengthError = std::max(stats_.maxRelativeLengthError, error);
    }
    for (const auto& collider : colliders_) {
        for (const auto& edge : stretch_) {
            const auto hit = sampleHairContact(p.positions[edge.a], p.positions[edge.b], collider, params_.collisionRadius);
            stats_.maxPenetration = std::max(stats_.maxPenetration, hit.penetration);
        }
        for (const auto& r : strands_->ranges_)
            stats_.pinnedPenetration = std::max(stats_.pinnedPenetration,
                sampleHairContact(p.positions[r.offset], p.positions[r.offset], collider, params_.collisionRadius).penetration);
    }
}
} // namespace Phantom::Physics
