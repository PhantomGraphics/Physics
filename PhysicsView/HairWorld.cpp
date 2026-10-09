#include "HairWorld.h"
#include <cmath>
#include <algorithm>
#include <string>

namespace Phantom {
HairWorld::~HairWorld() { clear(); }
void HairWorld::removeComponent() {
    if (registry_ && componentId_) registry_->remove(componentId_);
    componentId_ = 0;
}
void HairWorld::setComponentRegistry(SceneComponentRegistry* registry) {
    removeComponent();
    registry_ = registry;
}
bool HairWorld::setPreset(HairPreset preset) {
    Physics::HairGeneratorParams p;
    auto settings = params();
    // Style presets include their solver settings, independent of previous edits.
    if (preset == HairPreset::LongHair || preset == HairPreset::ShortFur) {
        settings = Physics::HairSolver::Params{};
        if (preset == HairPreset::ShortFur) {
            p.strands = 96;
            p.particlesPerStrand = 4;
            p.length = 0.06f;
            settings.bendCompliance = 1.e-6f;
            settings.shapeCompliance = 1.e-5f;
            settings.dampingRate = 6.f;
            settings.collisionRadius = 0.001f;
        }
    }
    if (preset == HairPreset::Single) p.strands = 1;
    Physics::HairStrands next;
    std::vector<Physics::HairRootPose> followerRoots;
    const bool body = preset == HairPreset::Body || preset == HairPreset::HeadShake || preset == HairPreset::StrongWind;
    if (body) {
        std::vector<Physics::HairStrandInput> inputs;
        const Math::Vector3df center(0.f,1.05f,0.f);
        for (int s = 0; s < 16; ++s) {
            Physics::HairStrandInput in;
            const float phi = -1.5707963f+(s-7.5f)*0.08f;
            const float theta = 0.45f+0.08f*(s%3);
            for (int j = 0; j < 24; ++j) {
                const float t = j/23.f;
                const float angle = theta+0.9f*t;
                auto point = center + 0.32f*Math::Vector3df(std::sin(angle)*std::cos(phi), std::cos(angle), std::sin(angle)*std::sin(phi));
                point.y -= 0.95f*std::max(0.f,t-0.2f);
                if (j == 0) in.root.position = point;
                in.restPositions.push_back(point-in.root.position);
            }
            inputs.push_back(std::move(in));
        }
        if (!next.initialize(inputs)) return false;
    } else if (preset == HairPreset::LongHair || preset == HairPreset::ShortFur) {
        Physics::HairSurfaceParams surface;
        surface.strands = p.strands;
        surface.particlesPerStrand = p.particlesPerStrand;
        surface.length = p.length;
        if (preset == HairPreset::ShortFur) {
            surface.surface = Physics::HairSurface::Capsule;
            surface.radius = 0.22f;
        }
        if (!Physics::generateHairSurface(next, surface)) return false;
        surface.strands *= 8;
        surface.particlesPerStrand = 2;
        Physics::HairStrands seeds;
        if (!Physics::generateHairSurface(seeds, surface)) return false;
        for (const auto& r : seeds.ranges()) followerRoots.push_back(r.root);
    } else if (!Physics::generateHairBundle(next, p)) return false;
    clear();
    solver_.setParams(settings);
    strands_ = std::move(next);
    solver_.setStrands(&strands_);
    if (!followerRoots.empty() && !followers_.initialize(strands_, followerRoots, p.particlesPerStrand))
        return false;
    for (const auto& r : strands_.ranges()) baseRoots_.push_back(r.root);
    if (body) {
        baseColliders_ = {{{0.f,1.05f,0.f}, {0.f,1.05f,0.f}, 0.28f, 0.3f},
                         {{0.f,0.3f,-0.18f}, {0.f,0.3f,0.18f}, 0.12f, 0.3f}};
        solver_.setColliders(baseColliders_, true);
        settings.numIterations = std::max(settings.numIterations, 32);
        settings.windVelocity = preset == HairPreset::StrongWind ? Math::Vector3df(12.f,0.f,0.f) : Math::Vector3df(0.f);
        settings.windDrag = preset == HairPreset::StrongWind ? 2.5f : 0.f;
        solver_.setParams(settings);
    }
    motion_ = preset == HairPreset::HeadShake;
    if (registry_) {
        componentId_ = registry_->add(SceneComponentKind::Hair, "Hair", [this] {
            return std::to_string(strands_.strandCount()) + " guides, " +
                   std::to_string(strands_.particleCount()) + " particles, " +
                   std::to_string(followers_.strandCount()) + " followers";
        });
    }
    return true;
}
void HairWorld::clear() {
    removeComponent();
    solver_.setStrands(nullptr);
    solver_.setColliders({}, true);
    followers_ = Physics::HairFollowers{};
    strands_ = Physics::HairStrands{};
    running_ = false;
    accumulator_ = 0.;
    droppedTime_ = 0.;
    baseRoots_.clear();
    baseColliders_.clear();
    rigPose_ = completedRig_ = {};
    motion_ = false;
}
void HairWorld::reset() {
    applyRig(rigPose_, true);
    solver_.reset();
    followers_.update(strands_);
    accumulator_ = 0.;
    droppedTime_ = 0.;
}
bool HairWorld::stepOnce() {
    accumulator_ = 0.;
    const bool ok = advanceOnce();
    if (!ok) running_ = false;
    return ok;
}
bool HairWorld::setParams(const Physics::HairSolver::Params& p) {
    if (!solver_.setParams(p)) return false;
    accumulator_ = 0.;
    return true;
}

Physics::HairRootPose HairWorld::animatedRig(double time) const {
    auto pose = rigPose_;
    if (motion_) {
        const float phase = static_cast<float>(time*6.283185307);
        pose.position.x += 0.08f*std::sin(phase);
        pose.rotation = rigPose_.rotation * glm::angleAxis(0.35f*std::sin(phase), Math::Vector3df(0.f,1.f,0.f));
    }
    return pose;
}
bool HairWorld::applyRig(const Physics::HairRootPose& pose, bool teleport) {
    const Math::Vector3df pivot(0.f,1.05f,0.f);
    const float angle = 2.f*std::acos(std::clamp(std::abs(glm::dot(completedRig_.rotation,pose.rotation)),0.f,1.f));
    const bool jumped = teleport || glm::length(completedRig_.position-pose.position) > params().teleportDistance || angle > params().teleportAngle;
    auto colliders = baseColliders_;
    for (auto& c : colliders) {
        c.a = pivot+pose.position+pose.rotation*(c.a-pivot);
        c.b = pivot+pose.position+pose.rotation*(c.b-pivot);
    }
    if (!solver_.setColliders(colliders, jumped)) return false;
    for (size_t s = 0; s < baseRoots_.size(); ++s) {
        Physics::HairRootPose root;
        root.position = pivot+pose.position+pose.rotation*(baseRoots_[s].position-pivot);
        root.rotation = pose.rotation*baseRoots_[s].rotation;
        if (!solver_.setRootPose(s, root, jumped)) return false;
    }
    if (jumped) {
        completedRig_ = pose;
        if (!followers_.update(strands_)) return false;
    }
    return true;
}
bool HairWorld::setRigPose(const Physics::HairRootPose& pose, bool teleport) {
    if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) || !std::isfinite(pose.position.z) ||
        glm::length(pose.position) > 1000.f) return false;
    const float norm = glm::dot(pose.rotation,pose.rotation);
    if (!std::isfinite(norm) || norm < 1.e-12f) return false;
    auto normalized = pose;
    normalized.rotation = glm::normalize(pose.rotation);
    const auto old = rigPose_;
    rigPose_ = normalized;
    if (!applyRig(animatedRig(stats().simulatedTime), teleport)) { rigPose_ = old; return false; }
    accumulator_ = 0.;
    return true;
}
bool HairWorld::setFriction(float friction) {
    if (!std::isfinite(friction) || friction < 0.f || friction > 1.f) return false;
    for (auto& c : baseColliders_) c.friction = friction;
    return applyRig(animatedRig(stats().simulatedTime), false);
}
bool HairWorld::advanceOnce() {
    const auto pose = animatedRig(stats().simulatedTime+params().timeStep);
    if (!applyRig(pose, false) || !solver_.step()) return false;
    completedRig_ = pose;
    return followers_.update(strands_);
}
bool HairWorld::update(double elapsedSeconds) {
    if (!running_ || strands_.particleCount() == 0 || !std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.)
        return false;
    const double dt = params().timeStep;
    // Bound catch-up work and account for discarded real time.
    const double budget = 8. * dt;
    if (elapsedSeconds > budget) droppedTime_ += elapsedSeconds - budget;
    accumulator_ += std::min(elapsedSeconds, budget);
    bool changed = false;
    int steps = 0;
    while (accumulator_ + 1.e-10 >= dt && steps < 8) {
        if (!advanceOnce()) { running_ = false; accumulator_ = 0.; return true; }
        accumulator_ = std::max(0., accumulator_ - dt);
        changed = true;
        ++steps;
    }
    return changed;
}
float HairWorld::tipY() const {
    return strands_.particleCount() ? strands_.particles().positions.back().y : 0.f;
}
float HairWorld::tipX() const {
    return strands_.particleCount() ? strands_.particles().positions.back().x : 0.f;
}
float HairWorld::maxRootError() const {
    float error = 0.f;
    for (const auto& r : strands_.ranges())
        error = std::max(error, glm::length(strands_.particles().positions[r.offset] - r.root.position));
    return error;
}
HairWorld::WireData HairWorld::buildWireData() const {
    WireData out;
    out.positions.reserve(strands_.particleCount()*3);
    out.colors.reserve(strands_.particleCount()*4);
    for (const auto& r : strands_.ranges()) {
        for (size_t j = 0; j < r.count; ++j) {
            const auto p = strands_.particles().positions[r.offset+j];
            out.positions.insert(out.positions.end(), {p.x, p.y, p.z});
            const float t = static_cast<float>(j) / (r.count-1);
            out.colors.insert(out.colors.end(), {1.f, 0.65f + 0.3f*t, 0.15f + 0.4f*t, 1.f});
            if (j) {
                out.indices.push_back(static_cast<uint32_t>(r.offset+j-1));
                out.indices.push_back(static_cast<uint32_t>(r.offset+j));
            }
        }
    }
    const auto& followerPositions = followers_.positions();
    const size_t count = followers_.particlesPerStrand();
    const auto first = static_cast<uint32_t>(out.positions.size()/3);
    for (size_t i = 0; i < followerPositions.size(); ++i) {
        const auto& point = followerPositions[i];
        const size_t j = i%count;
        const float t = static_cast<float>(j)/(count-1);
        out.positions.insert(out.positions.end(), {point.x,point.y,point.z});
        out.colors.insert(out.colors.end(), {0.8f,0.45f+0.3f*t,0.18f+0.25f*t,1.f});
        if (j) {
            out.indices.push_back(first+static_cast<uint32_t>(i-1));
            out.indices.push_back(first+static_cast<uint32_t>(i));
        }
    }
    auto addLine = [&](const Math::Vector3df& a, const Math::Vector3df& b) {
        const auto index = static_cast<uint32_t>(out.positions.size()/3);
        out.positions.insert(out.positions.end(), {a.x,a.y,a.z,b.x,b.y,b.z});
        out.colors.insert(out.colors.end(), {0.2f,0.6f,1.f,1.f, 0.2f,0.6f,1.f,1.f});
        out.indices.insert(out.indices.end(), {index,index+1});
    };
    for (const auto& c : solver_.colliders()) {
        // Rings plus capsule rails. Analytic collision does not use this wire mesh.
        const auto axis = glm::length(c.b-c.a) > 1.e-6f ? glm::normalize(c.b-c.a) : Math::Vector3df(0.f,1.f,0.f);
        const auto basis = std::abs(axis.x) < 0.8f ? Math::Vector3df(1.f,0.f,0.f) : Math::Vector3df(0.f,0.f,1.f);
        const auto u = glm::normalize(glm::cross(axis,basis));
        const auto v = glm::cross(axis,u);
        for (const auto& center : {c.a,c.b}) {
            for (int plane = 0; plane < 3; ++plane) {
                const auto x = plane == 0 ? u : axis;
                const auto y = plane == 2 ? v : (plane == 0 ? v : u);
                for (int j = 0; j < 32; ++j) {
                    const float a = j*6.2831853f/32.f, b = (j+1)*6.2831853f/32.f;
                    addLine(center+c.radius*(std::cos(a)*x+std::sin(a)*y),
                            center+c.radius*(std::cos(b)*x+std::sin(b)*y));
                }
            }
        }
        if (glm::length(c.b-c.a) > 1.e-6f)
            for (const auto& n : {u,-u,v,-v}) addLine(c.a+c.radius*n,c.b+c.radius*n);
    }
    return out;
}
} // namespace Phantom
