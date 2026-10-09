#include "HairWorld.h"
#include <cmath>
#include <algorithm>
#include <string>

namespace Phantom {
namespace {
Math::Vector3df sample(const std::vector<Math::Vector3df>& values, size_t offset, size_t count, float t) {
    const float index = t*static_cast<float>(count-1);
    const size_t a = std::min(static_cast<size_t>(index),count-1);
    return glm::mix(values[offset+a],values[offset+std::min(a+1,count-1)],index-static_cast<float>(a));
}
}
int HairWorld::lodUpdateScale() const {
    return std::min(1 << lodLevel_,std::max(1,static_cast<int>((1.f/15.f)/params().timeStep)));
}
size_t HairWorld::drawnFollowerCount() const {
    const size_t stride = size_t{1} << lodLevel_;
    return (followers_.strandCount()+stride-1)/stride;
}
bool HairWorld::setLodParams(const LodParams& p) {
    if (!std::isfinite(p.mediumDistance) || !std::isfinite(p.farDistance) || !std::isfinite(p.hysteresis) ||
        p.hysteresis < 0.f || p.mediumDistance <= p.hysteresis ||
        p.farDistance-p.mediumDistance <= 2.f*p.hysteresis || p.farDistance > 10000.f ||
        !std::isfinite(p.transitionSeconds) || p.transitionSeconds < 0.f || p.transitionSeconds > 2.f) return false;
    const auto old = lodParams_;
    lodParams_ = p;
    if (!setCameraDistance(cameraDistance_)) { lodParams_ = old; return false; }
    if (p.transitionSeconds == 0.f) transitionFrom_ = {};
    return true;
}
bool HairWorld::setCameraDistance(float distance) {
    if (!std::isfinite(distance) || distance < 0.f || distance > 100000.f) return false;
    int level = lodParams_.enabled ? lodLevel_ : 0;
    if (lodParams_.enabled) {
        while (level < 2 && distance > (level == 0 ? lodParams_.mediumDistance : lodParams_.farDistance)+lodParams_.hysteresis)
            ++level;
        while (level > 0 && distance < (level == 1 ? lodParams_.mediumDistance : lodParams_.farDistance)-lodParams_.hysteresis)
            --level;
    }
    if (level != lodLevel_ && !changeLod(level)) return false;
    cameraDistance_ = distance;
    return true;
}
bool HairWorld::changeLod(int level) {
    if (strands_.strandCount() == 0) { lodLevel_ = level; return true; }
    auto previousDisplay = lodParams_.transitionSeconds > 0.f ? canonicalWireData() : WireData{};
    std::vector<Physics::HairStrandInput> inputs;
    std::vector<Math::Vector3df> positions, velocities;
    const size_t stride = size_t{1} << level;
    for (size_t s = 0; s < strands_.strandCount(); ++s) {
        const auto& r = strands_.ranges()[s];
        const auto& reference = fullRest_[s];
        Physics::HairStrandInput in;
        in.root = r.root;
        in.inverseMass = reference.inverseMass;
        const size_t count = (reference.restPositions.size()-2+stride)/stride+1;
        for (size_t j = 0; j < count; ++j) {
            const float t = static_cast<float>(j)/static_cast<float>(count-1);
            const auto rest = sample(reference.restPositions,0,reference.restPositions.size(),t);
            in.restPositions.push_back(rest);
            // Interpolate displacement from rest, retaining curvature when returning to full resolution.
            const auto oldRest = sample(strands_.restPositions(),r.offset,r.count,t);
            positions.push_back(j == 0 ? r.root.position : sample(strands_.particles().positions,r.offset,r.count,t)+r.root.rotation*(rest-oldRest));
            velocities.push_back(j == 0 ? Math::Vector3df(0.f) : sample(strands_.particles().velocities,r.offset,r.count,t));
        }
        inputs.push_back(std::move(in));
    }
    Physics::HairStrands next;
    if (!next.initialize(inputs) || !next.setDynamicState(positions,velocities)) return false;
    auto roots = followerRoots_;
    const auto pose = animatedRig(stats().simulatedTime);
    const Math::Vector3df pivot(0.f,1.05f,0.f);
    for (auto& root : roots) {
        root.position = pivot+pose.position+pose.rotation*(root.position-pivot);
        root.rotation = pose.rotation*root.rotation;
    }
    // Validate interpolation before detaching the live solver.
    Physics::HairFollowers prepared;
    if (!roots.empty() && !prepared.initialize(next,roots,followerVertexCount_)) return false;
    solver_.setStrands(nullptr,true);
    followers_ = Physics::HairFollowers{};
    strands_ = std::move(next);
    solver_.setStrands(&strands_,true);
    if (!roots.empty()) followers_.initialize(strands_,roots,followerVertexCount_);
    lodLevel_ = level;
    transitionFrom_ = std::move(previousDisplay);
    transitionRig_ = completedRig_;
    transitionElapsed_ = 0.;
    return true;
}
float HairWorld::lodTransitionProgress() const {
    return transitionFrom_.positions.empty() ? 1.f :
        static_cast<float>(std::min(1.,transitionElapsed_/lodParams_.transitionSeconds));
}
bool HairWorld::advanceLodTransition(double seconds) {
    if (transitionFrom_.positions.empty()) return false;
    transitionElapsed_ += seconds;
    if (transitionElapsed_ >= lodParams_.transitionSeconds) transitionFrom_ = {};
    return true;
}
HairWorld::WireData HairWorld::canonicalWireData() const {
    WireData out;
    const float progress = lodTransitionProgress();
    const float blend = progress*progress*(3.f-2.f*progress);
    const auto rotation = completedRig_.rotation*glm::conjugate(transitionRig_.rotation);
    auto append = [&](const std::vector<Math::Vector3df>& points, const std::vector<float>& times, bool follower, float alpha) {
        const size_t first = out.positions.size()/3;
        const bool morph = !transitionFrom_.positions.empty();
        Math::Vector3df oldRoot(0.f);
        if (morph) oldRoot = {transitionFrom_.positions[first*3],transitionFrom_.positions[first*3+1],transitionFrom_.positions[first*3+2]};
        for (size_t j = 0; j < points.size(); ++j) {
            const size_t i = first+j;
            auto point = points[j];
            float opacity = alpha;
            if (morph) {
                const Math::Vector3df old(transitionFrom_.positions[i*3],transitionFrom_.positions[i*3+1],transitionFrom_.positions[i*3+2]);
                point = glm::mix(points.front()+rotation*(old-oldRoot),point,blend);
                opacity = glm::mix(transitionFrom_.colors[i*4+3],alpha,blend);
            }
            const float t = times[j];
            out.positions.insert(out.positions.end(),{point.x,point.y,point.z});
            if (follower) out.colors.insert(out.colors.end(),{0.8f,0.45f+0.3f*t,0.18f+0.25f*t,opacity});
            else out.colors.insert(out.colors.end(),{1.f,0.65f+0.3f*t,0.15f+0.4f*t,opacity});
            if (j) out.indices.insert(out.indices.end(),{static_cast<uint32_t>(i-1),static_cast<uint32_t>(i)});
        }
    };
    for (size_t s = 0; s < strands_.strandCount(); ++s) {
        const auto& r = strands_.ranges()[s];
        std::vector<float> times;
        // Union of all three LOD grids: removing redundant vertices at completion preserves the polyline.
        for (size_t stride : {size_t{1},size_t{2},size_t{4}}) {
            const size_t count = (fullRest_[s].restPositions.size()-2+stride)/stride+1;
            for (size_t j = 0; j < count; ++j) times.push_back(static_cast<float>(j)/(count-1));
        }
        std::sort(times.begin(),times.end());
        times.erase(std::unique(times.begin(),times.end()),times.end());
        std::vector<Math::Vector3df> points;
        for (float t : times) points.push_back(sample(strands_.particles().positions,r.offset,r.count,t));
        append(points,times,false,1.f);
    }
    const size_t count = followers_.particlesPerStrand();
    const size_t stride = size_t{1} << lodLevel_;
    for (size_t s = 0; s < followers_.strandCount(); ++s) {
        std::vector<float> times;
        std::vector<Math::Vector3df> points;
        for (size_t j = 0; j < count; ++j) {
            const size_t a = (j/stride)*stride, b = std::min(a+stride,count-1);
            const float fraction = b == a ? 0.f : static_cast<float>(j-a)/(b-a);
            points.push_back(glm::mix(followers_.positions()[s*count+a],followers_.positions()[s*count+b],fraction));
            times.push_back(static_cast<float>(j)/(count-1));
        }
        append(points,times,true,s%stride == 0 ? 1.f : 0.f);
    }
    return out;
}
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
    p.variation = generationParams_;
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
    if (preset == HairPreset::LongHair) p.strands = countParams_.longHairGuides;
    if (preset == HairPreset::ShortFur) p.strands = countParams_.shortFurGuides;
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
        surface.variation = generationParams_;
        surface.strands = p.strands;
        surface.particlesPerStrand = p.particlesPerStrand;
        surface.length = p.length;
        if (preset == HairPreset::ShortFur) {
            surface.surface = Physics::HairSurface::Capsule;
            surface.radius = 0.22f;
        }
        if (!Physics::generateHairSurface(next, surface)) return false;
        surface.strands = preset == HairPreset::LongHair ? countParams_.longHairFollowers : countParams_.shortFurFollowers;
        surface.particlesPerStrand = 2;
        Physics::HairStrands seeds;
        if (surface.strands > 0 && !Physics::generateHairSurface(seeds, surface)) return false;
        for (const auto& r : seeds.ranges()) followerRoots.push_back(r.root);
    } else if (!Physics::generateHairBundle(next, p)) return false;
    clear();
    solver_.setParams(settings);
    strands_ = std::move(next);
    for (const auto& r : strands_.ranges()) {
        Physics::HairStrandInput input;
        input.root = r.root;
        input.inverseMass = strands_.particles().inverseMasses[r.offset+1];
        input.restPositions.assign(strands_.restPositions().begin()+r.offset,strands_.restPositions().begin()+r.offset+r.count);
        fullRest_.push_back(std::move(input));
    }
    followerRoots_ = followerRoots;
    followerVertexCount_ = p.particlesPerStrand;
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
    return setCameraDistance(cameraDistance_);
}
bool HairWorld::setGenerationParams(const Physics::HairVariationParams& p) {
    if (!Physics::validateHairVariation(p)) return false;
    generationParams_ = p;
    return true;
}
bool HairWorld::setCountParams(const CountParams& p) {
    if (p.longHairGuides < 1 || p.longHairGuides > 10000 ||
        p.shortFurGuides < 1 || p.shortFurGuides > 10000 ||
        p.longHairFollowers < 0 || p.longHairFollowers > 10000 ||
        p.shortFurFollowers < 0 || p.shortFurFollowers > 10000) return false;
    countParams_ = p;
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
    lodLevel_ = 0;
    fullRest_.clear();
    followerRoots_.clear();
    followerVertexCount_ = 0;
    transitionFrom_ = {};
    transitionElapsed_ = 0.;
}
void HairWorld::reset() {
    transitionFrom_ = {};
    applyRig(rigPose_, true);
    solver_.reset();
    followers_.update(strands_);
    accumulator_ = 0.;
    droppedTime_ = 0.;
}
bool HairWorld::stepOnce() {
    accumulator_ = 0.;
    const bool ok = advanceOnce();
    if (ok) advanceLodTransition(params().timeStep);
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
        transitionFrom_ = {};
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
bool HairWorld::advanceOnce(int timeScale) {
    const auto base = params();
    auto effective = base;
    effective.timeStep *= static_cast<float>(timeScale);
    const auto pose = animatedRig(stats().simulatedTime+effective.timeStep);
    if (!solver_.setParams(effective)) return false;
    const bool ok = applyRig(pose,false) && solver_.step();
    solver_.setParams(base);
    if (!ok) return false;
    completedRig_ = pose;
    return followers_.update(strands_);
}
bool HairWorld::update(double elapsedSeconds) {
    if (!std::isfinite(elapsedSeconds) || elapsedSeconds <= 0.) return false;
    const bool displayChanged = advanceLodTransition(elapsedSeconds);
    if (!running_ || strands_.particleCount() == 0) return displayChanged;
    const double dt = params().timeStep*lodUpdateScale();
    // Bound catch-up work and account for discarded real time.
    const double budget = 8. * params().timeStep;
    if (elapsedSeconds > budget) droppedTime_ += elapsedSeconds - budget;
    accumulator_ += std::min(elapsedSeconds, budget);
    bool changed = displayChanged;
    int steps = 0;
    while (accumulator_ + 1.e-10 >= dt && steps < 8) {
        if (!advanceOnce(lodUpdateScale())) { running_ = false; accumulator_ = 0.; return true; }
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
    if (!transitionFrom_.positions.empty()) out = canonicalWireData();
    else {
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
    const size_t stride = size_t{1} << lodLevel_;
    for (size_t s = 0; s < followers_.strandCount(); s += stride) {
        for (size_t j = 0;; j = std::min(j+stride,count-1)) {
            const auto& point = followerPositions[s*count+j];
            const float t = static_cast<float>(j)/(count-1);
            const auto index = static_cast<uint32_t>(out.positions.size()/3);
            out.positions.insert(out.positions.end(), {point.x,point.y,point.z});
            out.colors.insert(out.colors.end(), {0.8f,0.45f+0.3f*t,0.18f+0.25f*t,1.f});
            if (j) out.indices.insert(out.indices.end(),{index-1,index});
            if (j == count-1) break;
        }
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
