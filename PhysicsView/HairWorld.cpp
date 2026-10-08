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
    if (preset == HairPreset::Single) p.strands = 1;
    Physics::HairStrands next;
    if (!Physics::generateHairBundle(next, p)) return false;
    clear();
    strands_ = std::move(next);
    solver_.setStrands(&strands_);
    if (registry_) {
        componentId_ = registry_->add(SceneComponentKind::Hair, "Hair", [this] {
            return std::to_string(strands_.strandCount()) + " guides, " +
                   std::to_string(strands_.particleCount()) + " particles";
        });
    }
    return true;
}
void HairWorld::clear() {
    removeComponent();
    solver_.setStrands(nullptr);
    strands_ = Physics::HairStrands{};
    running_ = false;
    accumulator_ = 0.;
    droppedTime_ = 0.;
}
void HairWorld::reset() {
    solver_.reset();
    accumulator_ = 0.;
    droppedTime_ = 0.;
}
bool HairWorld::stepOnce() {
    accumulator_ = 0.;
    const bool ok = solver_.step();
    if (!ok) running_ = false;
    return ok;
}
bool HairWorld::setParams(const Physics::HairSolver::Params& p) {
    if (!solver_.setParams(p)) return false;
    accumulator_ = 0.;
    return true;
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
        if (!solver_.step()) { running_ = false; accumulator_ = 0.; return true; }
        accumulator_ = std::max(0., accumulator_ - dt);
        changed = true;
        ++steps;
    }
    return changed;
}
float HairWorld::tipY() const {
    return strands_.particleCount() ? strands_.particles().positions.back().y : 0.f;
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
    return out;
}
} // namespace Phantom
