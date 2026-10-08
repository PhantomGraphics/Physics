#include "pch.h"
#include "HairControlPanel.h"

namespace Phantom {
namespace Im = UI::Immediate;
void HairControlPanel::drawContents() {
    if (!world_) return;
    auto& w = *world_;
    bool changed = false;
    Im::text("Hair guides (CPU XPBD / Space collision)");
    if (Im::button("Single strand")) changed = w.setPreset(HairPreset::Single);
    Im::sameLine();
    if (Im::button("Bundle (48 guides)")) changed = w.setPreset(HairPreset::Bundle);
    if (Im::button("Body collision")) { changed = w.setPreset(HairPreset::Body); if (frame_) frame_(); }
    Im::sameLine();
    if (Im::button("Head shake")) { changed = w.setPreset(HairPreset::HeadShake); if (frame_) frame_(); }
    Im::sameLine();
    if (Im::button("Strong wind")) { changed = w.setPreset(HairPreset::StrongWind); if (frame_) frame_(); }
    if (changed && frame_) frame_();
    Im::sameLine();
    if (Im::button("Frame guides") && frame_) frame_();
    bool running = w.isRunning();
    if (Im::checkbox("Running", running)) w.setRunning(running);
    Im::sameLine();
    if (Im::button("Step") && !w.isRunning()) changed |= w.stepOnce();
    Im::sameLine();
    if (Im::button("Reset")) { w.reset(); changed = true; }
    Im::sameLine();
    if (Im::button("Clear")) { w.clear(); changed = true; }
    auto p = w.params();
    bool edited = false;
    if (Im::collapsingHeader("Solver", false)) {
    edited |= Im::sliderFloat("Time step (s)", p.timeStep, 1.f/240.f, 1.f/30.f);
    edited |= Im::sliderInt("Substeps", p.numSubsteps, 1, 16);
    edited |= Im::sliderInt("Iterations", p.numIterations, 1, 64);
    edited |= Im::sliderFloat("Gravity Y (m/s2)", p.gravity.y, -20.f, 20.f);
    edited |= Im::sliderFloat("Damping (/s)", p.dampingRate, 0.f, 20.f);
    edited |= Im::sliderFloat("Stretch compliance", p.stretchCompliance, 0.f, 0.001f, "%.6f");
    edited |= Im::sliderFloat("Bend compliance", p.bendCompliance, 0.f, 0.01f, "%.6f");
    edited |= Im::checkbox("Rest shape enabled", p.shapeEnabled);
    edited |= Im::sliderFloat("Shape compliance", p.shapeCompliance, 0.f, 0.1f);
    }
    if (Im::collapsingHeader("Wind / collision", true)) {
        edited |= Im::sliderFloat("Wind X (m/s)", p.windVelocity.x, -30.f, 30.f);
        edited |= Im::sliderFloat("Wind Y (m/s)", p.windVelocity.y, -30.f, 30.f);
        edited |= Im::sliderFloat("Wind Z (m/s)", p.windVelocity.z, -30.f, 30.f);
        edited |= Im::sliderFloat("Wind drag (/s)", p.windDrag, 0.f, 10.f);
        edited |= Im::sliderFloat("Hair radius (m)", p.collisionRadius, 0.0001f, 0.02f, "%.4f");
        float friction = w.friction();
        if (Im::sliderFloat("Body friction", friction, 0.f, 1.f)) changed |= w.setFriction(friction);
    }
    if (Im::collapsingHeader("Root motion", false)) {
        bool motion = w.motionEnabled();
        if (Im::checkbox("Animated head shake", motion)) w.setMotion(motion);
        auto pose = w.rigPose();
        bool moved = Im::sliderFloat("Root translation X", pose.position.x, -2.f, 2.f);
        moved |= Im::sliderFloat("Root translation Y", pose.position.y, -2.f, 2.f);
        moved |= Im::sliderFloat("Root translation Z", pose.position.z, -2.f, 2.f);
        float yaw = glm::eulerAngles(pose.rotation).y;
        if (Im::sliderFloat("Root yaw (radians)", yaw, -1.5f, 1.5f)) {
            pose.rotation = glm::angleAxis(yaw, Math::Vector3df(0.f,1.f,0.f)); moved = true;
        }
        if (moved) changed |= w.setRigPose(pose);
        if (Im::button("Teleport to origin")) changed |= w.setRigPose(Physics::HairRootPose{}, true);
    }
    if (edited) w.setParams(p);
    const auto& s = w.stats();
    Im::separator();
    Im::text("Guides: %zu  Particles: %zu", w.strands().strandCount(), s.particleCount);
    Im::text("Time: %.3f s  Steps: %llu", s.simulatedTime, static_cast<unsigned long long>(s.steps));
    Im::text("Max speed: %.4f m/s", s.maxSpeed);
    Im::text("Max length error: %.3f %%", 100.f*s.maxRelativeLengthError);
    Im::text("Colliders: %zu  Penetration: %.5f m", w.colliderCount(), s.maxPenetration);
    Im::text("Pinned overlap: %.5f m  Root resets: %llu", s.pinnedPenetration, static_cast<unsigned long long>(s.rootResets));
    Im::text("Non-finite: %zu  Dropped time: %.3f s", s.nonFiniteCount, w.droppedTime());
    if (changed && changed_) changed_();
}
} // namespace Phantom
