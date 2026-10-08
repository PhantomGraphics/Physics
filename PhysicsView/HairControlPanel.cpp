#include "pch.h"
#include "HairControlPanel.h"

namespace Phantom {
namespace Im = UI::Immediate;
void HairControlPanel::drawContents() {
    if (!world_) return;
    auto& w = *world_;
    bool changed = false;
    Im::text("Fixed-root guides (CPU XPBD)");
    if (Im::button("Single strand")) changed = w.setPreset(HairPreset::Single);
    Im::sameLine();
    if (Im::button("Bundle (48 guides)")) changed = w.setPreset(HairPreset::Bundle);
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
    bool edited = Im::sliderFloat("Time step (s)", p.timeStep, 1.f/240.f, 1.f/30.f);
    edited |= Im::sliderInt("Substeps", p.numSubsteps, 1, 16);
    edited |= Im::sliderInt("Iterations", p.numIterations, 1, 64);
    edited |= Im::sliderFloat("Gravity Y (m/s2)", p.gravity.y, -20.f, 20.f);
    edited |= Im::sliderFloat("Damping (/s)", p.dampingRate, 0.f, 20.f);
    edited |= Im::sliderFloat("Stretch compliance", p.stretchCompliance, 0.f, 0.001f, "%.6f");
    edited |= Im::sliderFloat("Bend compliance", p.bendCompliance, 0.f, 0.01f, "%.6f");
    edited |= Im::checkbox("Rest shape enabled", p.shapeEnabled);
    edited |= Im::sliderFloat("Shape compliance", p.shapeCompliance, 0.f, 0.1f);
    if (edited) w.setParams(p);
    const auto& s = w.stats();
    Im::separator();
    Im::text("Guides: %zu  Particles: %zu", w.strands().strandCount(), s.particleCount);
    Im::text("Time: %.3f s  Steps: %llu", s.simulatedTime, static_cast<unsigned long long>(s.steps));
    Im::text("Max speed: %.4f m/s", s.maxSpeed);
    Im::text("Max length error: %.3f %%", 100.f*s.maxRelativeLengthError);
    Im::text("Non-finite: %zu  Dropped time: %.3f s", s.nonFiniteCount, w.droppedTime());
    if (changed && changed_) changed_();
}
} // namespace Phantom
