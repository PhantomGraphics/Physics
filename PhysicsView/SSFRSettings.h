#pragma once
#include "../FluidRenderer/SSFRAnisotropy.h"
#include <cstdint>

namespace Phantom {
class SSFluidRenderer;

// Command-driven rendering state, independent of ImGui windows and widgets.
class SSFRSettings {
public:
    void bindRenderer(SSFluidRenderer* renderer) { renderer_ = renderer; }
    bool isEnabled() const { return enabled_; }
    void setEnabled(bool enabled) { enabled_ = enabled; }
    int getModeIndex() const { return modeIndex_; }
    void setModeIndex(int index) { modeIndex_ = index; }
    SSFRKernelSettings& kernelSettings() { return kernelSettings_; }
    const SSFRKernelSettings& kernelSettings() const { return kernelSettings_; }
    void markKernelChanged() { ++kernelGeneration_; }
    uint64_t kernelGeneration() const { return kernelGeneration_; }
    void setKernelStats(const SSFRKernelStats& stats) { kernelStats_ = stats; }
    const SSFRKernelStats& kernelStats() const { return kernelStats_; }
    void setAnisotropicKernel(bool enabled);
    bool getAnisotropicKernel() const;
    bool isAnisotropicKernelActive() const;

private:
    SSFluidRenderer* renderer_ = nullptr;
    bool enabled_ = false;
    int modeIndex_ = 5;
    SSFRKernelSettings kernelSettings_;
    SSFRKernelStats kernelStats_;
    uint64_t kernelGeneration_ = 0;
};
} // namespace Phantom
