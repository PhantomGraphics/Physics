#include "SSFRSettings.h"
#include "../FluidRenderer/SSFluidRenderer.h"

namespace Phantom {
void SSFRSettings::setAnisotropicKernel(bool enabled) {
    if (!renderer_) return;
    renderer_->setAnisotropicKernel(enabled);
    markKernelChanged();
}
bool SSFRSettings::getAnisotropicKernel() const {
    return renderer_ && renderer_->getAnisotropicKernel();
}
bool SSFRSettings::isAnisotropicKernelActive() const {
    return renderer_ && renderer_->isAnisotropicKernelActive();
}
} // namespace Phantom
