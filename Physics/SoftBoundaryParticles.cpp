#include "pch.h"
#include "SoftBoundaryParticles.h"
#include "BoundaryPsi.h"

namespace Phantom {
namespace Physics {

void SoftBoundaryParticles::bind(SoftMesh* mesh) {
    mesh_ = mesh;
    particles_.clear();
    if (mesh_) particles_.resize(mesh_->particles.size());
}

void SoftBoundaryParticles::sync() {
    if (!mesh_) return;
    for (size_t i = 0; i < particles_.size() && i < mesh_->particles.size(); ++i) {
        particles_[i].worldPos = mesh_->particles.positions[i];
    }
}

void SoftBoundaryParticles::computePsi(const SPHKernel& kernel, float restDensity) {
    std::vector<Math::Vector3df> positions;
    positions.reserve(particles_.size());
    for (const auto& bp : particles_) positions.push_back(bp.worldPos);

    std::vector<float> psi;
    computeBoundaryPsi(positions, kernel, restDensity, psi);

    for (size_t k = 0; k < particles_.size(); ++k) particles_[k].psi = psi[k];
}

} // namespace Physics
} // namespace Phantom
