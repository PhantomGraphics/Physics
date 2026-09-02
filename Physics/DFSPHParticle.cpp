#include "pch.h"

#include "DFSPHParticle.h"
#include "DFSPHFluid.h"

#include <algorithm>
#include <iostream>

using namespace Phantom::Math;
using namespace Phantom::Physics;

void DFSPHParticle::calculateDensity(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	densityRef() = 0.0f;
	densityRef() += (getKernel()->getCubicSpline(0.0f) * this->getMass());
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const float distance = Math::getDistance(this->getPosition(), n.getPosition());
		densityRef() += (getKernel()->getCubicSpline(distance) * n.getMass());
	}
}

void DFSPHParticle::calculateAlpha(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	Vector3df a(0, 0, 0);
	float b = 0.0f;
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const auto v = this->getPosition() - n.getPosition();
		const auto weight = (getKernel()->getCubicSplineGradient(v) * (float)n.getMass());
		a += weight;
		b += Math::getLengthSquared(weight);
	}

	// Kept so a following boundary pass can extend the gradient sum without
	// re-walking the fluid neighbors (see addBoundaryAlphaGradient()).
	alphaGradSumRef() = a;
	alphaGradSqSumRef() = b;
	finalizeAlpha();
}

void DFSPHParticle::addBoundaryAlphaGradient(const Vector3df& gradient)
{
	alphaGradSumRef() += gradient;
	finalizeAlpha();
}

void DFSPHParticle::finalizeAlpha()
{
	alphaRef() = Math::getLengthSquared(alphaGradSumRef()) + alphaGradSqSumRef();

	if (alphaRef() < 1.0e-6f) {
		alphaRef() = 1.0e-6f;
	}
}

void DFSPHParticle::calculateDpDt(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	dpdtRef() = 0.0f;
	for (auto ni : neighbors) {
		const auto& n = all[ni];
        const auto v = this->getPosition() - n.getPosition();
		const auto grad = getKernel()->getCubicSplineGradient(v);
		const auto relativeVelocity = this->getVelocity() - n.getVelocity();
		dpdtRef() += n.getMass() * glm::dot(relativeVelocity, grad);
	}
}

void DFSPHParticle::predictDensity(const float dt, const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	calculateDpDt(all, neighbors);
	predictedDensityRef() = getDensity() + dt * getDpDt();
}

void DFSPHParticle::calculateVelocityInDivergenceError(const float dt, const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
  if (dt <= 0.0f) {
		return;
	}

	// k_i already carries the 1/rho_i that Bender & Koschier 2015's
	// alpha_i = rho_i / (|sum m_j gradW_ij|^2 + sum |m_j gradW_ij|^2) folds in
	// -- see calculateVelocityInDensityError() below for why dividing by rho^2
	// again here would be wrong.
	const float k_i = std::max(getDpDt(), 0.0f) / (getAlpha() * dt);

	Vector3df dv(0, 0, 0);
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const auto k_j = std::max(n.getDpDt(), 0.0f) / (n.getAlpha() * dt);
		const auto v = this->getPosition() - n.getPosition();
		dv += n.getMass() * (k_i + k_j) * getKernel()->getCubicSplineGradient(v);
	}
	addVelocity(-dt * dv);
}

void DFSPHParticle::calculateVelocityInDensityError(const float dt, const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
 if (dt <= 0.0f) {
		return;
	}

	// Bender & Koschier 2015, eq. 8-9:
	//   alpha_i = rho_i / (|sum_j m_j gradW_ij|^2 + sum_j |m_j gradW_ij|^2)
	//   kappa_i = (rho*_i - rho_0) / dt^2 * alpha_i
	//   v_i    -= dt * sum_j m_j (kappa_i/rho_i + kappa_j/rho_j) gradW_ij
	// getAlpha() stores only the denominator, so kappa_i/rho_i is exactly the
	// k_i below and no further division by rho is due. Dividing by rho^2 here
	// (as this did until internal design notes 1.6) made the solve's
	// effective stiffness scale as 1/rho_0^2 -- and rho_0 scales as radius^6 at
	// a fixed effectLength/radius ratio, so the same scene at particle radius
	// 0.3 over-corrected by ~500x (diverging on any boundary-induced density
	// excess) while radius 1.0 under-corrected by ~3400x, leaving the
	// incompressibility solve barely acting at all.
	const auto k_i = std::max(this->getPredictedDensity() - fluid_->getDensity(), 0.0f) / (dt * dt * getAlpha());

	Vector3df dv(0, 0, 0);
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const auto k_j = std::max(n.getPredictedDensity() - n.getParent()->getDensity(), 0.0f) / (dt * dt * n.getAlpha());
		const auto v = this->getPosition() - n.getPosition();
		dv += n.getMass() * (k_i + k_j) * getKernel()->getCubicSplineGradient(v);
	}
	addVelocity(-dt * dv);
}

void DFSPHParticle::calculateViscosity(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const auto viscosityCoe = (this->fluid_->getViscosityCoe() + n.getParent()->getViscosityCoe()) * 0.5f;
		const auto velocityDiff = (n.getVelocity() - this->getVelocity());
		const auto distance = Math::getDistance(getPosition(), n.getPosition());
		addForce(viscosityCoe * velocityDiff * getKernel()->getCubicSpline(distance));
	}
}

void DFSPHParticle::calculateNormal(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	normalRef() = Vector3df(0.0f, 0.0f, 0.0f);
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const auto v = this->getPosition() - n.getPosition();
		const float volume = n.getMass() / std::max(n.getDensity(), 1.0e-6f);
		normalRef() += getKernel()->getPoly6KernelGradient(v) * volume;
	}
}

Vector3df DFSPHParticle::surfaceNormalHat() const
{
	const auto normal = getNormal();
	const auto effectLength = fluid_->getKernel()->getEffectLength();
	// See WCSPHParticle::surfaceNormalHat() -- nondimensionalize by
	// effectLength^2 so the same threshold applies at every scene scale
	// (internal design notes).
	constexpr float kSurfaceThresholdSq = 1.0e-4f;
	if (Math::getLengthSquared(normal) * effectLength * effectLength < kSurfaceThresholdSq) {
		return Vector3df(0.0f, 0.0f, 0.0f);
	}
	return glm::normalize(normal);
}

void DFSPHParticle::calculateSurfaceTension(const std::vector<DFSPHParticle>& all, const Space::NeighborIndexView neighbors)
{
	const auto normalHat = surfaceNormalHat();
	if (Math::getLengthSquared(normalHat) < 1.0e-12f) {
		return;
	}
	for (auto ni : neighbors) {
		const auto& n = all[ni];
		const auto tensionCoe = (this->fluid_->getTensionCoe() + n.getParent()->getTensionCoe()) * 0.5f;
		const float distance = Math::getDistance(this->getPosition(), n.getPosition());
		addForce(-tensionCoe * getKernel()->getPoly6KernelLaplacian(distance) * n.getMass() * normalHat);
	}
}

SPHKernel* DFSPHParticle::getKernel()
{
	return fluid_->getKernel();
}
