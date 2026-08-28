#pragma once

#include "CGLib/Math/Vector3d.h"
#include "CGLib/Math/Matrix3d.h"

namespace Phantom {
	namespace Physics {

/**
 * @brief Collection of SPH smoothing kernel functions.
 *
 * Provides Poly6, Spiky, Viscosity, and Cubic Spline kernels along
 * with their gradients and Laplacians, parameterized by a support radius.
 */
class SPHKernel
{
public:
	SPHKernel() = default;

	/**
	 * @brief Constructs the kernel with the given support radius.
	 * @param effectLength Radius of kernel support (h).
	 */
	SPHKernel(const float effectLength);

	/**
	 * @brief Evaluates the Poly6 kernel W(r, h).
	 * @param distance Distance between two particles.
	 * @return Kernel value.
	 */
	float getPoly6Kernel(const float distance);

	/**
	 * @brief Evaluates the Poly6 kernel using squared distance.
	 * @param distanceSquared Squared distance between two particles.
	 * @return Kernel value.
	 */
	float getPoly6Kernel2(const float distanceSquared);

	/**
	 * @brief Computes the gradient of the Poly6 kernel.
	 * @param distanceVector Vector from neighbor to query particle.
	 * @return Gradient vector.
	 */
	Math::Vector3df getPoly6KernelGradient(const Math::Vector3df& distanceVector);

	/**
	 * @brief Returns the scalar coefficient of the Poly6 gradient at a distance.
	 * @param distance Distance between two particles.
	 * @return Gradient coefficient.
	 */
	float getPoly6KernelGradientCoe(const float distance);

	/**
	 * @brief Evaluates the Laplacian of the Poly6 kernel.
	 * @param distance Distance between two particles.
	 * @return Laplacian value.
	 */
	float getPoly6KernelLaplacian(const float distance);

	/**
	 * @brief Computes the gradient of the Spiky kernel.
	 * @param distanceVector Vector from neighbor to query particle.
	 * @return Gradient vector.
	 */
	Math::Vector3df getSpikyKernelGradient(const Math::Vector3df& distanceVector) const;

	/**
	 * @brief Returns the scalar weight of the Spiky kernel gradient at a distance.
	 * @param distance Distance between two particles.
	 * @return Gradient weight.
	 */
	float getSpikyKernelGradientWeight(const float distance);

	/**
	 * @brief Evaluates the Laplacian of the Viscosity kernel.
	 * @param distance Distance between two particles.
	 * @return Laplacian value.
	 */
	float getViscosityKernelLaplacian(const float distance);

	/**
	 * @brief Evaluates the Cubic Spline kernel W(r, h) from a scalar distance.
	 * @param distance Distance between two particles.
	 * @return Kernel value.
	 */
	float getCubicSpline(const float distance) const;

	/**
	 * @brief Evaluates the Cubic Spline kernel from a distance vector.
	 * @param v Distance vector between two particles.
	 * @return Kernel value.
	 */
	float getCubicSpline(const Math::Vector3df& v) const;

	/**
	 * @brief Computes the gradient of the Cubic Spline kernel.
	 * @param distanceVector Vector from neighbor to query particle.
	 * @return Gradient vector.
	 */
	Math::Vector3df getCubicSplineGradient(const Math::Vector3df& distanceVector);

	/**
	 * @brief Returns the kernel support radius.
	 * @return Effect length h.
	 */
	float getEffectLength() const { return effectLength; }

private:
	// Zero-initialized (rather than left indeterminate by `= default`) so a
	// kernel that was never given a support radius via the owning fluid's
	// setEffectLength() reads back as a deterministic, checkable "unset"
	// state (getEffectLength() == 0.f) instead of garbage -- see
	// docs/todo/PLAN_sph_scale_invariance.md Phase 5.
	float effectLength = 0.0f;
	float poly6KernelConstant = 0.0f;
	float spikyKernelGradConstant = 0.0f;
	float effectLengthSquared = 0.0f;
};

	}
}
