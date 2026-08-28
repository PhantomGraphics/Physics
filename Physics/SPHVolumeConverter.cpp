#include "pch.h"

#include "SPHVolumeConverter.h"

#include "CGLib/Space/Space/CompactSpaceHash.h"
#include "CGLib/Volume/Volume/SparseVolumeTree/SparseVolume.h"

#include "WPCA.h"
#include "SPHKernel.h"

using namespace Phantom::Math;
using namespace Phantom::Volume;
using namespace Phantom::Space;
using namespace Phantom::Physics;

// ============================================================================
// float overloads — canonical implementations
// ============================================================================

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::buildIsotoropic(
	const std::vector<Vector3df>& positions,
	const float particleRadius,
	const float cellLength)
{
	if (positions.empty()) {
		return nullptr;
	}

	particles.clear();

	for (const auto& p : positions) {
		particles.push_back(std::make_unique<SPHSurfaceParticle>(p, particleRadius));
	}

	calculateDensity(particleRadius);

	auto sv = createSparseVolume(cellLength);

	const SPHKernel kernel(particleRadius);
	const int halfCells = static_cast<int>(std::ceil(particleRadius / cellLength));

	for (const auto& p : particles) {
		if (p->getDensity() <= 0.0f) {
			continue;
		}

		const Vector3df pos   = p->getPosition();
		const float     mass  = p->getMass();
		const float     rho   = p->getDensity();
		const Coord     cBase = sv->worldToIndex(pos);

		for (int dz = -halfCells; dz <= halfCells; ++dz) {
			for (int dy = -halfCells; dy <= halfCells; ++dy) {
				for (int dx = -halfCells; dx <= halfCells; ++dx) {
					const Coord     c        = cBase + Coord(dx, dy, dz);
					const Vector3df voxelPos = sv->indexToWorld(c);
					const float     dist     = glm::length(voxelPos - pos);

					if (dist > particleRadius) {
						continue;
					}

					const float w = kernel.getCubicSpline(dist) * mass / rho;
					if (w <= 0.0f) {
						continue;
					}
					sv->setValue(c, sv->getValue(c) + w);
				}
			}
		}
	}

	return sv;
}

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::buildAnisotoropic(
	const std::vector<Vector3df>& positions,
	const float particleRadius,
	const float cellLength)
{
	if (positions.empty()) {
		return nullptr;
	}

	particles.clear();

	for (const auto& p : positions) {
		particles.push_back(std::make_unique<SPHSurfaceParticle>(p, particleRadius));
	}

	// calculateAnisotropy also computes density, so calculateDensity is not called separately.
	calculateAnisotropy(particleRadius);

	auto sv = createSparseVolume(cellLength);

	const SPHKernel kernel(particleRadius);

	// Conservative search box: the anisotropy ellipsoid can stretch the support
	// by at most kr in any direction (eigenvalue clamping: λ_j >= λ_max / kr).
	constexpr float kr = 4.0f;
	const int halfCells = static_cast<int>(std::ceil(particleRadius * kr / cellLength));

	for (const auto& p : particles) {
		if (p->getDensity() <= 0.0f) {
			continue;
		}

		const Vector3df pos   = p->getPosition();
		const float     mass  = p->getMass();
		const float     rho   = p->getDensity();
		const Matrix3dd G     = p->getMatrix();
		const float     detG  = static_cast<float>(std::abs(glm::determinant(G)));
		const Coord     cBase = sv->worldToIndex(pos);

		for (int dz = -halfCells; dz <= halfCells; ++dz) {
			for (int dy = -halfCells; dy <= halfCells; ++dy) {
				for (int dx = -halfCells; dx <= halfCells; ++dx) {
					const Coord     c        = cBase + Coord(dx, dy, dz);
					const Vector3df voxelPos = sv->indexToWorld(c);

					// Transform the displacement into the anisotropic kernel space.
					// v is computed in double precision to preserve accuracy near G.
					const Vector3dd v  = Vector3dd(voxelPos - pos);
					const Vector3dd Gv = G * v;
					const float     d  = static_cast<float>(glm::length(Gv));

					const float w = detG * kernel.getCubicSpline(d) * mass / rho;
					if (w <= 0.0f) {
						continue;
					}
					sv->setValue(c, sv->getValue(c) + w);
				}
			}
		}
	}

	return sv;
}

// ============================================================================
// double overloads — thin forwarding wrappers
// ============================================================================

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::buildIsotoropic(
	const std::vector<Vector3dd>& positions,
	const float particleRadius,
	const float cellLength)
{
	std::vector<Vector3df> fpos;
	fpos.reserve(positions.size());
	for (const auto& p : positions) {
		fpos.emplace_back(static_cast<float>(p.x),
		                  static_cast<float>(p.y),
		                  static_cast<float>(p.z));
	}
	return buildIsotoropic(fpos, particleRadius, cellLength);
}

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::buildAnisotoropic(
	const std::vector<Vector3dd>& positions,
	const float particleRadius,
	const float cellLength)
{
	std::vector<Vector3df> fpos;
	fpos.reserve(positions.size());
	for (const auto& p : positions) {
		fpos.emplace_back(static_cast<float>(p.x),
		                  static_cast<float>(p.y),
		                  static_cast<float>(p.z));
	}
	return buildAnisotoropic(fpos, particleRadius, cellLength);
}

// ============================================================================
// float overloads with pre-computed per-particle density
// ============================================================================

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::buildIsotoropic(
	const std::vector<Vector3df>& positions,
	const std::vector<float>& densities,
	const float particleRadius,
	const float cellLength)
{
	if (positions.empty())
		return nullptr;

	particles.clear();

	for (const auto& p : positions)
		particles.push_back(std::make_unique<SPHSurfaceParticle>(p, particleRadius));

	for (int i = 0; i < static_cast<int>(particles.size()); ++i)
		particles[i]->setDensity(densities[i]);

	auto sv = createSparseVolume(cellLength);

	const SPHKernel kernel(particleRadius);
	const int halfCells = static_cast<int>(std::ceil(particleRadius / cellLength));

	for (const auto& p : particles) {
		if (p->getDensity() <= 0.0f)
			continue;

		const Vector3df pos   = p->getPosition();
		const float     mass  = p->getMass();
		const float     rho   = p->getDensity();
		const Coord     cBase = sv->worldToIndex(pos);

		for (int dz = -halfCells; dz <= halfCells; ++dz) {
			for (int dy = -halfCells; dy <= halfCells; ++dy) {
				for (int dx = -halfCells; dx <= halfCells; ++dx) {
					const Coord     c        = cBase + Coord(dx, dy, dz);
					const Vector3df voxelPos = sv->indexToWorld(c);
					const float     dist     = glm::length(voxelPos - pos);

					if (dist > particleRadius)
						continue;

					const float w = kernel.getCubicSpline(dist) * mass / rho;
					if (w <= 0.0f)
						continue;
					sv->setValue(c, sv->getValue(c) + w);
				}
			}
		}
	}

	return sv;
}

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::buildAnisotoropic(
	const std::vector<Vector3df>& positions,
	const std::vector<float>& densities,
	const float particleRadius,
	const float cellLength)
{
	if (positions.empty())
		return nullptr;

	particles.clear();

	for (const auto& p : positions)
		particles.push_back(std::make_unique<SPHSurfaceParticle>(p, particleRadius));

	// calculateAnisotropy computes the anisotropy matrix (and internally also density).
	// Call it first to set the matrix, then override density with simulation values.
	calculateAnisotropy(particleRadius);

	for (int i = 0; i < static_cast<int>(particles.size()); ++i)
		particles[i]->setDensity(densities[i]);

	auto sv = createSparseVolume(cellLength);

	const SPHKernel kernel(particleRadius);
	constexpr float kr = 4.0f;
	const int halfCells = static_cast<int>(std::ceil(particleRadius * kr / cellLength));

	for (const auto& p : particles) {
		if (p->getDensity() <= 0.0f)
			continue;

		const Vector3df pos   = p->getPosition();
		const float     mass  = p->getMass();
		const float     rho   = p->getDensity();
		const Matrix3dd G     = p->getMatrix();
		const float     detG  = static_cast<float>(std::abs(glm::determinant(G)));
		const Coord     cBase = sv->worldToIndex(pos);

		for (int dz = -halfCells; dz <= halfCells; ++dz) {
			for (int dy = -halfCells; dy <= halfCells; ++dy) {
				for (int dx = -halfCells; dx <= halfCells; ++dx) {
					const Coord     c        = cBase + Coord(dx, dy, dz);
					const Vector3df voxelPos = sv->indexToWorld(c);

					const Vector3dd v  = Vector3dd(voxelPos - pos);
					const Vector3dd Gv = G * v;
					const float     d  = static_cast<float>(glm::length(Gv));

					const float w = detG * kernel.getCubicSpline(d) * mass / rho;
					if (w <= 0.0f)
						continue;
					sv->setValue(c, sv->getValue(c) + w);
				}
			}
		}
	}

	return sv;
}

// ============================================================================
// Private helpers
// ============================================================================

std::unique_ptr<Phantom::Volume::SparseVolumef> SPHVolumeConverter::createSparseVolume(const float cellLength)
{
	auto sv = std::make_unique<Phantom::Volume::SparseVolumef>(0.0f);
	sv->setVoxelSize(cellLength);
	return sv;
}

void SPHVolumeConverter::calculateDensity(const float searchRadius)
{
	const SPHKernel kernel(searchRadius);

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(particles.size()));

	for (const auto& p : particles) {
		spaceHash.add(p->getPosition());
	}

#pragma omp parallel for
	for (int i = 0; i < particles.size(); ++i) {
		auto& p = particles[i];
		const auto indices = spaceHash.findNeighborIndices(p->getPosition());
		std::vector<Vector3df> neighbors;
		for (const auto& ix : indices) {
			neighbors.push_back(particles[ix]->getPosition());
		}
		p->calculateDensity(neighbors, searchRadius, kernel);
	}
}

void SPHVolumeConverter::calculateAnisotropy(const float searchRadius)
{
	const SPHKernel kernel(searchRadius);

	CompactSpaceHash spaceHash(searchRadius, static_cast<int>(particles.size()));

	for (const auto& p : particles) {
		spaceHash.add(p->getPosition());
	}

#pragma omp parallel for
	for (int i = 0; i < particles.size(); ++i) {
		auto& p = particles[i];
		const auto indices = spaceHash.findNeighborIndices(p->getPosition());
		std::vector<Vector3df> neighbors;
		for (const auto& ix : indices) {
			neighbors.push_back(particles[ix]->getPosition());
		}

		p->calculateDensity(neighbors, searchRadius, kernel);
		WPCA wpca;
		wpca.setup(p->getPosition(), neighbors, searchRadius);
		const auto wm = wpca.calculateWeightedMean(p->getPosition(), neighbors, searchRadius);
		p->correctedPosition(0.95f, wm);
		p->calculateAnisotoropicMatrix(neighbors, searchRadius);
	}
}
