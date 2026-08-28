#pragma once
#include "CGLib/Math/Box3d.h"
#include "CGLib/Math/Triangle3d.h"
#include "CGLib/Space/Space/Octree.h"

#include <vector>

namespace Phantom {
	namespace Physics {

/**
 * @brief A triangle-mesh boundary accelerated by an octree.
 *
 * Builds a spatial octree over a set of triangles for efficient
 * boundary queries in fluid simulations.
 */
class OctreeBoundary
{
public:
	OctreeBoundary()
	{
	};

	/**
	 * @brief Builds the octree from a set of triangles.
	 * @param triangles Triangle mesh defining the boundary surface.
	 */
	void build(const std::vector<Math::Triangle3df>& triangles);

private:
	Space::Octree octree;
};

	}
}
