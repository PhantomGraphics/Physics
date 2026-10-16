#pragma once

#include "CylinderBoundary.h"
#include "IShapeBoundary.h"
#include "PlaneBoundary.h"
#include "PlateBoundary.h"
#include "SphereBoundary.h"

#include "CGLib/Math/Core/Box3d.h"

#include <memory>
#include <utility>
#include <vector>

namespace Phantom {
namespace Physics {

/**
 * @brief The analytic boundaries (box/plane walls, spheres, plates, cylinders and arbitrary
 * IShapeBoundary) one SPH solver owns -- the registration state WCSPH/DFSPH/PBSPH used to repeat.
 *
 * Contract (identical for the three solvers):
 *  - each typed setter replaces only its own list, so planes, spheres, plates and cylinders are
 *    layered on top of one another rather than overriding;
 *  - setShapes() replaces the generic list, add() appends to it (nullptr is ignored);
 *  - clear() empties every list, typed and generic alike;
 *  - forEach() visits planes, spheres, plates, cylinders, then generic shapes, each in
 *    registration order. The order is fixed because the solvers sum floating-point forces and
 *    densities over it; skipping empty slots never changes it.
 *
 * Not thread-safe to mutate, but forEach() is const and is called from OpenMP loops.
 */
class ShapeBoundarySet {
public:
	/** Replaces the plane list with the six inward-facing walls of `box`. */
	void setBox(const Math::Box3df& box) { planes_ = ownShapeBoundaries(makeBoxPlaneBoundaries(box)); }

	void setPlanes(std::vector<PlaneBoundary> planes) { planes_ = ownShapeBoundaries(std::move(planes)); }
	void setSpheres(std::vector<SphereBoundary> spheres) { spheres_ = ownShapeBoundaries(std::move(spheres)); }
	void setPlates(std::vector<PlateBoundary> plates) { plates_ = ownShapeBoundaries(std::move(plates)); }
	void setCylinders(std::vector<CylinderBoundary> cylinders) { cylinders_ = ownShapeBoundaries(std::move(cylinders)); }
	void setShapes(std::vector<std::shared_ptr<IShapeBoundary>> shapes) { shapes_ = std::move(shapes); }

	void add(std::shared_ptr<IShapeBoundary> shape) {
		if (shape) shapes_.push_back(std::move(shape));
	}

	void clear() {
		planes_.clear();
		spheres_.clear();
		plates_.clear();
		cylinders_.clear();
		shapes_.clear();
	}

	bool empty() const {
		return planes_.empty() && spheres_.empty() && plates_.empty() &&
		       cylinders_.empty() && shapes_.empty();
	}

	/** Calls fn(const IShapeBoundary&) for every non-null boundary, in the fixed order above. */
	template <class Fn>
	void forEach(Fn&& fn) const {
		visit(planes_, fn);
		visit(spheres_, fn);
		visit(plates_, fn);
		visit(cylinders_, fn);
		visit(shapes_, fn);
	}

private:
	using List = std::vector<std::shared_ptr<IShapeBoundary>>;

	template <class Fn>
	static void visit(const List& list, Fn& fn) {
		for (const auto& shape : list) {
			if (shape) fn(*shape);
		}
	}

	List planes_;
	List spheres_;
	List plates_;
	List cylinders_;
	List shapes_;
};

} // namespace Physics
} // namespace Phantom
