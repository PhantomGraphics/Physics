#include "pch.h"

#include "SVBoundary.h"
#include "../../CGLib/Volume/Volume/LevelSet.h"


using namespace Phantom::Math;
using namespace Phantom::Physics;
using namespace Phantom::Volume;

void SVBoundary::build(const Box3df& boundary, const Math::Vector3df& cellLength, const int tableSize)
{
	(void)tableSize;
	const float voxelSize = (cellLength.x + cellLength.y + cellLength.z) / 3.0f;
	auto volume = std::make_unique<SparseVolumef>(0.0f);
	volume->setVoxelSize(voxelSize);

	LevelSet builder;
	builder.setSignedDistance(boundary, *volume, 2.0);
	this->volume = std::move(volume);
}