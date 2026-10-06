#pragma once
#include "../Physics/CombustibleBody.h"
#include "../../CGLib/GltfRenderer/Gltf/GltfDocument.h"

namespace Phantom {
int combustibleColorLevel(const Physics::CombustibleSample& sample,int mode);
Gltf::GltfDocument makeCombustibleGltf(const Physics::CombustibleBody& body,int mode);
Gltf::GltfDocument makeCombustibleScalarGltf(const Physics::CombustibleBody& body);
}
