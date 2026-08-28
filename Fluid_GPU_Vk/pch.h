#pragma once

#include <vulkan/vulkan.h>

#include <vector>
#include <cstdint>
#include <cassert>
#include <cstdio>
#include <cmath>
#include <cstring>

#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Box3d.h"
#include "../../CGLib/Util/UnCopyable.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"
