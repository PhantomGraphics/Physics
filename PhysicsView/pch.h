#pragma once

// GLM (defines must precede the include)
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

// Vulkan
#include <vulkan/vulkan.h>

// Application UI facade (keeps Dear ImGui out of PhysicsView sources)
#include "CGLib/UIWidgets/Immediate.h"

// Standard library
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>
