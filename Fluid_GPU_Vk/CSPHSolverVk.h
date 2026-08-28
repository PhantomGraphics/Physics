#pragma once

#include <vector>
#include <vulkan/vulkan.h>

#include "CSPHParticleBufferVk.h"
#include "CSPHGridBufferVk.h"
#include "../../CGLib/Math/Vector3d.h"
#include "../../CGLib/Math/Box3d.h"
#include "../../CGLib/Util/UnCopyable.h"
#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"
#include "../../CGLib/VulkanGraphics/VulkanBuffer.h"

namespace Phantom {
    namespace Physics {

// CSPH fluid solver accelerated with Vulkan Compute.
//
// All seven passes run in a single GPU submit (no CPU prefix-sum round-trip):
//   1. clear    窶・zero per-cell counts and starts (compute)
//   2. count    窶・count particles per cell (atomic, device-local)
//   3. prefix   窶・exclusive prefix sum: cellStart[i] = sum(cellCount[0..i-1])
//   4. scatter  窶・place particle indices into sortedIdx in cell order
//   5. density  窶・accumulate SPH density (Poly6 kernel)
//   6. force    窶・pressure (Spiky) + viscosity forces
//   7. integrate 窶・gravity + boundary spring, semi-implicit Euler
//
// Lifetime: ctx and pool must outlive this object.
//
// Usage:
//   CSPHSolverVk solver;
//   solver.setParams(p);
//   solver.build(ctx, pool);       // compile shaders, create pipelines
//   solver.upload(positions, masses);
//   for (int i = 0; i < steps; ++i) solver.simulate();
//   auto pos = solver.getPositions();
class CSPHSolverVk : private UnCopyable
{
public:
    struct ProfileStats {
        float waitPrevSecondMs  = 0.0f;
        float recordClearCountMs = 0.0f;
        float submitClearCountMs = 0.0f;
        float waitClearCountMs   = 0.0f;
        float passClearCountMs = 0.0f;
        float prefixSumMs      = 0.0f;
        float recordRestMs      = 0.0f;
        float submitRestMs      = 0.0f;
        float waitRestMs        = 0.0f;
        float passRestMs       = 0.0f;
        float totalSimulateMs  = 0.0f;
    };

    struct Params {
        float effectLength = 1.25f;
        float timeStep     = 0.005f;
        float restDensity  = 1000.0f;
        float pressureCoe  = 50.0f;
        float viscosityCoe = 0.1f;
        Math::Vector3df gravity   = { 0.0f, -9.8f, 0.0f };
        Math::Box3df    boundary;
        float           boundaryK = 10000.0f;
    };

    CSPHSolverVk()  = default;
    ~CSPHSolverVk() { destroy(); }

    // Compiles shaders and creates all Vulkan objects.
    // ctx and pool must remain valid until destroy() is called.
    bool build(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool);

    void setParams(const Params& p) { params = p; }
    const Params& getParams() const { return params; }

    void upload(const std::vector<Math::Vector3df>& positions,
                const std::vector<float>& masses);

    void simulate();

    std::vector<Math::Vector3df> getPositions() const;
    VkBuffer getPositionBuffer() const { return particleBuffer.posBuffer(); }

    const ProfileStats& getLastProfileStats() const { return profileStats_; }

    int getNumParticles() const { return particleBuffer.getNumParticles(); }
    int getGridNx()       const { return gridNx; }
    int getGridNy()       const { return gridNy; }
    int getGridNz()       const { return gridNz; }
    int getNumCells()     const { return numCells; }

    static std::vector<int> exclusivePrefixSum(const std::vector<int>& counts);

private:    // Non-owning references; caller manages lifetime.
    const Phantom::VKG::VulkanContext*     ctx_  = nullptr;
    const Phantom::VKG::VulkanCommandPool* pool_ = nullptr;

    // Simulation-owned Vulkan objects
    VkCommandPool     cmdPool    = VK_NULL_HANDLE;
    VkCommandBuffer   cmdBuf     = VK_NULL_HANDLE;
    VkDescriptorPool  descPool   = VK_NULL_HANDLE;
    VkDescriptorSetLayout descLayout = VK_NULL_HANDLE;
    VkDescriptorSet   descSet    = VK_NULL_HANDLE;
    VkPipelineLayout  pipeLayout = VK_NULL_HANDLE;
    VkFence           fence      = VK_NULL_HANDLE;

    VkPipeline pipeClear     = VK_NULL_HANDLE;
    VkPipeline pipeCount     = VK_NULL_HANDLE;
    VkPipeline pipePrefix    = VK_NULL_HANDLE;
    VkPipeline pipeScatter   = VK_NULL_HANDLE;
    VkPipeline pipeDensity   = VK_NULL_HANDLE;
    VkPipeline pipeForce     = VK_NULL_HANDLE;
    VkPipeline pipeIntegrate = VK_NULL_HANDLE;
    bool       hasInFlightSecondSubmit = false;

    Params params;

    CSPHParticleBufferVk particleBuffer;
    CSPHGridBufferVk     gridBuffer;
    Phantom::VKG::VulkanBuffer    kernelLutBuffer;

    static constexpr uint32_t kKernelLutSize = 1024;
    float kernelLutEffectLength_ = -1.0f;

    ProfileStats profileStats_{};

    int gridNx = 0, gridNy = 0, gridNz = 0;
    int numCells = 0;

    VkShaderModule buildShaderModule(const char* spvModuleRelativePath) const;
    VkPipeline     buildComputePipeline(VkShaderModule mod) const;

    void setupGrid();
    void updateDescriptorSet();
    void destroy();

    static void ssboBarrier(VkCommandBuffer cb);
    void dispatch(VkCommandBuffer cb, VkPipeline pipe,
                  const void* pushData, uint32_t pushSize, int count) const;
};

    }
}
