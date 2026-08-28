#include "pch.h"

#include "CSPHSolverVk.h"

#include "../../CGLib/VulkanGraphics/VulkanSPVResolver.h"

#include <chrono>
#include <cmath>

using namespace Phantom::Math;
using namespace Phantom::Physics;

// ---------------------------------------------------------------------------
// Push-constant structs mirroring each shader's layout(push_constant) block.
// All fields are 4 bytes, ordered identically to the GLSL declaration.
// ---------------------------------------------------------------------------
struct PCClear {
    uint32_t numCells;
};

struct PCCountScatter {
    uint32_t numParticles;
    float    cellSize;
    int32_t  gDimX, gDimY, gDimZ;
    float    gOrigX, gOrigY, gOrigZ;
};

struct PCDensity {
    uint32_t numParticles;
    float    effectLength;
    float    cellSize;
    int32_t  gDimX, gDimY, gDimZ;
    float    gOrigX, gOrigY, gOrigZ;
};

struct PCForce {
    uint32_t numParticles;
    float    effectLength;
    float    restDensity;
    float    pressureCoe;
    float    viscosityCoe;
    float    cellSize;
    int32_t  gDimX, gDimY, gDimZ;
    float    gOrigX, gOrigY, gOrigZ;
};

struct PCIntegrate {
    uint32_t numParticles;
    float    timeStep;
    float    gravX, gravY, gravZ;
    float    bMinX, bMinY, bMinZ;
    float    bMaxX, bMaxY, bMaxZ;
    float    boundaryK;
};

// Maximum push-constant size used across all passes (PCForce / PCIntegrate = 48 bytes).
static constexpr uint32_t kMaxPCSize = 48;

// ---------------------------------------------------------------------------
// buildShaderModule
// ---------------------------------------------------------------------------
VkShaderModule CSPHSolverVk::buildShaderModule(const char* spvModuleRelativePath) const
{
    std::vector<uint32_t> spirv = Phantom::VKG::loadSPVRepo(spvModuleRelativePath);
    if (spirv.empty()) {
        std::fprintf(stderr, "[CSPHSolverVk] Failed to load SPIR-V (%s)\n", spvModuleRelativePath);
        return VK_NULL_HANDLE;
    }

    VkShaderModuleCreateInfo smci{ VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO };
    smci.codeSize = spirv.size() * sizeof(uint32_t);
    smci.pCode    = spirv.data();

    VkShaderModule mod = VK_NULL_HANDLE;
    if (vkCreateShaderModule(ctx_->getDevice(), &smci, nullptr, &mod) != VK_SUCCESS) {
        std::fprintf(stderr, "[CSPHSolverVk] vkCreateShaderModule failed (%s)\n", spvModuleRelativePath);
        return VK_NULL_HANDLE;
    }
    return mod;
}

// ---------------------------------------------------------------------------
// buildComputePipeline
// ---------------------------------------------------------------------------
VkPipeline CSPHSolverVk::buildComputePipeline(VkShaderModule mod) const
{
    VkPipelineShaderStageCreateInfo stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
    stage.stage  = VK_SHADER_STAGE_COMPUTE_BIT;
    stage.module = mod;
    stage.pName  = "main";

    VkComputePipelineCreateInfo pci{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
    pci.stage  = stage;
    pci.layout = pipeLayout;

    VkPipeline pipe = VK_NULL_HANDLE;
    VkResult r = vkCreateComputePipelines(ctx_->getDevice(), VK_NULL_HANDLE,
                                          1, &pci, nullptr, &pipe);
    vkDestroyShaderModule(ctx_->getDevice(), mod, nullptr);

    if (r != VK_SUCCESS) {
        std::fprintf(stderr, "[CSPHSolverVk] vkCreateComputePipelines failed\n");
        return VK_NULL_HANDLE;
    }
    return pipe;
}

// ---------------------------------------------------------------------------
// setupGrid
// ---------------------------------------------------------------------------
void CSPHSolverVk::setupGrid()
{
    const auto& b   = params.boundary;
    const float h   = params.effectLength;
    const auto  len = b.getMax() - b.getMin();

    gridNx   = static_cast<int>(std::ceil(len.x / h)) + 1;
    gridNy   = static_cast<int>(std::ceil(len.y / h)) + 1;
    gridNz   = static_cast<int>(std::ceil(len.z / h)) + 1;
    numCells = gridNx * gridNy * gridNz;
}

// ---------------------------------------------------------------------------
// updateDescriptorSet
// ---------------------------------------------------------------------------
void CSPHSolverVk::updateDescriptorSet()
{
    struct BufInfo { VkBuffer buf; uint32_t binding; };
    const BufInfo bufs[] = {
        { particleBuffer.posBuffer(),   0 },
        { particleBuffer.velBuffer(),   1 },
        { particleBuffer.forceBuffer(), 2 },
        { gridBuffer.cellCountBuffer(), 3 },
        { gridBuffer.cellStartBuffer(), 4 },
        { gridBuffer.sortedIdxBuffer(), 5 },
        { kernelLutBuffer.get(),        6 },
    };

    VkDescriptorBufferInfo bufInfos[7]{};
    VkWriteDescriptorSet   writes[7]{};

    for (int i = 0; i < 7; ++i) {
        bufInfos[i].buffer = bufs[i].buf;
        bufInfos[i].offset = 0;
        bufInfos[i].range  = VK_WHOLE_SIZE;

        writes[i].sType           = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet          = descSet;
        writes[i].dstBinding      = bufs[i].binding;
        writes[i].descriptorCount = 1;
        writes[i].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        writes[i].pBufferInfo     = &bufInfos[i];
    }
    vkUpdateDescriptorSets(ctx_->getDevice(), 7, writes, 0, nullptr);
}

// ---------------------------------------------------------------------------
// build
// ---------------------------------------------------------------------------
bool CSPHSolverVk::build(const Phantom::VKG::VulkanContext& ctx, const Phantom::VKG::VulkanCommandPool& pool)
{
    ctx_  = &ctx;
    pool_ = &pool;
    VkDevice dev = ctx_->getDevice();

    // Command pool for simulation dispatches (graphics queue supports compute)
    {
        VkCommandPoolCreateInfo cpci{ VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO };
        cpci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        cpci.queueFamilyIndex = ctx_->getGraphicsQueueFamily();
        if (vkCreateCommandPool(dev, &cpci, nullptr, &cmdPool) != VK_SUCCESS) return false;
    }

    {
        VkCommandBufferAllocateInfo cbai{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO };
        cbai.commandPool        = cmdPool;
        cbai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cbai.commandBufferCount = 1;
        if (vkAllocateCommandBuffers(dev, &cbai, &cmdBuf) != VK_SUCCESS) return false;
    }

    // Descriptor set layout: 7 x SSBO (particle/grid + kernel LUT)
    {
        VkDescriptorSetLayoutBinding bindings[7]{};
        for (uint32_t i = 0; i < 7; ++i) {
            bindings[i].binding         = i;
            bindings[i].descriptorType  = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
            bindings[i].descriptorCount = 1;
            bindings[i].stageFlags      = VK_SHADER_STAGE_COMPUTE_BIT;
        }
        VkDescriptorSetLayoutCreateInfo dslci{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO };
        dslci.bindingCount = 7;
        dslci.pBindings    = bindings;
        if (vkCreateDescriptorSetLayout(dev, &dslci, nullptr, &descLayout) != VK_SUCCESS) return false;
    }

    {
        VkDescriptorPoolSize ps{ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 7 };
        VkDescriptorPoolCreateInfo dpci{ VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO };
        dpci.maxSets       = 1;
        dpci.poolSizeCount = 1;
        dpci.pPoolSizes    = &ps;
        if (vkCreateDescriptorPool(dev, &dpci, nullptr, &descPool) != VK_SUCCESS) return false;
    }

    {
        VkDescriptorSetAllocateInfo dsai{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        dsai.descriptorPool     = descPool;
        dsai.descriptorSetCount = 1;
        dsai.pSetLayouts        = &descLayout;
        if (vkAllocateDescriptorSets(dev, &dsai, &descSet) != VK_SUCCESS) return false;
    }

    // Pipeline layout shared by all 6 compute pipelines
    {
        VkPushConstantRange pcRange{};
        pcRange.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        pcRange.offset     = 0;
        pcRange.size       = kMaxPCSize;

        VkPipelineLayoutCreateInfo plci{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        plci.setLayoutCount         = 1;
        plci.pSetLayouts            = &descLayout;
        plci.pushConstantRangeCount = 1;
        plci.pPushConstantRanges    = &pcRange;
        if (vkCreatePipelineLayout(dev, &plci, nullptr, &pipeLayout) != VK_SUCCESS) return false;
    }

    // Compile shaders and create compute pipelines
    {
        VkShaderModule m;

        m = buildShaderModule("shaders/csph_clear.comp.spv");
        if (!m) return false;
        pipeClear = buildComputePipeline(m);
        if (!pipeClear) return false;

        m = buildShaderModule("shaders/csph_count.comp.spv");
        if (!m) return false;
        pipeCount = buildComputePipeline(m);
        if (!pipeCount) return false;

        m = buildShaderModule("shaders/csph_prefix.comp.spv");
        if (!m) return false;
        pipePrefix = buildComputePipeline(m);
        if (!pipePrefix) return false;

        m = buildShaderModule("shaders/csph_scatter.comp.spv");
        if (!m) return false;
        pipeScatter = buildComputePipeline(m);
        if (!pipeScatter) return false;

        m = buildShaderModule("shaders/csph_density.comp.spv");
        if (!m) return false;
        pipeDensity = buildComputePipeline(m);
        if (!pipeDensity) return false;

        m = buildShaderModule("shaders/csph_force.comp.spv");
        if (!m) return false;
        pipeForce = buildComputePipeline(m);
        if (!pipeForce) return false;

        m = buildShaderModule("shaders/csph_integrate.comp.spv");
        if (!m) return false;
        pipeIntegrate = buildComputePipeline(m);
        if (!pipeIntegrate) return false;
    }

    {
        VkFenceCreateInfo fci{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
        if (vkCreateFence(dev, &fci, nullptr, &fence) != VK_SUCCESS) return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// upload
// ---------------------------------------------------------------------------
void CSPHSolverVk::upload(const std::vector<Vector3df>& positions,
                           const std::vector<float>& masses)
{
    const int n = static_cast<int>(positions.size());
    assert(n > 0 && static_cast<int>(masses.size()) == n);

    setupGrid();

    std::vector<Vector3df> zeroVel(n, Vector3df(0.0f));

    particleBuffer.create(*ctx_, *pool_, n);
    particleBuffer.upload(*ctx_, *pool_, positions, masses, zeroVel);

    gridBuffer.create(*ctx_, *pool_, numCells, n);

    // Build SPH kernel LUT (normalized distance t in [0, 1]).
    // Table stores:
    //   x = poly6(dist^2, h)
    //   y = spikyGradCoef(dist, h)   so grad = rij * y
    //   z = viscLaplacian(dist, h)
    //   w = reserved
    std::vector<glm::vec4> lut(kKernelLutSize, glm::vec4(0.0f));
    const float h = params.effectLength;
    const float h2 = h * h;
    const float invPI = 1.0f / 3.14159265358979323846f;
    const float poly6C = (315.0f / 64.0f) * invPI * std::pow(h, -9.0f);
    const float spikyC = 45.0f * invPI * std::pow(h, -6.0f);
    const float viscC  = 45.0f * invPI * std::pow(h, -6.0f);

    for (uint32_t i = 0; i < kKernelLutSize; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(kKernelLutSize - 1);
        const float dist = t * h;
        const float distSq = dist * dist;

        float poly6 = 0.0f;
        if (distSq < h2) {
            const float d = h2 - distSq;
            poly6 = poly6C * d * d * d;
        }

        float spikyGradCoef = 0.0f;
        if (dist < h && dist > 1e-6f) {
            const float hd = h - dist;
            spikyGradCoef = spikyC * (hd * hd) / dist;
        }

        float viscLap = 0.0f;
        if (dist < h) {
            viscLap = viscC * (h - dist);
        }

        lut[i] = glm::vec4(poly6, spikyGradCoef, viscLap, 0.0f);
    }

    if (!kernelLutBuffer.isValid()) {
        kernelLutBuffer.create(*ctx_, *pool_,
                               sizeof(glm::vec4) * lut.size(),
                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                               lut.data());
    }
    else {
        kernelLutBuffer.upload(*ctx_, *pool_, lut.data(), sizeof(glm::vec4) * lut.size());
    }
    kernelLutEffectLength_ = h;

    updateDescriptorSet();
}

// ---------------------------------------------------------------------------
// ssboBarrier
// ---------------------------------------------------------------------------
void CSPHSolverVk::ssboBarrier(VkCommandBuffer cb)
{
    VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
    mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    mb.dstAccessMask = VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT;

    vkCmdPipelineBarrier(cb,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
        0, 1, &mb, 0, nullptr, 0, nullptr);
}

// ---------------------------------------------------------------------------
// dispatch
// ---------------------------------------------------------------------------
void CSPHSolverVk::dispatch(VkCommandBuffer cb, VkPipeline pipe,
                             const void* pushData, uint32_t pushSize,
                             int count) const
{
    vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_COMPUTE, pipe);
    vkCmdPushConstants(cb, pipeLayout, VK_SHADER_STAGE_COMPUTE_BIT,
                       0, pushSize, pushData);
    const uint32_t groups = (static_cast<uint32_t>(count) + 63u) / 64u;
    vkCmdDispatch(cb, groups, 1, 1);
}

// ---------------------------------------------------------------------------
// simulate
// ---------------------------------------------------------------------------
void CSPHSolverVk::simulate()
{
    using Clock = std::chrono::high_resolution_clock;
    const auto tSimStart = Clock::now();
    profileStats_ = {};

    if (hasInFlightSecondSubmit) {
        const auto tWaitPrevStart = Clock::now();
        vkWaitForFences(ctx_->getDevice(), 1, &fence, VK_TRUE, UINT64_MAX);
        const auto tWaitPrevEnd = Clock::now();
        profileStats_.waitPrevSecondMs = std::chrono::duration<float, std::milli>(tWaitPrevEnd - tWaitPrevStart).count();
        hasInFlightSecondSubmit = false;
    }

    const int n = particleBuffer.getNumParticles();
    assert(n > 0);

    const auto orig = params.boundary.getMin();
    const auto bMin = params.boundary.getMin();
    const auto bMax = params.boundary.getMax();

    PCClear pcClear;
    pcClear.numCells = static_cast<uint32_t>(numCells);

    PCCountScatter pcCS;
    pcCS.numParticles = static_cast<uint32_t>(n);
    pcCS.cellSize     = params.effectLength;
    pcCS.gDimX        = gridNx;
    pcCS.gDimY        = gridNy;
    pcCS.gDimZ        = gridNz;
    pcCS.gOrigX       = orig.x;
    pcCS.gOrigY       = orig.y;
    pcCS.gOrigZ       = orig.z;

    PCDensity pcDen;
    pcDen.numParticles = static_cast<uint32_t>(n);
    pcDen.effectLength = params.effectLength;
    pcDen.cellSize     = params.effectLength;
    pcDen.gDimX        = gridNx;
    pcDen.gDimY        = gridNy;
    pcDen.gDimZ        = gridNz;
    pcDen.gOrigX       = orig.x;
    pcDen.gOrigY       = orig.y;
    pcDen.gOrigZ       = orig.z;

    PCForce pcForce;
    pcForce.numParticles = static_cast<uint32_t>(n);
    pcForce.effectLength = params.effectLength;
    pcForce.restDensity  = params.restDensity;
    pcForce.pressureCoe  = params.pressureCoe;
    pcForce.viscosityCoe = params.viscosityCoe;
    pcForce.cellSize     = params.effectLength;
    pcForce.gDimX        = gridNx;
    pcForce.gDimY        = gridNy;
    pcForce.gDimZ        = gridNz;
    pcForce.gOrigX       = orig.x;
    pcForce.gOrigY       = orig.y;
    pcForce.gOrigZ       = orig.z;

    PCIntegrate pcInt;
    pcInt.numParticles = static_cast<uint32_t>(n);
    pcInt.timeStep     = params.timeStep;
    pcInt.gravX        = params.gravity.x;
    pcInt.gravY        = params.gravity.y;
    pcInt.gravZ        = params.gravity.z;
    pcInt.bMinX        = bMin.x;
    pcInt.bMinY        = bMin.y;
    pcInt.bMinZ        = bMin.z;
    pcInt.bMaxX        = bMax.x;
    pcInt.bMaxY        = bMax.y;
    pcInt.bMaxZ        = bMax.z;
    pcInt.boundaryK    = params.boundaryK;

    VkQueue queue = ctx_->getGraphicsQueue();

    // --- Record all 7 passes in a single command buffer ---
    VkCommandBufferBeginInfo begin{ VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO };
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    const auto tRecStart = Clock::now();
    vkResetCommandBuffer(cmdBuf, 0);
    vkBeginCommandBuffer(cmdBuf, &begin);

    vkCmdBindDescriptorSets(cmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE,
                            pipeLayout, 0, 1, &descSet, 0, nullptr);

    // Pass 1: clear cellCount[] and cellStart[]
    dispatch(cmdBuf, pipeClear, &pcClear, sizeof(PCClear), numCells);
    ssboBarrier(cmdBuf);

    // Pass 2: count particles per cell (atomic, device-local)
    dispatch(cmdBuf, pipeCount, &pcCS, sizeof(PCCountScatter), n);
    ssboBarrier(cmdBuf);

    // Pass 3: GPU prefix sum  cellCount[] -> cellStart[]
    // Single-workgroup sequential shader; dispatch(1,1,1)
    vkCmdBindPipeline(cmdBuf, VK_PIPELINE_BIND_POINT_COMPUTE, pipePrefix);
    vkCmdPushConstants(cmdBuf, pipeLayout, VK_SHADER_STAGE_COMPUTE_BIT,
                       0, sizeof(PCClear), &pcClear);
    vkCmdDispatch(cmdBuf, 1, 1, 1);
    ssboBarrier(cmdBuf);

    // Pass 4: scatter
    dispatch(cmdBuf, pipeScatter, &pcCS, sizeof(PCCountScatter), n);
    ssboBarrier(cmdBuf);

    // Pass 5: density
    dispatch(cmdBuf, pipeDensity, &pcDen, sizeof(PCDensity), n);
    ssboBarrier(cmdBuf);

    // Pass 6: force
    dispatch(cmdBuf, pipeForce, &pcForce, sizeof(PCForce), n);
    ssboBarrier(cmdBuf);

    // Pass 7: integrate
    dispatch(cmdBuf, pipeIntegrate, &pcInt, sizeof(PCIntegrate), n);

    // Make compute writes visible to vertex fetch
    {
        VkMemoryBarrier mb{ VK_STRUCTURE_TYPE_MEMORY_BARRIER };
        mb.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        mb.dstAccessMask = VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT;
        vkCmdPipelineBarrier(cmdBuf,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_VERTEX_INPUT_BIT,
            0, 1, &mb, 0, nullptr, 0, nullptr);
    }

    vkEndCommandBuffer(cmdBuf);
    const auto tRecEnd = Clock::now();
    profileStats_.recordClearCountMs = std::chrono::duration<float, std::milli>(tRecEnd - tRecStart).count();
    profileStats_.recordRestMs       = 0.0f;
    profileStats_.prefixSumMs        = 0.0f;

    // Single submit
    VkSubmitInfo si{ VK_STRUCTURE_TYPE_SUBMIT_INFO };
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmdBuf;
    vkResetFences(ctx_->getDevice(), 1, &fence);
    const auto tSubmitStart = Clock::now();
    vkQueueSubmit(queue, 1, &si, fence);
    const auto tSubmitEnd = Clock::now();
    profileStats_.submitClearCountMs = std::chrono::duration<float, std::milli>(tSubmitEnd - tSubmitStart).count();
    profileStats_.submitRestMs       = 0.0f;
    profileStats_.waitClearCountMs   = 0.0f;
    profileStats_.waitRestMs         = 0.0f;
    hasInFlightSecondSubmit = true;   // fence waited at start of next frame

    const auto tSimEnd = Clock::now();
    profileStats_.passClearCountMs = 0.0f;
    profileStats_.passRestMs       = 0.0f;
    profileStats_.totalSimulateMs  = std::chrono::duration<float, std::milli>(tSimEnd - tSimStart).count();
}

// ---------------------------------------------------------------------------
// getPositions
// ---------------------------------------------------------------------------
std::vector<Vector3df> CSPHSolverVk::getPositions() const
{
    std::vector<Vector3df> out;
    particleBuffer.downloadPositions(*ctx_, *pool_, out);
    return out;
}

// ---------------------------------------------------------------------------
// exclusivePrefixSum
// ---------------------------------------------------------------------------
std::vector<int> CSPHSolverVk::exclusivePrefixSum(const std::vector<int>& counts)
{
    std::vector<int> result(counts.size());
    int acc = 0;
    for (size_t i = 0; i < counts.size(); ++i) {
        result[i] = acc;
        acc += counts[i];
    }
    return result;
}

// ---------------------------------------------------------------------------
// destroy
// ---------------------------------------------------------------------------
void CSPHSolverVk::destroy()
{
    if (!ctx_) return;
    VkDevice dev = ctx_->getDevice();

    if (hasInFlightSecondSubmit && fence) {
        vkWaitForFences(dev, 1, &fence, VK_TRUE, UINT64_MAX);
        hasInFlightSecondSubmit = false;
    }

    // Destroy particle/grid buffers before releasing Vulkan objects
    particleBuffer.remove();
    gridBuffer.remove();
    kernelLutBuffer.destroy(dev);

    auto destroyPipe = [&](VkPipeline& p) {
        if (p) { vkDestroyPipeline(dev, p, nullptr); p = VK_NULL_HANDLE; }
    };
    destroyPipe(pipeClear);
    destroyPipe(pipeCount);
    destroyPipe(pipePrefix);
    destroyPipe(pipeScatter);
    destroyPipe(pipeDensity);
    destroyPipe(pipeForce);
    destroyPipe(pipeIntegrate);

    if (pipeLayout)  { vkDestroyPipelineLayout(dev, pipeLayout, nullptr);     pipeLayout  = VK_NULL_HANDLE; }
    if (descPool)    { vkDestroyDescriptorPool(dev, descPool, nullptr);        descPool    = VK_NULL_HANDLE; }
    if (descLayout)  { vkDestroyDescriptorSetLayout(dev, descLayout, nullptr); descLayout  = VK_NULL_HANDLE; }
    if (fence)       { vkDestroyFence(dev, fence, nullptr);                    fence       = VK_NULL_HANDLE; }
    if (cmdPool)     { vkDestroyCommandPool(dev, cmdPool, nullptr);            cmdPool     = VK_NULL_HANDLE; }

    ctx_  = nullptr;
    pool_ = nullptr;
}
