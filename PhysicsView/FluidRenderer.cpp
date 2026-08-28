#include "pch.h"
#include "FluidRenderer.h"

#include "../../CGLib/VulkanGraphics/VulkanContext.h"
#include "../../CGLib/VulkanGraphics/VulkanCommandPool.h"

namespace Phantom {

VkVertexInputBindingDescription VkFluidVertex::getBindingDescription()
{
    VkVertexInputBindingDescription bd{};
    bd.binding = 0;
    bd.stride = sizeof(float) * 4;
    bd.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bd;
}

std::array<VkVertexInputAttributeDescription, 1> VkFluidVertex::getAttributeDescriptions()
{
    std::array<VkVertexInputAttributeDescription, 1> attrs{};
    attrs[0] = { 0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(VkFluidVertex, position) };
    return attrs;
}

void FluidRenderer::setParticles(const std::vector<glm::vec3>& positions)
{
    std::lock_guard<std::mutex> lock(mutex_);
    pendingPositions_ = positions;
    dirty_ = true;
}

void FluidRenderer::setDirectGpuBuffer(VkBuffer buf, uint32_t count)
{
    std::lock_guard<std::mutex> lock(mutex_);
    directVertexBuffer_ = buf;
    directPointCount_ = count;
}

void FluidRenderer::clearDirectGpuBuffer()
{
    std::lock_guard<std::mutex> lock(mutex_);
    directVertexBuffer_ = VK_NULL_HANDLE;
    directPointCount_ = 0;
}

void FluidRenderer::handleMouseButton(bool pressed, float x, float y)
{
    mouseDown_ = pressed;
    lastMouse_ = glm::vec2(x, y);
}

void FluidRenderer::handleMouseMove(float x, float y)
{
    if (mouseDown_) {
        const float dx = (x - lastMouse_.x) * 0.005f;
        const float dy = (y - lastMouse_.y) * 0.005f;
        yaw_ += dx;
        pitch_ = std::max(0.05f, std::min(3.09f, pitch_ + dy));
    }
    lastMouse_ = glm::vec2(x, y);
}

void FluidRenderer::handleScroll(float dy)
{
    distance_ = std::max(5.0f, distance_ - dy * 2.0f);
}

void FluidRenderer::onInit(Phantom::VKG::VulkanContext& ctx,
                             const Phantom::VKG::VulkanCommandPool& pool,
                             VkRenderPass renderPass,
                             uint32_t framesInFlight)
{
    ctx_ = &ctx;
    pool_ = &pool;

    FluidPipeline::Config cfg;
    cfg.vertSpv = std::move(shaders_.vertSpv);
    cfg.fragSpv = std::move(shaders_.fragSpv);
    cfg.bindingDesc = VkFluidVertex::getBindingDescription();
    auto attrs = VkFluidVertex::getAttributeDescriptions();
    cfg.attrDescs = { attrs[0] };
    cfg.framesInFlight = framesInFlight;
    pipeline_.create(ctx, renderPass, cfg);
}

void FluidRenderer::onUpdate(uint32_t frameIndex)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (directVertexBuffer_ != VK_NULL_HANDLE) {
            pointCount_ = directPointCount_;
            const glm::mat4 mvp = computeMVP();
            pipeline_.updateUBO(frameIndex, mvp);
            return;
        }
    }

    std::vector<glm::vec3> uploadData;
    bool hasNewData = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (dirty_) {
            uploadData = pendingPositions_;
            dirty_ = false;
            hasNewData = true;
        }
    }

    if (hasNewData) {
        vkDeviceWaitIdle(ctx_->getDevice());
        vertexBuffer_.destroy(ctx_->getDevice());
        uploadVertices(uploadData);
    }

    const glm::mat4 mvp = computeMVP();
    pipeline_.updateUBO(frameIndex, mvp);
}

void FluidRenderer::onRender(VkCommandBuffer cmd, uint32_t frameIndex)
{
    if (!enabled_ || pointCount_ == 0) {
        return;
    }

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.getPipeline());

    VkBuffer drawBuffer = vertexBuffer_.get();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (directVertexBuffer_ != VK_NULL_HANDLE) {
            drawBuffer = directVertexBuffer_;
        }
    }

    VkBuffer vbufs[] = { drawBuffer };
    VkDeviceSize offsets[] = { 0 };
    vkCmdBindVertexBuffers(cmd, 0, 1, vbufs, offsets);

    VkDescriptorSet ds = pipeline_.getDescriptorSet(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                            pipeline_.getLayout(), 0, 1, &ds, 0, nullptr);

    vkCmdDraw(cmd, pointCount_, 1, 0, 0);
}

void FluidRenderer::onCleanup(VkDevice device)
{
    vertexBuffer_.destroy(device);
    pipeline_.destroy(device);
}

void FluidRenderer::onImGui()
{
    if (ImGui::CollapsingHeader("Camera")) {
        ImGui::SliderFloat("Yaw", &yaw_, -3.14159f, 3.14159f);
        ImGui::SliderFloat("Pitch", &pitch_, 0.05f, 3.09f);
        ImGui::SliderFloat("Distance", &distance_, 5.0f, 300.0f);
    }
}

void FluidRenderer::uploadVertices(const std::vector<glm::vec3>& pts)
{
    pointCount_ = static_cast<uint32_t>(pts.size());
    if (pointCount_ == 0) {
        return;
    }

    std::vector<VkFluidVertex> vertices;
    vertices.reserve(pts.size());
    for (const auto& p : pts) {
        vertices.push_back({ glm::vec4(p, 1.0f) });
    }

    vertexBuffer_.create(*ctx_, *pool_,
                         sizeof(VkFluidVertex) * vertices.size(),
                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                         vertices.data());
}

glm::mat4 FluidRenderer::computeMVP() const
{
    return getProjMatrix() * getViewMatrix();
}

glm::mat4 FluidRenderer::getViewMatrix() const
{
    const float x = distance_ * sinf(pitch_) * cosf(yaw_);
    const float y = distance_ * cosf(pitch_);
    const float z = distance_ * sinf(pitch_) * sinf(yaw_);

    glm::vec3 center(20.f, 20.f, 20.f);
    glm::vec3 eye = center + glm::vec3(x, y, z);

    return glm::lookAt(eye, center, glm::vec3(0.f, 1.f, 0.f));
}

glm::mat4 FluidRenderer::getProjMatrix() const
{
    float aspect = extent_.height > 0
        ? static_cast<float>(extent_.width) / static_cast<float>(extent_.height)
        : 1.0f;

    glm::mat4 proj = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 1000.0f);
    proj[1][1] *= -1.0f;

    return proj;
}

} // namespace Phantom
