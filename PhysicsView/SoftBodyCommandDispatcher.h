#pragma once

#include "CGLib/VkAppBase/ScenarioRunner/IScenarioDispatcher.h"

#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <vector>

namespace Phantom {

class SoftBodyWorld;

class SoftBodyCommandDispatcher : public IScenarioDispatcher {
public:
    void setWorld(SoftBodyWorld* w)              { world_ = w; }
    void setOnWorldChanged(std::function<void()> fn)  { onWorldChanged_ = std::move(fn); }

    void processQueue();

    void dispatch(const std::string& command) override;
    std::vector<std::string> collectResponses() override;

    std::optional<std::filesystem::path> takePendingScreenshot();
    void signalScreenshotDone(bool ok, const std::string& path);

private:
    std::string route(const std::string& cmd);

    SoftBodyWorld*    world_          = nullptr;
    std::function<void()>  onWorldChanged_;

    std::optional<std::filesystem::path> pendingScreenshot_;

    std::mutex              mutex_;
    std::queue<std::string> inputQueue_;
    std::queue<std::string> outputQueue_;
};

} // namespace Phantom
