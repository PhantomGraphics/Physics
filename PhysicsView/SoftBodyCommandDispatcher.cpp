#include "pch.h"
#include "SoftBodyCommandDispatcher.h"
#include "SoftBodyWorld.h"

#include <charconv>

namespace Phantom {

namespace {

bool parseFlt(const char* begin, const char* end, float& out) {
    auto [ptr, ec] = std::from_chars(begin, end, out);
    return ec == std::errc{};
}

bool parseInt(const char* begin, const char* end, int& out) {
    auto [ptr, ec] = std::from_chars(begin, end, out);
    return ec == std::errc{};
}

} // namespace

void SoftBodyCommandDispatcher::dispatch(const std::string& command) {
    std::lock_guard<std::mutex> lk(mutex_);
    inputQueue_.push(command);
}

std::vector<std::string> SoftBodyCommandDispatcher::collectResponses() {
    std::vector<std::string> out;
    std::lock_guard<std::mutex> lk(mutex_);
    while (!outputQueue_.empty()) {
        out.push_back(std::move(outputQueue_.front()));
        outputQueue_.pop();
    }
    return out;
}

void SoftBodyCommandDispatcher::processQueue() {
    std::queue<std::string> local;
    { std::lock_guard<std::mutex> lk(mutex_); std::swap(local, inputQueue_); }
    while (!local.empty()) {
        std::string resp = route(local.front());
        local.pop();
        if (!resp.empty()) {
            std::lock_guard<std::mutex> lk(mutex_);
            outputQueue_.push(std::move(resp));
        }
    }
}

std::optional<std::filesystem::path> SoftBodyCommandDispatcher::takePendingScreenshot() {
    std::optional<std::filesystem::path> p;
    p.swap(pendingScreenshot_);
    return p;
}

void SoftBodyCommandDispatcher::signalScreenshotDone(bool ok, const std::string& path) {
    std::lock_guard<std::mutex> lk(mutex_);
    outputQueue_.push(ok ? "OK" : "FAIL:" + path);
}

std::string SoftBodyCommandDispatcher::route(const std::string& cmd) {
    if (!world_) return "Error:world not set";

    const std::string_view sv(cmd);

    if (cmd == "GetStatus")
        return "OK";

    if (cmd == "GetSoftBodyCount")
        return "Count:" + std::to_string(world_->getWorld().getBodyCount());

    if (cmd == "GetSoftParticleCount")
        return "Particles:" + std::to_string(world_->getWorld().getParticleCount());

    if (cmd == "GetMaxSpeed")
        return std::to_string(world_->getWorld().getMaxSpeed());

    if (sv.rfind("GetSoftParticlePositionY:", 0) == 0) {
        auto rest = sv.substr(25);
        auto colon = rest.find(':');
        if (colon == std::string_view::npos) return "Error:missing particle index";
        int bodyIdx = 0, particleIdx = 0;
        if (!parseInt(rest.data(), rest.data() + colon, bodyIdx)) return "Error:bad body index";
        if (!parseInt(rest.data() + colon + 1, rest.data() + rest.size(), particleIdx)) return "Error:bad particle index";
        return std::to_string(world_->getParticlePositionY(bodyIdx, particleIdx));
    }

    if (sv.rfind("GetSoftTotalVolume:", 0) == 0) {
        int bodyIdx = 0;
        if (!parseInt(cmd.data() + 19, cmd.data() + cmd.size(), bodyIdx)) return "Error:bad body index";
        return std::to_string(world_->getBodyTotalVolume(bodyIdx));
    }

    if (sv.rfind("GetSoftMinInterBodyDistance:", 0) == 0) {
        auto rest = sv.substr(28);
        auto colon = rest.find(':');
        if (colon == std::string_view::npos) return "Error:missing second body index";
        int idxA = 0, idxB = 0;
        if (!parseInt(rest.data(), rest.data() + colon, idxA)) return "Error:bad body index A";
        if (!parseInt(rest.data() + colon + 1, rest.data() + rest.size(), idxB)) return "Error:bad body index B";
        return std::to_string(world_->getMinInterBodyDistance(idxA, idxB));
    }

    if (sv.rfind("GetSoftMinNonEdgeDistance:", 0) == 0) {
        int bodyIdx = 0;
        if (!parseInt(cmd.data() + 26, cmd.data() + cmd.size(), bodyIdx)) return "Error:bad body index";
        return std::to_string(world_->getMinNonEdgeDistance(bodyIdx));
    }

    if (sv.rfind("SetCrossBodyCollisionEnabled:", 0) == 0) {
        std::string_view val = sv.substr(29);
        auto& sp = world_->getWorld().params();
        sp.crossBodyCollisionEnabled = (val == "true" || val == "1");
        sp.crossBodyCollisionThickness = 0.05f;
        sp.crossBodyCollisionCellSize  = 0.1f;
        return "OK";
    }

    if (cmd == "Reset") {
        world_->setPreset(world_->currentPreset());
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (cmd == "Step") {
        world_->getWorld().setRunning(true);
        world_->step();
        world_->getWorld().setRunning(false);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("Step:", 0) == 0) {
        int n = 1;
        parseInt(cmd.data() + 5, cmd.data() + cmd.size(), n);
        world_->getWorld().setRunning(true);
        for (int i = 0; i < n; ++i) world_->step();
        world_->getWorld().setRunning(false);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("SetRunning:", 0) == 0) {
        std::string_view val = sv.substr(11);
        world_->setRunning(val == "true" || val == "1");
        return "OK";
    }

    if (sv.rfind("SetPreset:", 0) == 0) {
        std::string_view name = sv.substr(10);
        SoftBodyPreset p;
        if      (name == "ClothTwoPin")    p = SoftBodyPreset::ClothTwoPin;
        else if (name == "ClothTopEdge")   p = SoftBodyPreset::ClothTopEdge;
        else if (name == "ClothWithSphere") p = SoftBodyPreset::ClothWithSphere;
        else if (name == "ClothOnBox")     p = SoftBodyPreset::ClothOnBox;
        else if (name == "JellySelfOverlap")  p = SoftBodyPreset::JellySelfOverlap;
        else if (name == "RopeHanging")    p = SoftBodyPreset::RopeHanging;
        else if (name == "RopePendulum")   p = SoftBodyPreset::RopePendulum;
        else if (name == "RopeBothEndsPinned") p = SoftBodyPreset::RopeBothEndsPinned;
        else if (name == "JellyDrop")      p = SoftBodyPreset::JellyDrop;
        else if (name == "JellyOnBox")     p = SoftBodyPreset::JellyOnBox;
        else if (name == "TwoJelliesStacked") p = SoftBodyPreset::TwoJelliesStacked;
        else if (name == "Mixed")          p = SoftBodyPreset::Mixed;
        // Unlike rigid-body's ScenePreset, SoftBodyPreset has no "Custom" escape
        // hatch -- every name must be a real preset (see internal design notes 1.4).
        else return "Error:unknown preset '" + std::string(name) + "'";
        world_->setPreset(p);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("SetSelfCollision:", 0) == 0) {
        std::string_view val = sv.substr(17);
        world_->getWorld().solverParams().selfCollisionEnabled = (val == "true" || val == "1");
        return "OK";
    }

    if (sv.rfind("SaveScreenshot:", 0) == 0) {
        pendingScreenshot_ = std::filesystem::path(cmd.substr(15));
        return {};
    }

    return "Error:Unknown:" + cmd;
}

} // namespace Phantom
