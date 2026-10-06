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
    queue_.submit(command);
}

std::vector<std::string> SoftBodyCommandDispatcher::collectResponses() {
    return queue_.collectResponses();
}

void SoftBodyCommandDispatcher::processQueue() {
    std::queue<std::string> local = queue_.takeAll();
    while (!local.empty()) {
        std::string resp = route(local.front());
        local.pop();
        if (!resp.empty()) queue_.respond(std::move(resp));
    }
}

std::optional<std::filesystem::path> SoftBodyCommandDispatcher::takePendingScreenshot() {
    std::optional<std::filesystem::path> p;
    p.swap(pendingScreenshot_);
    return p;
}

void SoftBodyCommandDispatcher::signalScreenshotDone(bool ok, const std::string& path) {
    queue_.respond(ok ? "OK" : "FAIL:" + path);
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

    if (sv.rfind("GetSoftParam:", 0) == 0) {
        const std::string_view name = sv.substr(13);
        const auto& sp = world_->getWorld().solverParams();
        const auto& wp = world_->getWorld().params();
        if      (name == "timeStep")               return std::to_string(sp.timeStep);
        else if (name == "numSubsteps")            return std::to_string(sp.numSubsteps);
        else if (name == "numIterations")          return std::to_string(sp.numIterations);
        else if (name == "gravityY")               return std::to_string(sp.gravity.y);
        else if (name == "selfCollisionEnabled")   return sp.selfCollisionEnabled ? "1" : "0";
        else if (name == "selfCollisionThickness") return std::to_string(sp.selfCollisionThickness);
        else if (name == "sphereEnabled")          return wp.sphereEnabled ? "1" : "0";
        else if (name == "sphereX")                return std::to_string(wp.sphereCenter.x);
        else if (name == "sphereY")                return std::to_string(wp.sphereCenter.y);
        else if (name == "sphereZ")                return std::to_string(wp.sphereCenter.z);
        else if (name == "sphereRadius")           return std::to_string(wp.sphereRadius);
        return "Error:unknown soft param '" + std::string(name) + "'";
    }

    // Solver / sphere-collider / self-collision parameters edited by the Soft Body panel.
    if (sv.rfind("SetSoftParam:", 0) == 0) {
        const std::string_view rest = sv.substr(13);
        const size_t comma = rest.find(',');
        if (comma == std::string_view::npos) return "Error:expected name,value";
        const std::string_view name = rest.substr(0, comma);
        const std::string_view val  = rest.substr(comma + 1);
        float f = 0.0f;
        if (!parseFlt(val.data(), val.data() + val.size(), f)) return "Error:bad value";
        auto& sp = world_->getWorld().solverParams();
        auto& wp = world_->getWorld().params();
        if      (name == "timeStep")                  sp.timeStep = f;
        else if (name == "numSubsteps")               sp.numSubsteps = static_cast<int>(f);
        else if (name == "numIterations")             sp.numIterations = static_cast<int>(f);
        else if (name == "gravityY")                  sp.gravity.y = f;
        else if (name == "selfCollisionEnabled")      sp.selfCollisionEnabled = f != 0.0f;
        else if (name == "selfCollisionThickness")    sp.selfCollisionThickness = f;
        else if (name == "sphereEnabled")             wp.sphereEnabled = f != 0.0f;
        else if (name == "sphereX")                   wp.sphereCenter.x = f;
        else if (name == "sphereY")                   wp.sphereCenter.y = f;
        else if (name == "sphereZ")                   wp.sphereCenter.z = f;
        else if (name == "sphereRadius")              wp.sphereRadius = f;
        else return "Error:unknown soft param '" + std::string(name) + "'";
        return "OK";
    }

    if (sv.rfind("SaveScreenshot:", 0) == 0) {
        pendingScreenshot_ = std::filesystem::path(cmd.substr(15));
        return {};
    }

    return "Error:Unknown:" + cmd;
}

} // namespace Phantom
