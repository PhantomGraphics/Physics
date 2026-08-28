#include "pch.h"
#include "RigidBodyCommandDispatcher.h"
#include "RigidBodyWorld.h"

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

std::vector<std::string_view> split(std::string_view sv, char delim) {
    std::vector<std::string_view> parts;
    size_t start = 0;
    while (true) {
        size_t pos = sv.find(delim, start);
        if (pos == std::string_view::npos) { parts.push_back(sv.substr(start)); break; }
        parts.push_back(sv.substr(start, pos - start));
        start = pos + 1;
    }
    return parts;
}

} // namespace

void RigidBodyCommandDispatcher::dispatch(const std::string& command) {
    std::lock_guard<std::mutex> lk(mutex_);
    inputQueue_.push(command);
}

std::vector<std::string> RigidBodyCommandDispatcher::collectResponses() {
    std::vector<std::string> out;
    std::lock_guard<std::mutex> lk(mutex_);
    while (!outputQueue_.empty()) {
        out.push_back(std::move(outputQueue_.front()));
        outputQueue_.pop();
    }
    return out;
}

void RigidBodyCommandDispatcher::processQueue() {
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

std::optional<std::filesystem::path> RigidBodyCommandDispatcher::takePendingScreenshot() {
    std::optional<std::filesystem::path> p;
    p.swap(pendingScreenshot_);
    return p;
}

void RigidBodyCommandDispatcher::signalScreenshotDone(bool ok, const std::string& path) {
    std::lock_guard<std::mutex> lk(mutex_);
    outputQueue_.push(ok ? "OK" : "FAIL:" + path);
}

std::string RigidBodyCommandDispatcher::route(const std::string& cmd) {
    if (!world_) return "Error:world not set";

    const std::string_view sv(cmd);

    if (cmd == "GetStatus")      return "OK";
    if (cmd == "GetBodyCount")   return "Count:" + std::to_string(world_->getWorld().getBodies().size());
    if (cmd == "GetContactCount") return "Contacts:" + std::to_string(world_->getWorld().getContacts().size());

    if (sv.rfind("GetBodyPositionY:", 0) == 0) {
        int index = -1;
        parseInt(cmd.data() + 17, cmd.data() + cmd.size(), index);
        const auto& bodies = world_->getWorld().getBodies();
        if (index < 0 || static_cast<size_t>(index) >= bodies.size())
            return "Error:BodyIndexOutOfRange:" + std::to_string(index);
        return std::to_string(bodies[index]->position.y);
    }

    if (sv.rfind("GetBodyVelocityY:", 0) == 0) {
        int index = -1;
        parseInt(cmd.data() + 17, cmd.data() + cmd.size(), index);
        const auto& bodies = world_->getWorld().getBodies();
        if (index < 0 || static_cast<size_t>(index) >= bodies.size())
            return "Error:BodyIndexOutOfRange:" + std::to_string(index);
        return std::to_string(bodies[index]->linearVelocity.y);
    }

    if (cmd == "Reset") {
        world_->reset();
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (cmd == "Step") {
        world_->stepForced();
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("Step:", 0) == 0) {
        int n = 1;
        parseInt(cmd.data() + 5, cmd.data() + cmd.size(), n);
        for (int i = 0; i < n; ++i) world_->stepForced();
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
        ScenePreset p;
        if      (name == "SphereDrop")        p = ScenePreset::SphereDrop;
        else if (name == "BoxDrop")            p = ScenePreset::BoxDrop;
        else if (name == "Stacking")           p = ScenePreset::Stacking;
        else if (name == "NewtonsCradle")      p = ScenePreset::NewtonsCradle;
        else if (name == "Billiards")          p = ScenePreset::Billiards;
        else if (name == "SphereBoxCollision") p = ScenePreset::SphereBoxCollision;
        // "Custom" is a real, explicit preset (an empty scene the caller then
        // populates via AddSphere/AddBox/AddFloor -- see scenarios/26_rigid_add_bodies.json),
        // accepted here same as any other named preset. Anything else is a typo,
        // not a scene: silently falling back to Custom would make a mistyped
        // preset name pass forever (see docs/todo/PLAN_physics_scenario_test_rebuild.md 1.4).
        else if (name == "Custom")             p = ScenePreset::Custom;
        else return "Error:unknown preset '" + std::string(name) + "'";
        world_->setPreset(p);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("SetGravity:", 0) == 0) {
        auto parts = split(sv.substr(11), ':');
        float x = 0.f, y = -9.8f, z = 0.f;
        if (parts.size() >= 3) {
            parseFlt(parts[0].data(), parts[0].data() + parts[0].size(), x);
            parseFlt(parts[1].data(), parts[1].data() + parts[1].size(), y);
            parseFlt(parts[2].data(), parts[2].data() + parts[2].size(), z);
        }
        world_->getWorld().params().gravity = {x, y, z};
        return "OK";
    }

    if (sv.rfind("SetTimeStep:", 0) == 0) {
        float dt = 0.016f;
        parseFlt(cmd.data() + 12, cmd.data() + cmd.size(), dt);
        world_->getWorld().timeStep = dt;
        return "OK";
    }

    if (sv.rfind("SetSolverIter:", 0) == 0) {
        int n = 10;
        parseInt(cmd.data() + 14, cmd.data() + cmd.size(), n);
        world_->getWorld().params().solverIterations = n;
        return "OK";
    }

    if (sv.rfind("AddSphere:", 0) == 0) {
        auto parts = split(sv.substr(10), ':');
        float px = 0.f, py = 3.f, pz = 0.f, r = 0.5f, mass = 1.f;
        if (parts.size() >= 5) {
            parseFlt(parts[0].data(), parts[0].data() + parts[0].size(), px);
            parseFlt(parts[1].data(), parts[1].data() + parts[1].size(), py);
            parseFlt(parts[2].data(), parts[2].data() + parts[2].size(), pz);
            parseFlt(parts[3].data(), parts[3].data() + parts[3].size(), r);
            parseFlt(parts[4].data(), parts[4].data() + parts[4].size(), mass);
        }
        world_->addSphere({px, py, pz}, r, mass);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("AddBox:", 0) == 0) {
        auto parts = split(sv.substr(7), ':');
        float px = 0.f, py = 3.f, pz = 0.f, hx = 0.5f, hy = 0.5f, hz = 0.5f, mass = 1.f;
        if (parts.size() >= 7) {
            parseFlt(parts[0].data(), parts[0].data() + parts[0].size(), px);
            parseFlt(parts[1].data(), parts[1].data() + parts[1].size(), py);
            parseFlt(parts[2].data(), parts[2].data() + parts[2].size(), pz);
            parseFlt(parts[3].data(), parts[3].data() + parts[3].size(), hx);
            parseFlt(parts[4].data(), parts[4].data() + parts[4].size(), hy);
            parseFlt(parts[5].data(), parts[5].data() + parts[5].size(), hz);
            parseFlt(parts[6].data(), parts[6].data() + parts[6].size(), mass);
        }
        world_->addBox({px, py, pz}, {hx, hy, hz}, mass);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("AddFloor:", 0) == 0) {
        float y = 0.f;
        parseFlt(cmd.data() + 9, cmd.data() + cmd.size(), y);
        world_->addFloor(y);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("SaveScreenshot:", 0) == 0) {
        pendingScreenshot_ = std::filesystem::path(cmd.substr(15));
        return {};
    }

    return "Error:Unknown:" + cmd;
}

} // namespace Phantom
