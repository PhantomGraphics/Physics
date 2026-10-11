#include "pch.h"
#include "RigidBodyCommandDispatcher.h"
#include "RigidBodyWorld.h"

#include "CommandParse.h"

namespace Phantom {

namespace {

using CommandParse::parseFloat;
using CommandParse::parseFloats;
using CommandParse::parseInt;
using CommandParse::parseIntIn;
using CommandParse::split;

// Upper bound for a single "Step:N" so a typo cannot hang the viewer.
constexpr int kMaxStepCount = 1000000;

} // namespace

void RigidBodyCommandDispatcher::dispatch(const std::string& command) {
    queue_.submit(command);
}

std::vector<std::string> RigidBodyCommandDispatcher::collectResponses() {
    return queue_.collectResponses();
}

void RigidBodyCommandDispatcher::processQueue() {
    std::queue<std::string> local = queue_.takeAll();
    while (!local.empty()) {
        std::string resp = route(local.front());
        local.pop();
        if (!resp.empty()) queue_.respond(std::move(resp));
    }
}

std::optional<std::filesystem::path> RigidBodyCommandDispatcher::takePendingScreenshot() {
    std::optional<std::filesystem::path> p;
    p.swap(pendingScreenshot_);
    return p;
}

void RigidBodyCommandDispatcher::signalScreenshotDone(bool ok, const std::string& path) {
    queue_.respond(ok ? "OK" : "FAIL:" + path);
}

std::string RigidBodyCommandDispatcher::route(const std::string& cmd) {
    if (!world_) return "Error:world not set";

    const std::string_view sv(cmd);

    if (cmd == "GetStatus")      return "OK";
    if (cmd == "GetBodyCount")   return "Count:" + std::to_string(world_->getWorld().getBodies().size());
    if (cmd == "GetContactCount") return "Contacts:" + std::to_string(world_->getWorld().getContacts().size());

    if (sv.rfind("GetBodyPositionY:", 0) == 0) {
        int index = -1;
        if (!parseInt(sv.substr(17), index)) return "Error:bad body index";
        const auto& bodies = world_->getWorld().getBodies();
        if (index < 0 || static_cast<size_t>(index) >= bodies.size())
            return "Error:BodyIndexOutOfRange:" + std::to_string(index);
        return std::to_string(bodies[index]->position.y);
    }

    if (sv.rfind("GetBodyVelocityY:", 0) == 0) {
        int index = -1;
        if (!parseInt(sv.substr(17), index)) return "Error:bad body index";
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
        if (!parseIntIn(sv.substr(5), 0, kMaxStepCount, n)) return "Error:bad step count";
        for (int i = 0; i < n; ++i) world_->stepForced();
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("SetRunning:", 0) == 0) {
        bool on = false;
        if (!CommandParse::parseBool(sv.substr(11), on)) return "Error:expected true|false|1|0";
        world_->setRunning(on);
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
        // preset name pass forever (see internal design notes 1.4).
        else if (name == "Custom")             p = ScenePreset::Custom;
        else return "Error:unknown preset '" + std::string(name) + "'";
        world_->setPreset(p);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("SetGravity:", 0) == 0) {
        float g[3];
        if (!parseFloats(split(sv.substr(11), ':'), 3, g)) return "Error:expected x:y:z";
        world_->getWorld().params().gravity = {g[0], g[1], g[2]};
        return "OK";
    }

    if (sv.rfind("SetTimeStep:", 0) == 0) {
        float dt = 0.0f;
        if (!parseFloat(sv.substr(12), dt) || dt <= 0.0f) return "Error:timeStep must be positive";
        world_->getWorld().timeStep = dt;
        return "OK";
    }

    if (sv.rfind("SetSolverIter:", 0) == 0) {
        int n = 0;
        if (!parseIntIn(sv.substr(14), 1, 1000, n)) return "Error:expected an integer in [1, 1000]";
        world_->getWorld().params().solverIterations = n;
        return "OK";
    }

    if (sv == "GetBaumgarteBeta") return std::to_string(world_->getWorld().params().baumgarteBeta);

    if (sv.rfind("SetBaumgarteBeta:", 0) == 0) {
        float v = 0.0f;
        if (!parseFloat(sv.substr(17), v)) return "Error:bad float";
        world_->getWorld().params().baumgarteBeta = v;
        return "OK";
    }

    if (sv.rfind("AddSphere:", 0) == 0) {
        float v[5];  // px, py, pz, radius, mass
        if (!parseFloats(split(sv.substr(10), ':'), 5, v)) return "Error:expected px:py:pz:radius:mass";
        if (v[3] <= 0.0f) return "Error:radius must be positive";
        if (v[4] < 0.0f) return "Error:mass must be non-negative (0 = static)";
        world_->addSphere({v[0], v[1], v[2]}, v[3], v[4]);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("AddBox:", 0) == 0) {
        float v[7];  // px, py, pz, hx, hy, hz, mass
        if (!parseFloats(split(sv.substr(7), ':'), 7, v)) return "Error:expected px:py:pz:hx:hy:hz:mass";
        if (v[3] <= 0.0f || v[4] <= 0.0f || v[5] <= 0.0f) return "Error:half extents must be positive";
        if (v[6] < 0.0f) return "Error:mass must be non-negative (0 = static)";
        world_->addBox({v[0], v[1], v[2]}, {v[3], v[4], v[5]}, v[6]);
        if (onWorldChanged_) onWorldChanged_();
        return "OK";
    }

    if (sv.rfind("AddFloor:", 0) == 0) {
        float y = 0.f;
        if (!parseFloat(sv.substr(9), y)) return "Error:bad float";
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
