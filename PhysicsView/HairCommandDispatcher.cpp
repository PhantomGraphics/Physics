#include "HairCommandDispatcher.h"
#include "HairWorld.h"
#include <charconv>
#include <cmath>
#include <cstdio>
#include <string_view>

namespace Phantom {
namespace {
std::string number(double value) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.9g", value);
    return buf;
}
bool parse(std::string_view text, double& value) {
    const auto result = std::from_chars(text.data(), text.data()+text.size(), value);
    return result.ec == std::errc{} && result.ptr == text.data()+text.size() && std::isfinite(value);
}
bool param(Physics::HairSolver::Params& p, std::string_view name, double& value, bool set) {
    // Explicit branches keep the integer/boolean validation shared by UI and CLI.
    if (name == "substeps" || name == "iterations") {
        if (set && (value != std::floor(value) || value < 1 || value > 128)) return false;
        int& field = name == "substeps" ? p.numSubsteps : p.numIterations;
        if (set) field = static_cast<int>(value); else value = field;
        return true;
    }
    if (name == "shapeEnabled") {
        if (set && value != 0 && value != 1) return false;
        if (set) p.shapeEnabled = value != 0; else value = p.shapeEnabled ? 1 : 0;
        return true;
    }
    float* field = nullptr;
    if (name == "timeStep") field = &p.timeStep;
    else if (name == "stretchCompliance") field = &p.stretchCompliance;
    else if (name == "bendCompliance") field = &p.bendCompliance;
    else if (name == "shapeCompliance") field = &p.shapeCompliance;
    else if (name == "dampingRate") field = &p.dampingRate;
    else if (name == "gravityX") field = &p.gravity.x;
    else if (name == "gravityY") field = &p.gravity.y;
    else if (name == "gravityZ") field = &p.gravity.z;
    else if (name == "windX") field = &p.windVelocity.x;
    else if (name == "windY") field = &p.windVelocity.y;
    else if (name == "windZ") field = &p.windVelocity.z;
    else if (name == "windDrag") field = &p.windDrag;
    else if (name == "collisionRadius") field = &p.collisionRadius;
    else if (name == "teleportDistance") field = &p.teleportDistance;
    else if (name == "teleportAngle") field = &p.teleportAngle;
    if (!field) return false;
    if (set) *field = static_cast<float>(value); else value = *field;
    return true;
}
}
std::optional<std::string> HairCommandDispatcher::route(const std::string& cmd) {
    const std::string_view text(cmd);
    const auto colon = text.find(':');
    const auto verb = text.substr(0, colon);
    const auto arg = colon == std::string_view::npos ? std::string_view{} : text.substr(colon+1);
    if (verb != "HairPreset" && verb != "HairClear" && verb != "HairReset" && verb != "HairStep" &&
        verb != "SetHairRunning" && verb != "IsHairRunning" && verb != "SetHairPage" &&
        verb != "IsHairPage" && verb != "SetHairParam" && verb != "GetHairParam" && verb != "GetHairStat" &&
        verb != "SetHairRootPose" && verb != "HairTeleport" && verb != "SetHairMotion" &&
        verb != "IsHairMotion" && verb != "SetHairFriction" &&
        verb != "SetHairGenerationParam" && verb != "GetHairGenerationParam")
        return std::nullopt;
    if (!world_) return "Error: hair world unavailable";
    auto& w = *world_;
    if (verb == "SetHairGenerationParam" || verb == "GetHairGenerationParam") {
        const bool set = verb == "SetHairGenerationParam";
        const auto comma = arg.find(',');
        const auto name = set ? arg.substr(0,comma) : arg;
        double value = 0.;
        if (set && (comma == std::string_view::npos || !parse(arg.substr(comma+1),value)))
            return "Error: expected name,finite value";
        auto p = w.generationParams();
        if (name == "seed") {
            if (set && (value != std::floor(value) || value < 0. || value > 4294967295.))
                return "Error: invalid hair generation seed";
            if (set) p.seed = static_cast<uint32_t>(value); else return std::to_string(p.seed);
        } else {
            float* field = name == "rootJitter" ? &p.rootJitter :
                           name == "shapeVariation" ? &p.shapeVariation :
                           name == "lengthVariation" ? &p.lengthVariation : nullptr;
            if (!field) return "Error: unknown hair generation parameter";
            if (set) *field = static_cast<float>(value); else value = *field;
        }
        if (!set) return number(value);
        return w.setGenerationParams(p) ? "OK" : "Error: invalid hair generation parameter";
    }
    if (verb == "SetHairRootPose" || verb == "HairTeleport") {
        double values[7];
        auto remaining = arg;
        for (int i = 0; i < 7; ++i) {
            const auto comma = remaining.find(',');
            if ((i < 6 && comma == std::string_view::npos) || (i == 6 && comma != std::string_view::npos) ||
                !parse(remaining.substr(0, comma), values[i])) return "Error: expected x,y,z,qw,qx,qy,qz";
            if (i < 6) remaining.remove_prefix(comma+1);
        }
        Physics::HairRootPose pose;
        pose.position = Math::Vector3df(values[0], values[1], values[2]);
        pose.rotation = Math::Quaternion(static_cast<float>(values[3]), static_cast<float>(values[4]),
            static_cast<float>(values[5]), static_cast<float>(values[6]));
        if (!w.setRigPose(pose, verb == "HairTeleport")) return "Error: invalid hair root pose";
        if (changed_) changed_();
        return "OK";
    }
    if (verb == "SetHairFriction") {
        double value;
        if (!parse(arg, value) || !w.setFriction(static_cast<float>(value))) return "Error: invalid hair friction";
        return "OK";
    }
    if (verb == "GetHairStat") {
        const auto& s = w.stats();
        if (arg == "guides") return number(static_cast<double>(w.strands().strandCount()));
        if (arg == "particles") return number(static_cast<double>(s.particleCount));
        if (arg == "followers") return number(static_cast<double>(w.followers().strandCount()));
        if (arg == "followerVertices") return number(static_cast<double>(w.followers().particleCount()));
        if (arg == "followerTipX") return number(w.followers().positions().empty() ? 0.f : w.followers().positions().back().x);
        if (arg == "followerTipY") return number(w.followers().positions().empty() ? 0.f : w.followers().positions().back().y);
        if (arg == "time") return number(s.simulatedTime);
        if (arg == "steps") return number(static_cast<double>(s.steps));
        if (arg == "maxSpeed") return number(s.maxSpeed);
        if (arg == "lengthError") return number(s.maxRelativeLengthError);
        if (arg == "nonFinite") return number(static_cast<double>(s.nonFiniteCount));
        if (arg == "rootError") return number(w.maxRootError());
        if (arg == "tipY") return number(w.tipY());
        if (arg == "tipX") return number(w.tipX());
        if (arg == "colliders") return number(static_cast<double>(w.colliderCount()));
        if (arg == "penetration") return number(s.maxPenetration);
        if (arg == "pinnedPenetration") return number(s.pinnedPenetration);
        if (arg == "rootResets") return number(static_cast<double>(s.rootResets));
        if (arg == "droppedTime") return number(w.droppedTime());
        return "Error: unknown hair statistic";
    }
    if (verb == "SetHairParam" || verb == "GetHairParam") {
        const bool set = verb == "SetHairParam";
        const auto comma = arg.find(',');
        double value = 0.;
        if (set && (comma == std::string_view::npos || !parse(arg.substr(comma+1), value)))
            return "Error: expected name,finite value";
        auto p = w.params();
        if (!param(p, set ? arg.substr(0, comma) : arg, value, set)) return "Error: invalid hair parameter";
        if (!set) return number(value);
        return w.setParams(p) ? "OK" : "Error: invalid hair parameter value";
    }
    if (verb == "SetHairPage" || verb == "SetHairRunning" || verb == "SetHairMotion") {
        if (arg != "true" && arg != "false" && arg != "1" && arg != "0") return "Error: expected boolean";
        const bool on = arg == "true" || arg == "1";
        if (verb == "SetHairPage") {
            if (!setPage_) return "Error: hair page unavailable";
            setPage_(on);
        } else if (verb == "SetHairMotion") w.setMotion(on);
        else w.setRunning(on);
        return "OK";
    }
    if (colon != std::string_view::npos && verb != "HairPreset" && verb != "HairStep")
        return "Error: unexpected hair command argument";
    if (verb == "IsHairPage") return isPage_ && isPage_() ? "true" : "false";
    if (verb == "IsHairRunning") return w.isRunning() ? "true" : "false";
    if (verb == "IsHairMotion") return w.motionEnabled() ? "true" : "false";
    if (verb == "HairPreset") {
        HairPreset preset;
        if (arg == "Single") preset = HairPreset::Single;
        else if (arg == "Bundle") preset = HairPreset::Bundle;
        else if (arg == "Body") preset = HairPreset::Body;
        else if (arg == "HeadShake") preset = HairPreset::HeadShake;
        else if (arg == "StrongWind") preset = HairPreset::StrongWind;
        else if (arg == "LongHair") preset = HairPreset::LongHair;
        else if (arg == "ShortFur") preset = HairPreset::ShortFur;
        else return "Error: unknown hair preset";
        if (!w.setPreset(preset)) return "Error: hair generation failed";
    } else if (verb == "HairClear") w.clear();
    else if (verb == "HairReset") w.reset();
    else if (verb == "HairStep") {
        double n = 1.;
        if (colon != std::string_view::npos && (!parse(arg, n) || n != std::floor(n) || n < 1 || n > 1000))
            return "Error: hair steps must be integer 1..1000";
        for (int i = 0; i < static_cast<int>(n); ++i) if (!w.stepOnce()) return "Error: hair step failed";
    }
    if (changed_) changed_();
    return "OK";
}
} // namespace Phantom
