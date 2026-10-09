#pragma once
#include <functional>
#include <optional>
#include <string>

namespace Phantom {
class HairWorld;
// nullopt means not a hair command. Invalid hair commands return Error:.
// HairPreset:Single|Bundle|Body|HeadShake|StrongWind|LongHair|ShortFur,
// HairClear, HairReset, HairStep[:1..1000],
// SetHairRunning:true|false, IsHairRunning, SetHairPage:true|false, IsHairPage,
// SetHairParam:name,value, GetHairParam:name, GetHairStat:name.
// SetHairGenerationParam:name,value / GetHairGenerationParam:name stage
// rootJitter, shapeVariation, lengthVariation and seed for the next preset.
// Also longHairGuides/longHairFollowers/shortFurGuides/shortFurFollowers:
// guides 1..10000, followers 0..10000, independent of each other.
class HairCommandDispatcher {
public:
    void setWorld(HairWorld* world) { world_ = world; }
    void setPageHooks(std::function<void(bool)> set, std::function<bool()> get) {
        setPage_ = std::move(set); isPage_ = std::move(get);
    }
    void setOnChanged(std::function<void()> fn) { changed_ = std::move(fn); }
    std::optional<std::string> route(const std::string& cmd);
private:
    HairWorld* world_ = nullptr;
    std::function<void(bool)> setPage_;
    std::function<bool()> isPage_;
    std::function<void()> changed_;
};
} // namespace Phantom
