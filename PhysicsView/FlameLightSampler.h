#pragma once

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace Phantom {

/** Importance samples the actual emissive particles; never merges light sources.
 * Flux = blackbody radiance times projected disc area. A selected light carries
 * flux / selection probability, so the ensemble mean estimates the full sum.
 */
class FlameLightSampler {
public:
    struct Light { glm::vec3 position; glm::vec3 flux; float radius; };
    void clear() { lights_.clear(); cdf_.clear(); total_=0; }
    void add(const Light& light) {
        const double weight=0.2126*light.flux.r+0.7152*light.flux.g+0.0722*light.flux.b;
        if (!(weight>0) || !std::isfinite(weight) || !(light.radius>0)) return;
        total_+=weight; lights_.push_back(light); cdf_.push_back(total_);
    }
    Light sample(double uniform) const {
        if (lights_.empty()) return {{0,0,0},{0,0,0},0};
        const double target=std::clamp(uniform,0.0,std::nextafter(1.0,0.0))*total_;
        const size_t index=std::min(static_cast<size_t>(std::upper_bound(cdf_.begin(),cdf_.end(),target)-cdf_.begin()),lights_.size()-1);
        const double weight=cdf_[index]-(index?cdf_[index-1]:0);
        Light result=lights_[index]; result.flux*=static_cast<float>(total_/weight);
        return result;
    }
    bool empty() const { return lights_.empty(); }
    static uint32_t hash(uint32_t v) {
        const uint32_t state=v*747796405u+2891336453u;
        const uint32_t word=((state>>((state>>28u)+4u))^state)*277803737u;
        return (word>>22u)^word;
    }
    static double uniform(uint32_t seed) { return (double(hash(seed))+0.5)/4294967296.0; }
private:
    std::vector<Light> lights_;
    std::vector<double> cdf_;
    double total_=0;
};

} // namespace Phantom
