#include "pch.h"
#include "CombustibleBody.h"
#include <algorithm>
#include <cmath>
#include <cstring>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

namespace {
bool finite(const Vector3df& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
constexpr float pi = 3.14159265359f;
}

bool CombustibleMaterial::valid() const
{
    return std::isfinite(density) && density > 0 && std::isfinite(specificHeat) && specificHeat > 0 &&
        std::isfinite(conductivity) && conductivity >= 0 && std::isfinite(pyrolysisTemperature) && pyrolysisTemperature > 0 &&
        std::isfinite(pyrolysisRate) && pyrolysisRate >= 0 && std::isfinite(latentHeat) && latentHeat >= 0 &&
        std::isfinite(residueFraction) && residueFraction >= 0 && residueFraction <= 1;
}

CombustibleMaterial CombustibleMaterial::preset(int index)
{
    CombustibleMaterial m;
    if (index == 0) { m.pyrolysisTemperature = 500; m.pyrolysisRate = 1.2f; }
    if (index == 2) { m.pyrolysisTemperature = 950; m.pyrolysisRate = 0.3f; }
    if (index == 3) m.combustible = false;
    return m;
}

const char* Phantom::Physics::combustionStateName(CombustionState s)
{
    const char* names[] = {"Unburned", "Heating", "Burning", "Extinguished", "Exhausted"};
    return names[static_cast<int>(s)];
}

bool CombustibleBody::initialize(uint64_t id, Shape shape, const Vector3df& center,
    const Vector3df& extent, int n, double fuelMass, const CombustibleMaterial& material)
{
    if (!id || !finite(center) || !finite(extent) || glm::min(extent.x, glm::min(extent.y, extent.z)) <= 0 ||
        n < 1 || n > 16 || !std::isfinite(fuelMass) || fuelMass <= 0 || !material.valid()) return false;
    id_ = id; shape_ = shape; center_ = center; halfExtent_ = extent; material_ = material;
    smoothingLength_ = 0; resolution_ = n;
    if (shape == Shape::Sphere) halfExtent_ = Vector3df(extent.x);
    samples_.clear();
    const float volume = shape == Shape::Box ? 8 * extent.x * extent.y * extent.z : 4 * pi * extent.x * extent.x * extent.x / 3;
    if (shape == Shape::Box) {
        for (int axis = 0; axis < 3; ++axis) for (int sign : {-1, 1}) {
            const int u = (axis + 1) % 3, v = (axis + 2) % 3;
            for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) {
                CombustibleSample s;
                s.position[axis] = sign * extent[axis]; s.normal[axis] = static_cast<float>(sign);
                s.position[u] = extent[u] * (2 * (i + 0.5f) / n - 1);
                s.position[v] = extent[v] * (2 * (j + 0.5f) / n - 1);
                s.area = 4 * extent[u] * extent[v] / (n * n);
                samples_.push_back(s);
            }
        }
    } else {
        // Equal-area latitude bands (uniform cos(theta)); no singular pole cells.
        for (int i = 0; i < 2 * n; ++i) for (int j = 0; j < 4 * n; ++j) {
            const float y = 1 - (i + 0.5f) / n, phi = 2 * pi * (j + 0.5f) / (4 * n);
            CombustibleSample s;
            s.normal = Vector3df(std::sqrt(1 - y*y) * std::cos(phi), y, std::sqrt(1-y*y) * std::sin(phi));
            s.position = s.normal * extent.x; s.area = 4 * pi * extent.x * extent.x / (8 * n * n);
            samples_.push_back(s);
        }
    }
    double totalArea = 0;
    for (const auto& s : samples_) totalArea += s.area;
    for (auto& s : samples_) {
        s.volume = static_cast<float>(volume * s.area / totalArea);
        s.initialFuel = s.fuel = fuelMass * s.area / totalArea;
    }
    // Symmetric local graph: include closest cells, then symmetrize. Distance
    // and area (not degree) determine conductance in SolidCombustionSolver.
    for (size_t i = 0; i < samples_.size(); ++i) {
        std::vector<std::pair<float, size_t>> distances;
        for (size_t j = 0; j < samples_.size(); ++j) if (i != j)
            distances.emplace_back(glm::length(samples_[i].position - samples_[j].position), j);
        std::sort(distances.begin(), distances.end());
        for (size_t k = 0; k < std::min(size_t(4), distances.size()); ++k) {
            const size_t j = distances[k].second;
            auto add = [](std::vector<size_t>& a, size_t x) { if (std::find(a.begin(), a.end(), x) == a.end()) a.push_back(x); };
            add(samples_[i].neighbors, j); add(samples_[j].neighbors, i);
        }
    }
    pyrolyzed = emitted = heatExchange = 0; reactionRate = 0; firstIgnitionTime = -1; wasBurning = false;
    return true;
}

bool CombustibleBody::initializeParticles(uint64_t id, Shape shape, const Vector3df& center,
    const Vector3df& extent, int n, double fuelMass, const CombustibleMaterial& material)
{
    if (!initialize(id,shape,center,extent,n,fuelMass,material)) return false;
    samples_.clear();
    const auto e=halfExtent_;
    float spacing=2*std::min(e.x,std::min(e.y,e.z))/std::max(3,2*n);
    double counts[]={std::ceil(2.0*e.x/spacing),std::ceil(2.0*e.y/spacing),std::ceil(2.0*e.z/spacing)};
    // Thin plates require an anisotropic lattice rather than forcing their
    // smallest dimension's spacing onto both large dimensions.
    if(counts[0]*counts[1]*counts[2]>4096) {
        spacing=2*std::max(e.x,std::max(e.y,e.z))/std::max(3,2*n);
        for(int k=0;k<3;++k) counts[k]=std::max(3.0,std::ceil(2.0*e[k]/spacing));
    }
    // Reject extreme aspect ratios before allocation; caller retains no body.
    if (!std::isfinite(spacing) || spacing<=0 || !std::isfinite(counts[0]*counts[1]*counts[2]) ||
        counts[0]*counts[1]*counts[2]>4096) return false;
    const int nx=static_cast<int>(counts[0]),ny=static_cast<int>(counts[1]),nz=static_cast<int>(counts[2]);
    const Vector3df cell=2.0f*e/Vector3df(nx,ny,nz);
    smoothingLength_=1.6f*glm::length(cell)/std::sqrt(3.0f);
    double areaSum=0;
    for(int x=0;x<nx;++x) for(int y=0;y<ny;++y) for(int z=0;z<nz;++z) {
        CombustibleSample s;
        s.position=-e+cell*Vector3df(x+0.5f,y+0.5f,z+0.5f);
        if(shape==Shape::Sphere && glm::length(s.position)>e.x) continue;
        s.radius=0.5f*std::min(cell.x,std::min(cell.y,cell.z));
        s.glyphHalfExtent=cell*0.5f;
        Vector3df normal;
        const float distance=signedDistance(center+s.position,&normal);
        if(-distance<=glm::length(cell)*0.6f) {
            s.normal=normal; s.area=spacing*spacing; areaSum+=s.area;
        }
        samples_.push_back(s);
    }
    const double volume=shape==Shape::Box?8.0*e.x*e.y*e.z:4.0*pi*e.x*e.x*e.x/3;
    const double area=shape==Shape::Box?8.0*(e.x*e.y+e.y*e.z+e.z*e.x):4.0*pi*e.x*e.x;
    for(auto& s:samples_) {
        s.volume=static_cast<float>(volume/samples_.size());
        s.initialFuel=s.fuel=fuelMass/samples_.size();
        s.area=static_cast<float>(s.area*area/areaSum);
    }
    for(size_t i=0;i<samples_.size();++i) for(size_t j=i+1;j<samples_.size();++j)
        if(glm::length(samples_[i].position-samples_[j].position)<smoothingLength_) {
            samples_[i].neighbors.push_back(j); samples_[j].neighbors.push_back(i);
        }
    return !samples_.empty();
}

Vector3df CombustibleBody::surfacePosition(const CombustibleSample& s) const
{
    const Vector3df p=center_+s.position;
    return usesSolidParticles() && s.area>0 ? p-s.normal*signedDistance(p) : p;
}

bool CombustibleBody::setCenter(const Vector3df& center) { if (!finite(center)) return false; center_ = center; return true; }
bool CombustibleBody::setMaterial(const CombustibleMaterial& m) { if (!m.valid()) return false; material_ = m; return true; }

float CombustibleBody::signedDistance(const Vector3df& p, Vector3df* normal) const
{
    const Vector3df d = p - center_;
    if (shape_ == Shape::Sphere) {
        const float len = glm::length(d);
        if (normal) *normal = len > 1e-8f ? d / len : Vector3df(0,1,0);
        return len - halfExtent_.x;
    }
    const Vector3df q = glm::abs(d) - halfExtent_;
    const Vector3df outside = glm::max(q, Vector3df(0));
    const float len = glm::length(outside);
    if (normal) {
        if (len > 1e-8f) *normal = outside / len * glm::sign(d);
        else { int axis = q.y > q.x ? 1 : 0; if (q.z > q[axis]) axis = 2;
            *normal = Vector3df(0); (*normal)[axis] = d[axis] < 0 ? -1.0f : 1.0f; }
    }
    return len + std::min(0.0f, std::max(q.x, std::max(q.y, q.z)));
}

bool CombustibleBody::blocksSegment(const Vector3df& a, const Vector3df& b) const
{
    const Vector3df d = b - a;
    if (shape_ == Shape::Sphere) {
        const float d2 = glm::dot(d,d);
        const float t = d2 > 0 ? std::clamp(glm::dot(center_-a,d)/d2, 0.0f, 1.0f) : 0;
        return signedDistance(a + d*t) < -1e-6f;
    }
    float lo = 0, hi = 1;
    for (int k = 0; k < 3; ++k) {
        const float min = center_[k] - halfExtent_[k] + 1e-6f, max = center_[k] + halfExtent_[k] - 1e-6f;
        if (std::abs(d[k]) < 1e-10f) { if (a[k] <= min || a[k] >= max) return false; }
        else { float t0 = (min-a[k])/d[k], t1 = (max-a[k])/d[k]; if (t0 > t1) std::swap(t0,t1);
            lo = std::max(lo,t0); hi = std::min(hi,t1); if (lo >= hi) return false; }
    }
    return lo < hi;
}

double CombustibleBody::heatCapacity(const CombustibleSample& s) const
{
    // Inert matrix remains after devolatilization; no disappearing heat capacity.
    return (s.volume * material_.density + s.fuel + s.residue) * material_.specificHeat;
}

CombustibleBodyStats CombustibleBody::stats() const
{
    CombustibleBodyStats out;
    double temperature = 0, capacity = 0;
    for (const auto& s : samples_) {
        out.initialFuel += s.initialFuel; out.fuel += s.fuel; out.residue += s.residue; out.pending += s.pending;
        const double c = heatCapacity(s); temperature += s.temperature * c; capacity += c;
    }
    out.temperature = capacity > 0 ? static_cast<float>(temperature/capacity) : 300;
    out.pyrolyzed = pyrolyzed; out.emitted = emitted; out.heatExchange = heatExchange;
    out.reactionRate = reactionRate; out.firstIgnitionTime = firstIgnitionTime;
    if (out.fuel <= out.initialFuel * 1e-6) out.state = CombustionState::Exhausted;
    else if (reactionRate > 1e-8f && material_.combustible && pyrolyzed > 1e-12) out.state = CombustionState::Burning;
    else if (wasBurning) out.state = CombustionState::Extinguished;
    else if (out.temperature > 305) out.state = CombustionState::Heating;
    return out;
}

bool CombustibleBodyStats::get(const char* name, double& v) const
{
#define FIELD(n) if (std::strcmp(name, #n) == 0) { v = static_cast<double>(n); return true; }
    FIELD(initialFuel) FIELD(fuel) FIELD(residue) FIELD(pending) FIELD(pyrolyzed) FIELD(emitted)
    FIELD(temperature) FIELD(reactionRate) FIELD(heatExchange) FIELD(firstIgnitionTime) FIELD(state)
#undef FIELD
    if (std::strcmp(name,"fuelFraction") == 0) { v = initialFuel > 0 ? fuel / initialFuel : 0; return true; }
    return false;
}
