#include "FlameWorld.h"
#include "RigidBodyWorld.h"
#include <algorithm>
#include <cmath>

#include "../../CGLib/Math/Box3d.h"

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace Phantom {

FlameWorld::FlameWorld()
{
    clear();
}

void FlameWorld::reset()
{
    releaseRigidClock();
    rigidBindings_.clear();
    if (solver_) solver_->setSolidCoupler(nullptr);
    coupler_.clear();
    bodies_.clear();
    coupler_ = FlameSolidCoupler{};
    fluid_  = std::make_unique<FlameFluid>();
    solver_ = std::make_unique<FlameSolver>();
    simTime_ = 0.0f;
    buildScene();
    populated_ = true;
}

void FlameWorld::clear()
{
    reset();
    running_ = false;
    populated_ = false;
}

void FlameWorld::buildScene()
{
    // SPH numbers from the former FlameView::FlameApp::setupInitialScene();
    // combustion/emitter/noise re-tuned for the Physical model (plan Phase 2).
    fluid_->setEffectLength(0.12f);
    fluid_->setDensity(1.0f);
    fluid_->setPressureCoe(20.0f);
    fluid_->setVicosityCoe(0.001f);
    fluid_->setMaxParticles(4000);
    fluid_->setMaxSpeed(3.0f);

    // Gentle flicker: an 8 m/s^2 curl-noise acceleration. (The historical
    // value was 0.5 per step without dt = 30 m/s^2 at 1/60 s, which kept
    // nearly every particle pinned at the maxSpeed cap -- plan A1 / Phase 2.)
    fluid_->setCurlNoiseStrength(8.0f);

    FlameFluid::Emitter e;
    e.center = Vector3df(0.0f, 0.0f, 0.0f);
    e.radius = 0.12f;
    e.rate   = 150.0f;
    // Co-emit ambient "air" particles so freshly ignited particles have SPH
    // neighbours from frame one instead of spawning into a near-vacuum -- and,
    // under the Physical model, so the fuel has oxygen to mix with. 2:1 air
    // keeps the mixture lean enough to burn out instead of leaving an
    // oxygen-starved fuel plume.
    e.airRate = 300.0f;
    // Physical combustion model (the FlameFluid default): fuel vapor leaves
    // the emitter preheated but below ignition and with no oxygen, so it only
    // burns where it has mixed with the air carriers. The wick-like pilot
    // keeps the base of that mixing layer above ignition so the flame stays
    // lit and anchored; above it the reaction's own heat release (spread by
    // the thermal diffusion pass) is what keeps the flame burning.
    e.fuelTemperature  = 700.0f;
    e.pilotTemperature = 1500.0f;
    e.pilotHeight      = 0.15f;
    fluid_->addEmitter(e);

    // Cosmetic secondary particles (non-SPH): population targets, not flat rates.
    fluid_->setSparkCountPerPrimary(6.0f);
    // 5 puffs per primary (was 14): at 14 the closed domain filled with smoke
    // thick enough that the order-independent PBVR mode (correctly) hid the
    // flame inside it -- Normal mode had only masked that by drawing the flame
    // on top of the smoke regardless of depth.
    fluid_->setSmokeCountPerPrimary(5.0f);

    solver_->add(fluid_.get());
    solver_->setEffectLength(fluid_->getEffectLength());
    solver_->setBoundary(Box3df(Vector3df(-1.5f, -0.05f, -1.5f),
                                Vector3df(1.5f, 4.0f, 1.5f)), 0.01f);
}

void FlameWorld::step()
{
    syncRigidBindings();
    if (!isRunning()) return;
    stepOnce();
}

void FlameWorld::stepOnce()
{
    populated_ = true;
    syncRigidBindings();
    int count = bodies_.empty() ? 1 : std::max(1,static_cast<int>(std::ceil(timeStep_/0.004f)));
    // Bound the relative-pose sweep approximation for rotating thin solids.
    // Limit the work for pathological input before converting to an integer.
    for(const auto& binding:rigidBindings_) {
        const auto* rigid=rigidWorld_->findBody(binding.rigidId);
        const auto* solid=findBody(binding.bodyId);
        const double angular=glm::length(glm::dvec3(rigid->angularVelocity));
        const auto extent=solid->halfExtent();
        const double speed=glm::length(glm::dvec3(rigid->linearVelocity))/physicalTransform_.scale+
            angular*glm::length(glm::dvec3(extent));
        const double motionSteps=std::max(angular*timeStep_/0.05,
            speed*timeStep_/(0.5*std::min(extent.x,std::min(extent.y,extent.z))));
        count=std::max(count,static_cast<int>(std::min(4096.0,std::ceil(motionSteps))));
    }
    std::vector<std::pair<Vector3df,Vector3df>> forces;
    if(hasRigidBindings()) for(const auto* rigid:rigidWorld_->getWorld().getBodies())
        forces.emplace_back(rigid->forceAccum,rigid->torqueAccum);
    for (int i=0;i<count;++i) {
        for (const auto& body:bodies_) body->beginMotionStep();
        if (hasRigidBindings()) {
            auto& rigid=rigidWorld_->getWorld(); const float saved=rigid.timeStep;
            // The solver clears force accumulators per substep. Frame forces
            // must act for the full common interval, not just the first slice.
            for(size_t j=0;j<forces.size();++j) {
                rigid.getBodies()[j]->forceAccum=forces[j].first;
                rigid.getBodies()[j]->torqueAccum=forces[j].second;
            }
            rigid.timeStep=timeStep_/count; rigid.stepUnconditional(); rigid.timeStep=saved;
            syncRigidBindings();
        }
        solver_->simulate(timeStep_/count);
    }
    simTime_ += timeStep_;
}

uint64_t FlameWorld::addBody(CombustibleBody::Shape shape, const glm::vec3& center,
    const glm::vec3& extent, double fuel, int material, int resolution)
{
    if (fluid_->fixedCarriers) return 0; // The current solid release adds gas carriers.
    if (material < 0 || material > 3 || bodies_.size() >= 32) return 0;
    auto body=std::make_unique<CombustibleBody>();
    if (!body->initializeParticles(nextBodyId_,shape,center,extent,resolution,fuel,CombustibleMaterial::preset(material))) return 0;
    if (!coupler_.add(body.get())) return 0;
    const auto id=nextBodyId_++; bodies_.push_back(std::move(body));
    solver_->setSolidCoupler(&coupler_); populated_=true;
    // Physical chemistry is mandatory while coupled; legacy temperature-only
    // burning cannot represent oxygen starvation.
    fluid_->setCombustionModel(FlameFluid::CombustionModel::Physical);
    return id;
}

bool FlameWorld::sphericalPreset(bool combustion, float radius, float spacing)
{
    if (!std::isfinite(radius) || !std::isfinite(spacing) || radius < 0.05f || radius > 10 || spacing < 0.005f ||
        spacing > radius || radius / spacing > 16) return false;
    const Vector3df center(0, radius, 0);
    std::vector<Vector3df> seeds;
    const int cells = static_cast<int>(std::floor((radius - spacing * 0.5f) / spacing));
    for (int z=-cells; z<=cells; ++z) for (int y=-cells; y<=cells; ++y) for (int x=-cells; x<=cells; ++x) {
        const Vector3df offset(x*spacing,y*spacing,z*spacing);
        if (getLength(offset) <= radius - spacing * 0.5f) seeds.push_back(center + offset);
    }
    if (seeds.empty() || seeds.size() > 12000) return false;
    reset(); running_ = false;
    fluid_->clearEmitters(); fluid_->getParticles().clear(); fluid_->fixedCarriers = true;
    fluid_->setMaxParticles(static_cast<int>(seeds.size()));
    fluid_->setEffectLength(spacing * 2.0f); solver_->setEffectLength(fluid_->getEffectLength());
    fluid_->setCoolRate(0); fluid_->setCurlNoiseStrength(0); fluid_->setVorticityEps(0);
    fluid_->setSparkCountPerPrimary(0); fluid_->setSmokeCountPerPrimary(0);
    fluid_->setThermalExpansionPressure(false);
    fluid_->setBurnRate(combustion ? 10.0f : 0.0f);
    fluid_->setThermalDiffusivity(0.005f); fluid_->setFuelDiffusivity(0.005f);
    fluid_->setOxygenDiffusivity(0.005f); fluid_->setSootDiffusivity(0.005f);
    solver_->setBoundarySphere(center, radius);
    FlameSolver::ThermalBoundary thermal;
    thermal.sourceCenter = center - Vector3df(0, radius * 0.5f, 0);
    thermal.sourceRadius = radius * 0.3f;
    const float volumeScale=std::pow(radius/0.6f,3.0f);
    thermal.sourcePower = (combustion ? 150.0f : 8.0f)*volumeScale;
    thermal.sourceDuration = combustion ? 0.3f : 0.0f;
    thermal.wallThickness = spacing * 2; thermal.wallRate = 2;
    solver_->setThermalBoundary(thermal);
    for (const auto& pos : seeds) {
        fluid_->createParticle(pos, spacing * 0.5f);
        const size_t i = fluid_->getParticles().size()-1;
        FlameParticle p(fluid_->getParticles(), i, fluid_.get());
        const bool fuel = combustion && getDistance(pos,thermal.sourceCenter) <= radius*0.35f;
        p.setFuel(fuel ? 0.3f : 0); p.setOxygen(fuel ? 0.7f : 1);
        p.setAir(!fuel);
        fluid_->initialFuelMass += p.getMass()*p.getFuel();
        fluid_->initialOxygenMass += p.getMass()*p.getOxygen();
        fluid_->initialHeat += p.getMass()*p.getTemperature();
    }
    render_.hazeEnabled = false;
    render_.carrierDebug = combustion ? 0 : 2;
    return true;
}

CombustibleBody* FlameWorld::findBody(uint64_t id)
{
    for (auto& b:bodies_) if (b->id()==id) return b.get();
    return nullptr;
}
bool FlameWorld::removeBody(uint64_t id)
{
    const auto it=std::find_if(bodies_.begin(),bodies_.end(),[id](const auto& b){return b->id()==id;});
    if (it==bodies_.end()) return false;
    unbindRigidBody(id);
    coupler_.remove(id); bodies_.erase(it);
    if (bodies_.empty()) solver_->setSolidCoupler(nullptr);
    return true;
}
FlameWorld::~FlameWorld() { releaseRigidClock(); }

void FlameWorld::releaseRigidClock()
{
    if (rigidWorld_ && hasRigidBindings()) {
        running_=rigidWorld_->isRunning();
        rigidWorld_->setExternalClock({});
        for(const auto& b:rigidBindings_) {
            rigidWorld_->setCombustibleRepresentation(b.rigidId,false);
            if(auto* solid=findBody(b.bodyId)) {
                solid->setMotion(solid->center(),solid->orientation(),Vector3df(0),Vector3df(0));
                solid->beginMotionStep();
            }
        }
    }
}
void FlameWorld::setRigidWorld(RigidBodyWorld* world)
{
    releaseRigidClock(); rigidBindings_.clear(); rigidWorld_=world;
}
void FlameWorld::setRunning(bool r)
{
    running_=r; if(r) populated_=true;
    if(hasRigidBindings()) rigidWorld_->setRunning(r);
}
bool FlameWorld::isRunning() const
{
    return hasRigidBindings()?rigidWorld_->isRunning():running_;
}
uint64_t FlameWorld::rigidBodyId(uint64_t id) const
{
    for(const auto& b:rigidBindings_) if(b.bodyId==id) return b.rigidId;
    return 0;
}
bool FlameWorld::bindRigidBody(uint64_t id, std::size_t index)
{
    auto* solid=findBody(id);
    if(!solid || !rigidWorld_ || (canBindRigid_ && !canBindRigid_())) return false;
    const auto rigidId=rigidWorld_->bodyId(index);
    auto* rigid=rigidWorld_->findBody(rigidId);
    if(!rigid || !rigid->shape) return false;
    for(const auto& b:rigidBindings_) if(b.rigidId==rigidId && b.bodyId!=id) return false;
    Vector3df extent;
    if(rigid->shape->getType()==ShapeType::Box && solid->shape()==CombustibleBody::Shape::Box)
        extent=static_cast<BoxShape*>(rigid->shape)->halfExtents/physicalTransform_.scale;
    else if(rigid->shape->getType()==ShapeType::Sphere && solid->shape()==CombustibleBody::Shape::Sphere)
        extent=Vector3df(static_cast<SphereShape*>(rigid->shape)->radius/physicalTransform_.scale);
    else return false;
    if(!std::isfinite(extent.x) || !std::isfinite(extent.y) || !std::isfinite(extent.z) ||
        std::min(extent.x,std::min(extent.y,extent.z))<=0) return false;
    if(glm::length(extent-solid->halfExtent())>1e-5f) return false;
    if(!solid->setMotion(physicalTransform_.toSimulation(rigid->position),rigid->orientation,
        physicalTransform_.velocityToSimulation(rigid->linearVelocity),rigid->angularVelocity)) return false;
    const bool first=!hasRigidBindings();
    for(auto& b:rigidBindings_) if(b.bodyId==id) { rigidWorld_->setCombustibleRepresentation(b.rigidId,false); b.rigidId=rigidId;
        rigidWorld_->setCombustibleRepresentation(rigidId,true); solid->beginMotionStep(); return true; }
    rigidBindings_.push_back({id,rigidId}); solid->beginMotionStep();
    rigidWorld_->setCombustibleRepresentation(rigidId,true);
    if(first) { rigidWorld_->setRunning(running_); rigidWorld_->setExternalClock([this]{ stepOnce(); }); }
    return true;
}
bool FlameWorld::unbindRigidBody(uint64_t id)
{
    auto it=std::find_if(rigidBindings_.begin(),rigidBindings_.end(),[id](const auto& b){return b.bodyId==id;});
    if(it==rigidBindings_.end()) return false;
    rigidWorld_->setCombustibleRepresentation(it->rigidId,false);
    if(rigidBindings_.size()==1) releaseRigidClock();
    if(auto* body=findBody(id)) { body->setMotion(body->center(),body->orientation(),Vector3df(0),Vector3df(0)); body->beginMotionStep(); }
    rigidBindings_.erase(it); return true;
}
void FlameWorld::syncRigidBindings()
{
    for(std::size_t i=0;i<rigidBindings_.size();) {
        const auto binding=rigidBindings_[i];
        auto* rigid=rigidWorld_->findBody(binding.rigidId); auto* solid=findBody(binding.bodyId);
        if(!rigid || !solid) { unbindRigidBody(binding.bodyId); continue; }
        if(!solid->setMotion(physicalTransform_.toSimulation(rigid->position),rigid->orientation,
            physicalTransform_.velocityToSimulation(rigid->linearVelocity),rigid->angularVelocity)) {
            unbindRigidBody(binding.bodyId); continue;
        }
        ++i;
    }
}

void FlameWorld::stopSource()
{
    auto thermal=solver_->getThermalBoundary(); thermal.sourcePower=0; solver_->setThermalBoundary(thermal);
    for (auto& e:fluid_->getEmittersMutable()) { e.rate=0; e.accumulator=0; e.pilotTemperature=0; }
}
double FlameWorld::gasFuelMass() const
{
    auto& gas=fluid_->getParticles(); double mass=0;
    for (size_t i=0;i<gas.size();++i) mass+=gas.fuels[i]*FlameParticle(gas,i,fluid_.get()).getMass();
    return mass;
}
bool FlameWorld::combustionPreset(int resolution, double fuelMass)
{
    if(resolution<1 || resolution>8 || !std::isfinite(fuelMass) || fuelMass<=0) return false;
    reset();
    fluid_->setIgnitionTemperature(350);
    render_.particleSize=0.045f;
    render_.smokeParticleSize=0.07f;
    render_.autoReferenceTemperature=false;
    render_.referenceTemperature=2000;
    render_.exposure=1;
    fluid_->setIgnitionWidth(30);
    fluid_->setBurnRate(30);
    fluid_->setHeatRelease(5000);
    fluid_->setCoolRate(0.1f);
    fluid_->setCurlNoiseStrength(0);
    fluid_->setSparkCountPerPrimary(0);
    fluid_->setSmokeCountPerPrimary(2);
    fluid_->setMaxParticles(1800);
    fluid_->setLifeMax(2);
    auto& source=fluid_->getEmittersMutable().front();
    source.center=Vector3df(-0.15f,0.15f,0);
    source.radius=0.05f; source.rate=100; source.airRate=300;
    source.pilotTemperature=1500; source.pilotHeight=0.14f;
    // Explicit scene air supply, retained when the ignition source is stopped.
    FlameFluid::Emitter air;
    air.center=Vector3df(0,0.35f,0); air.radius=0.3f; air.rate=0;
    air.airRate=400; air.pilotTemperature=0;
    fluid_->addEmitter(air);
    const uint64_t a=addBody(CombustibleBody::Shape::Box,{0,0.22f,0},{0.08f,0.06f,0.08f},fuelMass,0,resolution);
    const uint64_t b=addBody(CombustibleBody::Shape::Box,{0.14f,0.36f,0},{0.06f,0.06f,0.08f},fuelMass,0,resolution);
    addBody(CombustibleBody::Shape::Sphere,{0.3f,0.3f,0},{0.07f,0.07f,0.07f},fuelMass,3,resolution);
    for (uint64_t id:{a,b}) {
        auto m=findBody(id)->material(); m.density=0.03f; m.pyrolysisTemperature=420;
        m.pyrolysisRate=0.3f; m.latentHeat=30; findBody(id)->setMaterial(m);
    }
    return true;
}

} // namespace Phantom
