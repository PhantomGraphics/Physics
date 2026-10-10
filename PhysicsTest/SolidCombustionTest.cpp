#include "pch.h"
#include "../Physics/CombustibleBody.h"
#include "../Physics/SolidCombustionSolver.h"
#include "../Physics/FlameSolidCoupler.h"
#include "../Physics/FlameFluid.h"
#include "../PhysicsView/FlameWorld.h"
#include "../Physics/FlameStats.h"
#include "../PhysicsView/CombustibleGltf.h"
#include <cstring>
#include <limits>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

namespace {
CombustibleBody makeBody(bool combustible=true,int resolution=2)
{
    CombustibleBody b; auto m=CombustibleMaterial::preset(0); m.combustible=combustible;
    b.initialize(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},resolution,0.01,m); return b;
}
double energy(const CombustibleBody& b) { double e=0; for(const auto& s:b.samples()) e+=s.temperature*b.heatCapacity(s); return e; }
double gasFuel(FlameFluid& f) { double m=0; auto& g=f.getParticles(); for(size_t i=0;i<g.size();++i) m+=g.fuels[i]*FlameParticle(g,i,&f).getMass(); return m; }
}

TEST(CombustibleBody, SurfaceAreaVolumeAndFuel)
{
    for(int n:{1,2,4}) {
        auto b=makeBody(true,n); double area=0,volume=0;
        for(const auto& s:b.samples()) { area+=s.area; volume+=s.volume; }
        EXPECT_NEAR(area,0.24,1e-7); EXPECT_NEAR(volume,0.008,1e-8); EXPECT_NEAR(b.stats().fuel,0.01,1e-12);
    }
    CombustibleBody sphere;
    ASSERT_TRUE(sphere.initialize(2,CombustibleBody::Shape::Sphere,{0,0,0},{0.1f,0.1f,0.1f},3,0.01,{}));
    double area=0,volume=0; for(const auto& s:sphere.samples()) { area+=s.area; volume+=s.volume; }
    EXPECT_NEAR(area,4*3.14159265*0.01,1e-7); EXPECT_NEAR(volume,4*3.14159265*0.001/3,1e-8);
}

TEST(CombustibleBody, VolumeParticlesHaveInteriorAndConserveStock)
{
    for(auto shape:{CombustibleBody::Shape::Box,CombustibleBody::Shape::Sphere}) {
        CombustibleBody b;
        ASSERT_TRUE(b.initializeParticles(1,shape,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,{}));
        double volume=0,area=0,fuel=0; size_t interior=0;
        for(const auto& p:b.samples()) {
            volume+=p.volume; area+=p.area; fuel+=p.fuel;
            EXPECT_GT(p.radius,0); EXPECT_LE(b.signedDistance(p.position),1e-6f);
            if(p.area==0) ++interior;
            else EXPECT_NEAR(b.signedDistance(b.surfacePosition(p)),0,1e-6);
            for(auto j:p.neighbors) EXPECT_LT(glm::length(p.position-b.samples()[j].position),b.smoothingLength());
        }
        EXPECT_GT(interior,0u); EXPECT_NEAR(fuel,0.01,1e-12);
        EXPECT_NEAR(volume,shape==CombustibleBody::Shape::Box?0.008:4*3.14159265359*0.001/3,1e-8);
        EXPECT_NEAR(area,shape==CombustibleBody::Shape::Box?0.24:4*3.14159265359*0.01,1e-7);
    }
}

TEST(SolidCombustionSolver, SolidSPHTransfersInteriorHeatConservatively)
{
    CombustibleBody b; auto m=CombustibleMaterial::preset(3);
    ASSERT_TRUE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,m));
    size_t hot=0;
    while(b.samples()[hot].area>0) ++hot;
    b.samples()[hot].temperature=1000;
    const double before=energy(b); SolidCombustionSolver solver; solver.cooling=0;
    for(int i=0;i<100;++i) solver.update(b,0.001f);
    EXPECT_LT(b.samples()[hot].temperature,1000);
    EXPECT_GT(b.samples()[b.samples()[hot].neighbors[0]].temperature,300);
    EXPECT_NEAR(energy(b),before,before*2e-6);
    for(const auto& p:b.samples()) { EXPECT_GE(p.temperature,300); EXPECT_LE(p.temperature,1000); }
}

TEST(FlameSolidCoupler, SolidSPHInteriorVaporReachesSurfaceWithoutLoss)
{
    SKIP_IN_DEBUG_SLOW();
    CombustibleBody b; auto m=CombustibleMaterial::preset(3);
    ASSERT_TRUE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,m));
    size_t interior=0; while(b.samples()[interior].area>0) ++interior;
    auto& p=b.samples()[interior]; const double stock=p.initialFuel*0.08;
    p.fuel-=stock; p.pending=stock; p.pendingHeat=stock*100;
    FlameFluid f; FlameSolidCoupler c; ASSERT_TRUE(c.add(&b)); c.solidSolver.cooling=0;
    c.update(f,0.004f,0);
    EXPECT_EQ(f.getParticles().size(),0u);
    for(int i=0;i<1000;++i) c.update(f,0.004f,i*0.004);
    EXPECT_LT(p.pending,stock);
    const auto stats=b.stats();
    EXPECT_NEAR(stats.fuel+stats.residue+stats.pending+gasFuel(f),0.01,2e-10);
    for(size_t i=0;i<f.getParticles().size();++i) {
        EXPECT_GE(b.signedDistance(f.getParticles().positions[i]),0);
        EXPECT_EQ(f.getParticles().oxygens[i],0);
    }
    // Exhausted stock flushes fractional carriers instead of trapping the
    // final interior vapor behind the normal emission threshold.
    for(auto& s:b.samples()) { s.residue+=s.fuel; s.fuel=0; }
    for(int i=0;i<1000;++i) c.update(f,0.004f,4+i*0.004);
    EXPECT_GT(f.getParticles().size(),0u);
    EXPECT_GT(b.emitted,0);
    const auto final=b.stats();
    EXPECT_NEAR(final.fuel+final.residue+final.pending+gasFuel(f),0.01,2e-10);
}

TEST(SolidCombustionSolver, SolidSPHResolutionAndTimeStepFuelConvergence)
{
    double consumed[4]; int index=0;
    for(int n:{2,4}) for(float dt:{0.004f,0.002f}) {
        CombustibleBody b;
        ASSERT_TRUE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},n,0.01,{}));
        for(auto& p:b.samples()) p.temperature=1000;
        SolidCombustionSolver s; s.cooling=0; s.pendingFractionLimit=1;
        for(int i=0;i<100;++i) s.update(b,dt);
        consumed[index++]=b.pyrolyzed/(100*dt);
        const auto stats=b.stats();
        EXPECT_NEAR(stats.fuel+stats.residue+stats.pending,0.01,1e-12);
    }
    for(int i=1;i<4;++i) EXPECT_NEAR(consumed[i],consumed[0],consumed[0]*0.02);
}

TEST(SolidCombustionSolver, ColdAndInertDoNotPyrolyze)
{
    SolidCombustionSolver solver;
    auto cold=makeBody(); auto inert=makeBody(false);
    for(auto& s:inert.samples()) s.temperature=1800;
    for(int i=0;i<100;++i) { solver.update(cold,0.01f); solver.update(inert,0.01f); }
    EXPECT_DOUBLE_EQ(cold.stats().pyrolyzed,0); EXPECT_DOUBLE_EQ(inert.stats().pyrolyzed,0);
}

TEST(SolidCombustionSolver, ClosedConductionConservesHeat)
{
    auto b=makeBody(false); b.samples()[0].temperature=1800;
    const double before=energy(b); SolidCombustionSolver solver; solver.cooling=0;
    for(int i=0;i<100;++i) solver.update(b,0.004f);
    EXPECT_NEAR(energy(b),before,before*2e-6);
    EXPECT_LT(b.samples()[0].temperature,1800); EXPECT_GT(b.samples()[1].temperature,300);
    for(const auto& s:b.samples()) { EXPECT_GE(s.temperature,300); EXPECT_LE(s.temperature,1800); }
}

TEST(FlameSolidCoupler, ClosedGasSolidHeatAndOcclusion)
{
    for(bool solidSPH:{false,true}) {
    auto b=makeBody(false); FlameFluid f; FlameSolidCoupler coupler; coupler.add(&b); coupler.solidSolver.cooling=0;
    if(solidSPH) ASSERT_TRUE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,b.material()));
    auto& g=f.getParticles(); g.push_back({0.13f,0.05f,0.05f},0.02f,1,1800);
    const double cg=FlameParticle(g,0,&f).getMass(); const double before=energy(b)+cg*g.temperatures[0];
    coupler.update(f,0.02f,0);
    EXPECT_NEAR(energy(b)+cg*g.temperatures[0],before,before*2e-6);
    EXPECT_LT(g.temperatures[0],1800);
    EXPECT_TRUE(coupler.occluded({-0.12f,0,0},{0.12f,0,0}));
    EXPECT_FALSE(coupler.occluded({0.11f,0,0},{0.2f,0,0}));
    }
}

TEST(FlameSolidCoupler, SweptBoxAndSphereCollision)
{
    for(auto shape:{CombustibleBody::Shape::Box,CombustibleBody::Shape::Sphere}) {
        CombustibleBody b; ASSERT_TRUE(b.initialize(1,shape,{0,0,0},{0.01f,0.1f,0.1f},2,0.01,{}));
        FlameSolidCoupler c; c.add(&b); FlameFluid f; auto& g=f.getParticles();
        g.push_back({0.3f,0,0},0.02f,1,300); g.velocities[0]={10,0,0};
        c.constrain(f,{{-0.3f,0,0}});
        EXPECT_LT(g.positions[0].x,0); EXPECT_GE(b.signedDistance(g.positions[0]),0.01999f); EXPECT_LE(g.velocities[0].x,0);
    }
}

TEST(FlameSolidCoupler, ParticleLimitRetainsFuelAndStopsPyrolysis)
{
    for(bool solidSPH:{false,true}) {
    auto b=makeBody();
    if(solidSPH) ASSERT_TRUE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,b.material()));
    for(auto& s:b.samples()) s.temperature=1200;
    FlameFluid f; f.setMaxParticles(0); FlameSolidCoupler c; c.add(&b); c.solidSolver.cooling=0;
    for(int i=0;i<500;++i) c.update(f,0.004f,i*0.004);
    auto s=b.stats(); EXPECT_GT(s.pending,0); EXPECT_LE(s.pending,s.initialFuel*c.solidSolver.pendingFractionLimit+1e-12);
    EXPECT_NEAR(s.fuel+s.residue+s.pending,s.initialFuel,1e-12);
    const double fuel=s.fuel; c.update(f,0.1f,3); EXPECT_NEAR(b.stats().fuel,fuel,1e-12);
    f.setMaxParticles(10000); c.update(f,0.004f,4);
    s=b.stats(); EXPECT_NEAR(s.fuel+s.pending+s.residue+gasFuel(f),s.initialFuel,1e-9);
    for(float oxygen:f.getParticles().oxygens) EXPECT_FLOAT_EQ(oxygen,0);
    auto& g=f.getParticles(); const double before=gasFuel(f);
    for(size_t i=0;i<g.size();++i) { g.temperatures[i]=1800; FlameParticle(g,i,&f).react(0.1f); }
    EXPECT_NEAR(gasFuel(f),before,1e-12);
    }
}

// SourceOffRetainsCarriersAndFiniteSolidFuel moved to scenarios/49_flame_combustion_regression.json (too slow for a unit test).

// InertAAndDistantBDoNotIgnite moved to scenarios/49_flame_combustion_regression.json (too slow for a unit test).

TEST(FlameWorld, DeleteResetAndDisplayIndependence)
{
    SKIP_IN_DEBUG_SLOW();
    Phantom::FlameWorld a,b; a.combustionPreset(); b.combustionPreset();
    EXPECT_FALSE(a.combustionPreset(16)); EXPECT_EQ(a.bodies().size(),3u);
    b.render().renderScale=30; b.render().renderOffset={100,200,300};
    for(int i=0;i<20;++i) { a.stepOnce(); b.stepOnce(); }
    EXPECT_EQ(a.fluid().getParticles().positions,b.fluid().getParticles().positions);
    EXPECT_EQ(a.bodies()[0]->stats().fuel,b.bodies()[0]->stats().fuel);
    const auto id=a.bodies()[0]->id(); EXPECT_TRUE(a.removeBody(id)); EXPECT_FALSE(a.removeBody(id));
    EXPECT_NEAR(computeFlameStats(a.fluid(),&a.coupler()).fuelBalanceError,0,1e-7);
    a.reset(); EXPECT_TRUE(a.bodies().empty()); EXPECT_TRUE(a.coupler().bodies().empty());
    EXPECT_EQ(a.coupler().initialFuelMass,0); EXPECT_EQ(a.fluid().burnedFuelMass,0);
    EXPECT_GT(a.addBody(CombustibleBody::Shape::Sphere,{0,1,0},{0.1f,0.1f,0.1f},0.01),id);
}

TEST(SolidCombustionSolver, TimeStepAndSurfaceResolutionConvergence)
{
    double consumed[4]; int index=0;
    for(int n:{2,4}) for(float dt:{0.004f,0.002f}) {
        auto b=makeBody(true,n); for(auto& s:b.samples()) s.temperature=1000;
        SolidCombustionSolver solver; solver.cooling=0; solver.pendingFractionLimit=1;
        for(int i=0;i<static_cast<int>(0.4f/dt);++i) solver.update(b,dt);
        consumed[index++]=b.stats().pyrolyzed;
    }
    for(int i=1;i<4;++i) EXPECT_NEAR(consumed[i],consumed[0],consumed[0]*0.02);
}

TEST(CombustibleBody, InvalidParametersAndTransform)
{
    SKIP_IN_DEBUG_SLOW();
    CombustibleBody b;
    EXPECT_FALSE(b.initialize(1,CombustibleBody::Shape::Box,{0,0,0},{-1,1,1},2,1,{}));
    EXPECT_FALSE(b.initialize(1,CombustibleBody::Shape::Box,{0,0,0},{1,1,1},99,1,{}));
    EXPECT_TRUE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.5f,0.005f,0.5f},2,0.01,{}));
    EXPECT_LE(b.particles().size(),4096u);
    EXPECT_FALSE(b.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{1,1,1},16,1,{}));
    auto material=CombustibleMaterial{}; material.specificHeat=std::numeric_limits<float>::quiet_NaN(); EXPECT_FALSE(material.valid());
    FlamePhysicalTransform xf; xf.offset={10,20,30}; const Vector3df p{0.2f,0.3f,0.4f};
    EXPECT_LT(glm::length(xf.toSimulation(xf.toScene(p))-p),1e-6);
    EXPECT_FLOAT_EQ(xf.areaToScene(1),144); EXPECT_FLOAT_EQ(xf.volumeToScene(1),1728);
}

// CoupledTimeStepAndResolutionRetainCarriersAndFuelBudget moved to scenarios/49_flame_combustion_regression.json (too slow for a unit test).

TEST(SolidCombustionSolver, MoreFuelLastsLongerAndExtinguishedCanResume)
{
    double exhausted[2]={0,0};
    for(int variant=0;variant<2;++variant) {
        auto b=makeBody(); if(variant) for(auto& s:b.samples()) { s.initialFuel*=2; s.fuel*=2; }
        SolidCombustionSolver solver; solver.cooling=0; solver.pendingFractionLimit=1;
        for(int step=0;step<1000;++step) {
            for(auto& s:b.samples()) { s.temperature=1200; s.pending=0; s.pendingHeat=0; }
            solver.update(b,0.01f);
            if(b.stats().fuel<1e-10) { exhausted[variant]=step*0.01; break; }
        }
    }
    EXPECT_GT(exhausted[1],exhausted[0]*1.8);
    auto b=makeBody(); b.wasBurning=true;
    EXPECT_EQ(b.stats().state,CombustionState::Extinguished);
    SolidCombustionSolver solver;
    const double before=b.stats().fuel;
    for(auto& s:b.samples()) s.temperature=1200;
    solver.update(b,0.1f); EXPECT_LT(b.stats().fuel,before);
}

TEST(CombustibleGltf, LocalCharAndOpaqueGeometry)
{
    auto body=makeBody(); body.samples()[0].fuel=0;
    const auto doc=Phantom::makeCombustibleGltf(body,0);
    ASSERT_EQ(doc.meshes.size(),1); EXPECT_EQ(doc.meshes[0].primitives.size(),2);
    EXPECT_LT(doc.materials[0].pbrMetallicRoughness.baseColorFactor.r,0.02f);
    EXPECT_GT(doc.materials[15].pbrMetallicRoughness.baseColorFactor.r,0.4f);
    for(const auto& m:doc.materials) EXPECT_FLOAT_EQ(m.pbrMetallicRoughness.baseColorFactor.a,1);
    EXPECT_EQ(Phantom::combustibleColorLevel(body.samples()[0],2),0);
    ASSERT_TRUE(body.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,{}));
    body.particles()[0].fuel=0;
    const auto particleDoc=Phantom::makeCombustibleGltf(body,0);
    ASSERT_EQ(particleDoc.meshes.size(),1u);
    EXPECT_EQ(particleDoc.meshes[0].primitives.size(),2u);
    size_t vertices=0;
    for(const auto& primitive:particleDoc.meshes[0].primitives)
        vertices+=particleDoc.accessors[primitive.positionAccessor].count;
    EXPECT_EQ(vertices,body.particles().size()*63);
}

TEST(CombustibleGltf, ScalarGeometryKeepsSampleIndicesAndDoesNotChangeWithHeatOrFuel)
{
    CombustibleBody body;
    ASSERT_TRUE(body.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},2,0.01,{}));
    const auto before=Phantom::makeCombustibleScalarGltf(body);
    ASSERT_EQ(before.meshes.size(),1u);
    ASSERT_EQ(before.meshes[0].primitives.size(),1u);
    ASSERT_EQ(before.materials.size(),1u);
    const auto& primitive=before.meshes[0].primitives[0];
    const auto& uv=before.accessors[primitive.texCoord0Accessor];
    const auto& view=before.bufferViews[uv.bufferViewIndex];
    const auto& bytes=before.buffers[view.bufferIndex].data;
    ASSERT_EQ(uv.count,body.samples().size()*63);
    for(size_t i=0;i<uv.count;++i) {
        glm::vec2 value;
        std::memcpy(&value,bytes.data()+view.byteOffset+uv.byteOffset+i*sizeof(value),sizeof(value));
        EXPECT_FLOAT_EQ(value.x,static_cast<float>(i/63));
    }
    for(auto& sample:body.samples()) { sample.temperature=1500; sample.fuel=0; }
    const auto after=Phantom::makeCombustibleScalarGltf(body);
    ASSERT_EQ(before.buffers.size(),after.buffers.size());
    for(size_t i=0;i<before.buffers.size();++i) EXPECT_EQ(before.buffers[i].data,after.buffers[i].data);
    EXPECT_FLOAT_EQ(after.materials[0].pbrMetallicRoughness.baseColorFactor.a,1);
}

TEST(SolidCombustionSolver, PyrolysisPendingSensibleHeatAndLatentEnergy)
{
    auto b=makeBody(); for(auto& s:b.samples()) s.temperature=1200;
    const auto excessEnergy=[](const CombustibleBody& body) { double e=0;
        for(const auto& s:body.samples()) e+=(s.temperature-300)*body.heatCapacity(s)+s.pendingHeat; return e; };
    const double before=excessEnergy(b);
    SolidCombustionSolver solver; solver.cooling=0;
    for(int i=0;i<20;++i) solver.update(b,0.004f);
    EXPECT_NEAR(excessEnergy(b)+b.pyrolyzed*b.material().latentHeat,before,before*2e-6);
    EXPECT_GT(b.stats().pending,0);
}
