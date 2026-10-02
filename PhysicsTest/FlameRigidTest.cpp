#include "pch.h"
#include "../Physics/CombustibleBody.h"
#include "../Physics/FlameSolidCoupler.h"
#include "../Physics/FlameFluid.h"
#include "../Physics/PhysicsSolver.h"
#include "../PhysicsView/FlameWorld.h"
#include "../PhysicsView/RigidBodyWorld.h"
#include <limits>

using namespace Phantom;
using namespace Phantom::Physics;
using Math::Vector3df;

TEST(CombustibleBody, RotatedSurfaceDistanceAndOcclusion)
{
    CombustibleBody body;
    ASSERT_TRUE(body.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.2f,0.04f,0.08f},2,0.01,{}));
    const auto q=glm::angleAxis(0.7f,glm::normalize(Vector3df(1,2,3)));
    const Vector3df center{0.5f,0.6f,-0.2f};
    const double fuel=body.stats().fuel;
    ASSERT_TRUE(body.setMotion(center,q,{0,0,0},{0,0,0}));
    for(const auto& sample:body.samples()) if(sample.area>0) {
        const auto p=body.surfacePosition(sample),n=body.surfaceNormal(sample);
        EXPECT_NEAR(body.signedDistance(p),0,2e-6);
        EXPECT_NEAR(glm::length(n),1,1e-6);
        EXPECT_GT(body.signedDistance(p+n*0.01f),0);
        EXPECT_FALSE(body.blocksSegment(p,p+n*0.1f));
    }
    EXPECT_TRUE(body.blocksSegment(center+q*Vector3df(0,0.1f,0),center-q*Vector3df(0,0.1f,0)));
    EXPECT_FALSE(body.blocksSegment(center+q*Vector3df(0.3f,0.1f,0),center+q*Vector3df(0.3f,-0.1f,0)));
    EXPECT_DOUBLE_EQ(body.stats().fuel,fuel);
    EXPECT_FALSE(body.setMotion(center,{0,0,0,0},{0,0,0},{0,0,0}));
    EXPECT_FALSE(body.setMotion(center,q,{std::numeric_limits<float>::infinity(),0,0},{0,0,0}));
    EXPECT_NEAR(std::abs(glm::dot(body.orientation(),q)),1,1e-6);
}

TEST(FlameSolidCoupler, RotatedEmissionInheritsSurfaceVelocity)
{
    CombustibleBody body;
    ASSERT_TRUE(body.initializeParticles(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.06f,0.08f},2,0.01,{}));
    const auto q=glm::angleAxis(1.1f,Vector3df(0,0,1));
    ASSERT_TRUE(body.setMotion({0.2f,0.3f,0},q,{0.2f,-0.1f,0.3f},{0,0,2}));
    body.beginMotionStep();
    const auto it=std::find_if(body.samples().begin(),body.samples().end(),[](const auto& s){return s.area>0;});
    ASSERT_NE(it,body.samples().end());
    const double amount=0.000016;
    it->fuel-=amount; it->pending=amount; it->pendingHeat=amount*400;
    const auto surface=body.surfacePosition(*it),normal=body.surfaceNormal(*it);
    FlameFluid fluid; FlameSolidCoupler coupler;
    coupler.solidSolver.cooling=0;
    for(auto& s:body.samples()) s.neighbors.clear();
    ASSERT_TRUE(coupler.add(&body)); coupler.update(fluid,0.001f,0);
    ASSERT_EQ(fluid.getParticles().size(),1u);
    const auto& gas=fluid.getParticles();
    EXPECT_NEAR(glm::length(gas.positions[0]-(surface+normal*0.0126f)),0,1e-6);
    EXPECT_NEAR(glm::length(gas.velocities[0]-(body.velocityAt(gas.positions[0])+normal*coupler.emissionSpeed)),0,1e-6);
    EXPECT_FLOAT_EQ(gas.oxygens[0],0);
    EXPECT_NEAR(body.stats().fuel+body.stats().pending+body.stats().residue+body.emitted,0.01,1e-10);
}

TEST(FlameSolidCoupler, MovingPlateSweepsGasAndUsesRelativeVelocity)
{
    CombustibleBody body;
    ASSERT_TRUE(body.initialize(1,CombustibleBody::Shape::Box,{-0.2f,0,0},{0.01f,0.1f,0.1f},1,0.01,{}));
    body.beginMotionStep();
    ASSERT_TRUE(body.setMotion({0.2f,0,0},{1,0,0,0},{1,0,0},{0,0,0}));
    FlameFluid fluid; auto& gas=fluid.getParticles();
    gas.push_back({0,0,0},0.01f,1,300); gas.velocities.back()={0,0,0};
    FlameSolidCoupler coupler; ASSERT_TRUE(coupler.add(&body));
    coupler.constrain(fluid,{{0,0,0}});
    EXPECT_GE(gas.positions[0].x,0.21999f);
    EXPECT_NEAR(gas.velocities[0].x,1,1e-6);
    EXPECT_GE(body.signedDistance(gas.positions[0]),0.00999f);
}

TEST(FlameSolidCoupler, RotationPreservesLocalHeatExchange)
{
    CombustibleBody reference,rotated;
    auto material=CombustibleMaterial::preset(3); material.conductivity=0;
    ASSERT_TRUE(reference.initialize(1,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.04f,0.08f},2,0.01,material));
    ASSERT_TRUE(rotated.initialize(2,CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.04f,0.08f},2,0.01,material));
    const auto q=glm::angleAxis(1.2f,glm::normalize(Vector3df(1,2,3)));
    ASSERT_TRUE(rotated.setMotion({0.4f,0.3f,0.2f},q,{0,0,0},{0,0,0})); rotated.beginMotionStep();
    FlameFluid a,b;
    for(const auto& point:{Vector3df(0.13f,0.02f,0),Vector3df(-0.13f,0.02f,0),Vector3df(0,0.08f,0)}) {
        a.getParticles().push_back(point,0.01f,1,1000);
        b.getParticles().push_back(rotated.worldPosition(point),0.01f,1,1000);
    }
    FlameSolidCoupler ca,cb; ca.solidSolver.cooling=cb.solidSolver.cooling=0;
    ASSERT_TRUE(ca.add(&reference)); ASSERT_TRUE(cb.add(&rotated));
    ca.update(a,0.004f,0); cb.update(b,0.004f,0);
    EXPECT_GT(reference.heatExchange,0);
    EXPECT_NEAR(rotated.heatExchange,reference.heatExchange,1e-8);
    for(size_t i=0;i<reference.samples().size();++i)
        EXPECT_NEAR(rotated.samples()[i].temperature,reference.samples()[i].temperature,1e-4);
    for(size_t i=0;i<a.getParticles().size();++i)
        EXPECT_NEAR(b.getParticles().temperatures[i],a.getParticles().temperatures[i],1e-4);
}

TEST(FlameWorld, RigidClockFollowsRotationAndDoesNotDoubleStep)
{
    PhysicsSolver physics; RigidBodyWorld rigid(physics); FlameWorld flame;
    flame.setRigidWorld(&rigid); rigid.getWorld().params().gravity={0,0,0};
    auto* moving=rigid.addBox({0,6,0},{1.2f,0.6f,0.96f},1);
    moving->linearVelocity={1.2f,0,0}; moving->angularVelocity={0,0,2};
    const auto id=flame.addBody(CombustibleBody::Shape::Box,{0,0.5f,0},{0.1f,0.05f,0.08f},0.006,3);
    ASSERT_TRUE(flame.bindRigidBody(id,0));
    EXPECT_TRUE(rigid.hasCombustibleRepresentation(moving));
    EXPECT_TRUE(rigid.buildWireData().positions.empty());
    const auto fuel=flame.findBody(id)->stats().fuel;
    flame.setTimeStep(0.02f); flame.render().renderScale=35; flame.setRunning(true);
    rigid.step(); EXPECT_FLOAT_EQ(moving->position.x,0); // app's ordinary rigid update is suppressed
    flame.step();
    EXPECT_NEAR(moving->position.x,0.024f,1e-6);
    EXPECT_NEAR(flame.findBody(id)->center().x,0.002f,1e-6);
    EXPECT_NEAR(std::abs(glm::dot(flame.findBody(id)->orientation(),moving->orientation)),1,1e-6);
    EXPECT_GT(std::abs(moving->orientation.z),0.01f);
    EXPECT_FLOAT_EQ(rigid.getWorld().timeStep,0.016f);
    flame.setRunning(false); rigid.step(); flame.step();
    EXPECT_NEAR(moving->position.x,0.024f,1e-6);
    rigid.stepForced(); EXPECT_NEAR(moving->position.x,0.048f,1e-6);
    EXPECT_NEAR(flame.getSimTime(),0.04f,1e-6);
    EXPECT_DOUBLE_EQ(flame.findBody(id)->stats().fuel,fuel);
    rigid.setRunning(true); EXPECT_TRUE(flame.isRunning());
    rigid.setRunning(false); EXPECT_FALSE(flame.isRunning());
}

TEST(FlameWorld, RigidHandlesDoNotRebindAfterPresetRebuild)
{
    PhysicsSolver physics; RigidBodyWorld rigid(physics); FlameWorld flame;
    flame.setRigidWorld(&rigid);
    rigid.addSphere({0,6,0},1.2f,1);
    const auto id=flame.addBody(CombustibleBody::Shape::Sphere,{0,0.5f,0},{0.1f,0.1f,0.1f},0.006,1);
    ASSERT_TRUE(flame.bindRigidBody(id,0)); const auto handle=flame.rigidBodyId(id);
    rigid.clear(); rigid.addSphere({12,6,0},1.2f,1);
    EXPECT_NE(rigid.bodyId(0),handle); flame.syncRigidBindings();
    EXPECT_FALSE(flame.hasRigidBindings()); EXPECT_FALSE(rigid.hasExternalClock());
    EXPECT_NEAR(flame.findBody(id)->center().x,0,1e-6);
    ASSERT_TRUE(flame.bindRigidBody(id,0)); ASSERT_TRUE(flame.removeBody(id));
    EXPECT_FALSE(rigid.hasExternalClock());
}

TEST(FlameWorld, FastRotationRefinesCommonSubsteps)
{
    PhysicsSolver physics; RigidBodyWorld rigid(physics); FlameWorld flame;
    flame.setRigidWorld(&rigid); rigid.getWorld().params().gravity={0,0,0};
    auto* moving=rigid.addBox({0,6,0},{1.2f,0.6f,0.96f},1);
    moving->angularVelocity={0,0,100};
    const auto id=flame.addBody(CombustibleBody::Shape::Box,{0,0.5f,0},{0.1f,0.05f,0.08f},0.006,3);
    ASSERT_TRUE(flame.bindRigidBody(id,0));
    flame.setTimeStep(0.02f); flame.stepOnce();
    EXPECT_NEAR(2*std::atan2(moving->orientation.z,moving->orientation.w),2,0.001);
    EXPECT_NEAR(flame.getSimTime(),0.02f,1e-6);
}

TEST(FlameWorld, FrameForceActsForWholeSharedInterval)
{
    PhysicsSolver physics; RigidBodyWorld rigid(physics); FlameWorld flame;
    flame.setRigidWorld(&rigid); rigid.getWorld().params().gravity={0,0,0};
    auto* moving=rigid.addSphere({0,6,0},1.2f,2);
    const auto id=flame.addBody(CombustibleBody::Shape::Sphere,{0,0.5f,0},{0.1f,0.1f,0.1f},0.006,3);
    ASSERT_TRUE(flame.bindRigidBody(id,0));
    moving->applyForce({10,0,0});
    flame.setTimeStep(0.02f); flame.stepOnce();
    EXPECT_NEAR(moving->linearVelocity.x,0.1f,1e-6);
    EXPECT_EQ(moving->forceAccum,Vector3df(0));
    flame.stepOnce(); EXPECT_NEAR(moving->linearVelocity.x,0.1f,1e-6);
}

TEST(FlameWorld, BindingValidationAndResetReleaseClock)
{
    PhysicsSolver physics; RigidBodyWorld rigid(physics); FlameWorld flame;
    flame.setRigidWorld(&rigid); rigid.addSphere({0,6,0},1.2f,1); rigid.addFloor();
    const auto box=flame.addBody(CombustibleBody::Shape::Box,{0,0,0},{0.1f,0.1f,0.1f},0.006,1);
    const auto sphere=flame.addBody(CombustibleBody::Shape::Sphere,{0,0,0},{0.1f,0.1f,0.1f},0.006,1);
    EXPECT_FALSE(flame.bindRigidBody(box,0)); EXPECT_FALSE(flame.bindRigidBody(sphere,1));
    EXPECT_FALSE(flame.bindRigidBody(sphere,100)); EXPECT_FALSE(flame.bindRigidBody(999,0));
    flame.setCanBindRigid([]{return false;}); EXPECT_FALSE(flame.bindRigidBody(sphere,0));
    flame.setCanBindRigid({}); ASSERT_TRUE(flame.bindRigidBody(sphere,0));
    auto* shape=static_cast<SphereShape*>(rigid.getWorld().getBodies()[0]->shape);
    shape->radius=std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(flame.bindRigidBody(sphere,0));
    shape->radius=1.2f;
    const auto duplicate=flame.addBody(CombustibleBody::Shape::Sphere,{0,0,0},{0.1f,0.1f,0.1f},0.006,1);
    EXPECT_FALSE(flame.bindRigidBody(duplicate,0));
    flame.reset(); EXPECT_FALSE(rigid.hasExternalClock()); EXPECT_FALSE(flame.hasRigidBindings());
    const auto badSize=flame.addBody(CombustibleBody::Shape::Sphere,{0,0,0},{0.2f,0.2f,0.2f},0.006,1);
    EXPECT_FALSE(flame.bindRigidBody(badSize,0));
}

TEST(FlameWorld, UnbindFreezesSolidAndDestructionReleasesClock)
{
    PhysicsSolver physics; RigidBodyWorld rigid(physics); rigid.addSphere({0,6,0},1.2f,1);
    {
        FlameWorld flame; flame.setRigidWorld(&rigid);
        const auto id=flame.addBody(CombustibleBody::Shape::Sphere,{0,0,0},{0.1f,0.1f,0.1f},0.006,1);
        ASSERT_TRUE(flame.bindRigidBody(id,0));
        rigid.getWorld().getBodies()[0]->position.x=3; flame.syncRigidBindings();
        const auto center=flame.findBody(id)->center();
    ASSERT_TRUE(flame.unbindRigidBody(id)); EXPECT_FALSE(rigid.hasExternalClock());
    EXPECT_FALSE(rigid.hasCombustibleRepresentation(rigid.getWorld().getBodies()[0]));
        rigid.getWorld().getBodies()[0]->position.x=6; flame.syncRigidBindings();
        EXPECT_EQ(flame.findBody(id)->center(),center);
        EXPECT_EQ(flame.findBody(id)->velocityAt(center),Vector3df(0));
        ASSERT_TRUE(flame.bindRigidBody(id,0));
    }
    EXPECT_FALSE(rigid.hasExternalClock());
}
