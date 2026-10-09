#include "pch.h"
#include "../PhysicsView/HairWorld.h"
#include "../PhysicsView/HairCommandDispatcher.h"
#include <limits>

using namespace Phantom;

TEST(HairLodTest, ShapeAndDensityAreContinuousOnSwitchAndReversalWhilePaused) {
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        HairWorld world;
        ASSERT_TRUE(world.setPreset(preset));
        ASSERT_TRUE(world.stepOnce());
        const auto before = world.buildWireData();
        const auto time = world.stats().simulatedTime;
        HairWorld::LodParams lod;
        lod.enabled = true;
        lod.transitionSeconds = 0.2f;
        ASSERT_TRUE(world.setLodParams(lod));
        ASSERT_TRUE(world.setCameraDistance(15.f));
        EXPECT_FLOAT_EQ(world.lodTransitionProgress(),0.f);
        const auto start = world.buildWireData();
        // Every original guide vertex remains on the initial displayed polyline.
        const size_t guides = world.strands().strandCount();
        const size_t originalCount = preset == HairPreset::LongHair ? 24 : 4;
        std::vector<float> grid;
        for (size_t stride : {size_t{1},size_t{2},size_t{4}}) {
            const size_t count = (originalCount-2+stride)/stride+1;
            for (size_t j = 0; j < count; ++j) grid.push_back(static_cast<float>(j)/(count-1));
        }
        std::sort(grid.begin(),grid.end());
        grid.erase(std::unique(grid.begin(),grid.end()),grid.end());
        for (size_t s = 0; s < guides; ++s) for (size_t j = 0; j < originalCount; ++j) {
            const float t = static_cast<float>(j)/(originalCount-1);
            const size_t k = static_cast<size_t>(std::find(grid.begin(),grid.end(),t)-grid.begin());
            for (size_t axis = 0; axis < 3; ++axis)
                EXPECT_NEAR(start.positions[(s*grid.size()+k)*3+axis],before.positions[(s*originalCount+j)*3+axis],2.e-6f);
        }
        for (size_t i = 3; i < start.colors.size(); i += 4) EXPECT_FLOAT_EQ(start.colors[i],1.f);
        ASSERT_TRUE(world.update(0.1)); // paused physics, display transition still advances
        EXPECT_NEAR(world.lodTransitionProgress(),0.5f,1.e-6f);
        EXPECT_DOUBLE_EQ(world.stats().simulatedTime,time);
        const auto middle = world.buildWireData();
        const size_t followerFirst = guides*grid.size();
        for (size_t s = 0; s < world.followers().strandCount(); ++s)
            EXPECT_NEAR(middle.colors[(followerFirst+s*originalCount)*4+3],s%4 == 0 ? 1.f : 0.5f,1.e-6f);
        ASSERT_TRUE(world.setCameraDistance(0.f));
        const auto reversed = world.buildWireData();
        ASSERT_EQ(reversed.positions.size(),middle.positions.size());
        for (size_t i = 0; i < middle.positions.size(); ++i) EXPECT_NEAR(reversed.positions[i],middle.positions[i],2.e-6f);
        for (size_t i = 3; i < middle.colors.size(); i += 4) EXPECT_FLOAT_EQ(reversed.colors[i],middle.colors[i]);
        ASSERT_TRUE(world.update(0.21));
        EXPECT_FLOAT_EQ(world.lodTransitionProgress(),1.f);
        EXPECT_EQ(world.buildWireData().positions.size(),before.positions.size());
        EXPECT_DOUBLE_EQ(world.stats().simulatedTime,time);
    }
}

TEST(HairLodTest, TransitionCancelsForTeleportResetAndClearAndValidatesDuration) {
    HairWorld world;
    HairCommandDispatcher d;
    d.setWorld(&world);
    ASSERT_TRUE(world.setPreset(HairPreset::ShortFur));
    EXPECT_EQ(d.route("SetHairLodParam:transitionSeconds,0.2"),"OK");
    for (auto cmd : {"SetHairLodParam:transitionSeconds,-1", "SetHairLodParam:transitionSeconds,3",
                     "SetHairLodParam:transitionSeconds,nan"}) {
        auto result = d.route(cmd);
        ASSERT_TRUE(result); EXPECT_EQ(result->find("Error:"),0u);
    }
    EXPECT_NEAR(world.lodParams().transitionSeconds,0.2f,1.e-6f);
    EXPECT_EQ(d.route("SetHairLodParam:enabled,1"),"OK");
    ASSERT_TRUE(world.setCameraDistance(15.f));
    EXPECT_FLOAT_EQ(world.lodTransitionProgress(),0.f);
    EXPECT_EQ(d.route("HairTeleport:1,0,0,1,0,0,0"),"OK");
    EXPECT_EQ(d.route("GetHairLodStat:transitionProgress"),"1");
    ASSERT_TRUE(world.setCameraDistance(0.f));
    world.reset();
    EXPECT_FLOAT_EQ(world.lodTransitionProgress(),1.f);
    ASSERT_TRUE(world.setCameraDistance(15.f));
    world.clear();
    EXPECT_TRUE(world.buildWireData().positions.empty());
    EXPECT_FLOAT_EQ(world.lodTransitionProgress(),1.f);
}

TEST(HairLodTest, RepeatedReversalKeepsBoundedGeometryAndAnimatedRootsAttached) {
    HairWorld world;
    ASSERT_TRUE(world.setPreset(HairPreset::LongHair));
    HairWorld::LodParams lod;
    lod.enabled = true;
    lod.transitionSeconds = 0.2f;
    ASSERT_TRUE(world.setLodParams(lod));
    ASSERT_TRUE(world.setCameraDistance(15.f));
    const auto size = world.buildWireData().positions.size();
    for (int i = 0; i < 20; ++i) {
        ASSERT_TRUE(world.update(0.01));
        const auto previous = world.buildWireData();
        ASSERT_TRUE(world.setCameraDistance(i%2 == 0 ? 0.f : 15.f));
        const auto next = world.buildWireData();
        ASSERT_EQ(next.positions.size(),size);
        for (size_t j = 0; j < next.positions.size(); ++j) EXPECT_NEAR(next.positions[j],previous.positions[j],2.e-6f);
    }
    world.setMotion(true);
    world.setRunning(true);
    ASSERT_TRUE(world.update(4.*world.params().timeStep));
    const auto displayed = world.buildWireData();
    // The canonical grid for a 24-particle guide has a constant count during a transition.
    std::vector<float> grid;
    for (size_t count : {size_t{24},size_t{13},size_t{7}})
        for (size_t j = 0; j < count; ++j) grid.push_back(static_cast<float>(j)/(count-1));
    std::sort(grid.begin(),grid.end());
    grid.erase(std::unique(grid.begin(),grid.end()),grid.end());
    for (size_t s = 0; s < world.strands().strandCount(); ++s) {
        const Math::Vector3df root(displayed.positions[s*grid.size()*3],displayed.positions[s*grid.size()*3+1],displayed.positions[s*grid.size()*3+2]);
        EXPECT_LT(glm::length(root-world.strands().ranges()[s].root.position),1.e-6f);
    }
}

TEST(HairLodTest, DistanceHysteresisChangesParticlesAndDrawingWithoutResettingClock) {
    HairWorld world;
    ASSERT_TRUE(world.setPreset(HairPreset::LongHair));
    ASSERT_TRUE(world.stepOnce());
    const auto tip = world.strands().particles().positions.back();
    const auto velocity = world.strands().particles().velocities.back();
    const auto time = world.stats().simulatedTime;
    HairWorld::LodParams p;
    p.enabled = true;
    p.transitionSeconds = 0.f;
    ASSERT_TRUE(world.setLodParams(p));
    ASSERT_TRUE(world.setCameraDistance(7.f));
    EXPECT_EQ(world.lodLevel(),1);
    EXPECT_EQ(world.strands().particleCount(),48u*13u);
    EXPECT_EQ(world.drawnFollowerCount(),192u);
    EXPECT_EQ(world.stats().steps,1u);
    EXPECT_DOUBLE_EQ(world.stats().simulatedTime,time);
    EXPECT_LT(glm::length(world.strands().particles().positions.back()-tip),1.e-6f);
    EXPECT_LT(glm::length(world.strands().particles().velocities.back()-velocity),1.e-6f);
    ASSERT_TRUE(world.setCameraDistance(6.1f));
    EXPECT_EQ(world.lodLevel(),1);
    ASSERT_TRUE(world.setCameraDistance(15.f));
    EXPECT_EQ(world.lodLevel(),2);
    EXPECT_EQ(world.strands().particleCount(),48u*7u);
    EXPECT_EQ(world.drawnFollowerCount(),96u);
    const auto wire = world.buildWireData();
    EXPECT_EQ(wire.positions.size(),(48u*7u+96u*7u)*3u);
    EXPECT_EQ(wire.indices.size(),(48u+96u)*6u*2u);
    for (size_t i = 0; i < wire.indices.size(); i += 2) {
        EXPECT_EQ(wire.indices[i]/7u,wire.indices[i+1]/7u);
        EXPECT_LT(wire.indices[i+1],wire.positions.size()/3);
    }
    ASSERT_TRUE(world.setCameraDistance(12.f));
    EXPECT_EQ(world.lodLevel(),2);
    ASSERT_TRUE(world.setCameraDistance(11.f));
    EXPECT_EQ(world.lodLevel(),1);
    ASSERT_TRUE(world.setCameraDistance(4.f));
    EXPECT_EQ(world.lodLevel(),0);
    EXPECT_EQ(world.strands().particleCount(),1152u);
    EXPECT_EQ(world.followers().strandCount(),384u);
    EXPECT_EQ(world.stats().steps,1u);
    EXPECT_LT(glm::length(world.strands().particles().positions.back()-tip),1.e-6f);
}

TEST(HairLodTest, RealtimeFrequencyPreservesElapsedTimeAndManualStepUsesBaseDt) {
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        HairWorld world;
        ASSERT_TRUE(world.setPreset(preset));
        HairWorld::LodParams p;
        p.enabled = true;
        p.transitionSeconds = 0.f;
        ASSERT_TRUE(world.setLodParams(p));
        ASSERT_TRUE(world.setCameraDistance(15.f));
        world.setRunning(true);
        const double dt = world.params().timeStep;
        EXPECT_FALSE(world.update(dt));
        EXPECT_FALSE(world.update(dt));
        EXPECT_FALSE(world.update(dt));
        EXPECT_TRUE(world.update(dt));
        EXPECT_EQ(world.stats().steps,1u);
        EXPECT_NEAR(world.stats().simulatedTime,4.*dt,1.e-8);
        EXPECT_EQ(world.stats().nonFiniteCount,0u);
        EXPECT_FLOAT_EQ(world.maxRootError(),0.f);
        EXPECT_LT(world.stats().maxRelativeLengthError,0.01f);
        world.setRunning(false);
        ASSERT_TRUE(world.stepOnce());
        EXPECT_NEAR(world.stats().simulatedTime,5.*dt,1.e-8);
        world.reset();
        p.enabled = false;
        ASSERT_TRUE(world.setLodParams(p));
        EXPECT_EQ(world.lodLevel(),0);
        EXPECT_EQ(world.strands().particleCount(),preset == HairPreset::LongHair ? 1152u : 384u);
    }
}

TEST(HairLodTest, StaticRestRoundtripAndPresetClearKeepOriginalTopology) {
    HairWorld world;
    ASSERT_TRUE(world.setPreset(HairPreset::LongHair));
    const auto rest = world.strands().particles().positions;
    HairWorld::LodParams p;
    p.enabled = true;
    p.transitionSeconds = 0.f;
    ASSERT_TRUE(world.setLodParams(p));
    ASSERT_TRUE(world.setCameraDistance(15.f));
    ASSERT_TRUE(world.setCameraDistance(0.f));
    for (size_t i = 0; i < rest.size(); ++i)
        EXPECT_LT(glm::length(world.strands().particles().positions[i]-rest[i]),1.e-6f);
    ASSERT_TRUE(world.setCameraDistance(15.f));
    ASSERT_TRUE(world.setPreset(HairPreset::ShortFur));
    EXPECT_EQ(world.strands().particleCount(),96u*2u);
    Physics::HairRootPose pose;
    pose.position = {1.f,0.f,0.f};
    ASSERT_TRUE(world.setRigPose(pose,true));
    ASSERT_TRUE(world.setCameraDistance(0.f));
    EXPECT_EQ(world.strands().particleCount(),384u);
    EXPECT_FLOAT_EQ(world.maxRootError(),0.f);
    ASSERT_TRUE(world.stepOnce());
    EXPECT_EQ(world.stats().nonFiniteCount,0u);
    world.clear();
    ASSERT_TRUE(world.setCameraDistance(15.f));
    EXPECT_TRUE(world.buildWireData().positions.empty());
    ASSERT_TRUE(world.setPreset(HairPreset::Single));
    EXPECT_EQ(world.strands().particleCount(),7u);
}

TEST(HairLodTest, InvalidParametersAndDynamicStatePreserveExistingState) {
    HairWorld world;
    HairCommandDispatcher dispatcher;
    dispatcher.setWorld(&world);
    ASSERT_TRUE(world.setPreset(HairPreset::Single));
    for (auto cmd : {"SetHairLodParam:enabled,2", "SetHairLodParam:mediumDistance,0",
                     "SetHairLodParam:farDistance,5", "SetHairLodParam:hysteresis,-1",
                     "SetHairLodParam:mediumDistance,nan", "GetHairLodParam:unknown"}) {
        const auto r = dispatcher.route(cmd);
        ASSERT_TRUE(r); EXPECT_EQ(r->find("Error:"),0u);
    }
    EXPECT_FALSE(world.setCameraDistance(-1.f));
    EXPECT_FALSE(world.setCameraDistance(std::numeric_limits<float>::infinity()));
    EXPECT_EQ(world.strands().particleCount(),24u);
    EXPECT_EQ(dispatcher.route("GetHairLodParam:enabled"),"0");
    EXPECT_EQ(dispatcher.route("GetHairLodStat:level"),"0");
    Physics::HairStrands strands;
    ASSERT_TRUE(Physics::generateHairBundle(strands,{}));
    const auto initial = strands.particles().positions;
    auto positions = initial;
    auto velocities = strands.particles().velocities;
    positions.back().x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(strands.setDynamicState(positions,velocities));
    EXPECT_EQ(strands.particles().positions,initial);
    positions = initial;
    velocities.front().x = 1.f;
    EXPECT_FALSE(strands.setDynamicState(positions,velocities));
    EXPECT_EQ(strands.particles().positions,initial);
}

TEST(HairLodTest, CoarseStylesRemainStableInWindAndRootMotion) {
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        HairWorld world;
        ASSERT_TRUE(world.setPreset(preset));
        HairWorld::LodParams lod;
        lod.enabled = true;
        lod.transitionSeconds = 0.f;
        ASSERT_TRUE(world.setLodParams(lod));
        ASSERT_TRUE(world.setCameraDistance(15.f));
        auto p = world.params();
        p.windVelocity = {2.f,0.f,0.f};
        p.windDrag = 1.f;
        ASSERT_TRUE(world.setParams(p));
        world.setMotion(true);
        world.setRunning(true);
        for (int i = 0; i < 30; ++i) {
            ASSERT_TRUE(world.update(4.*world.params().timeStep));
            EXPECT_EQ(world.stats().nonFiniteCount,0u);
            EXPECT_FLOAT_EQ(world.maxRootError(),0.f);
            EXPECT_LT(world.stats().maxRelativeLengthError,0.01f);
        }
        const auto time = world.stats().simulatedTime;
        const auto tip = world.strands().particles().positions.back();
        ASSERT_TRUE(world.setCameraDistance(0.f));
        EXPECT_DOUBLE_EQ(world.stats().simulatedTime,time);
        EXPECT_LT(glm::length(world.strands().particles().positions.back()-tip),1.e-6f);
        ASSERT_TRUE(world.update(world.params().timeStep));
        EXPECT_EQ(world.stats().nonFiniteCount,0u);
        EXPECT_FLOAT_EQ(world.maxRootError(),0.f);
    }
}
