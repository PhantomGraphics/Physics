#include "pch.h"
#include "../PhysicsView/HairWorld.h"
#include "../PhysicsView/HairCommandDispatcher.h"
#include <algorithm>

using namespace Phantom;

TEST(HairWorldTest, CommonApiDemoUpdatesBothStylesWithRealtimeClockAndRootMotion) {
    SceneComponentRegistry registry;
    HairWorld world;
    world.setComponentRegistry(&registry);
    Physics::HairVariationParams variation{0.5f,0.02f,0.05f,2026};
    ASSERT_TRUE(world.setGenerationParams(variation));
    HairWorld::CountParams counts{32,512,64,1024};
    ASSERT_TRUE(world.setCountParams(counts));
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        SCOPED_TRACE(preset == HairPreset::LongHair ? "LongHair" : "ShortFur");
        ASSERT_TRUE(world.setPreset(preset));
        EXPECT_EQ(registry.count(SceneComponentKind::Hair),1);
        EXPECT_EQ(world.stats().steps,0u);
        EXPECT_FALSE(world.motionEnabled());
        EXPECT_FALSE(world.isRunning());
        EXPECT_FLOAT_EQ(world.params().windDrag,0.f);
        EXPECT_FLOAT_EQ(world.rigPose().position.x,0.f);
        const auto restGuides = world.strands().particles().positions;
        const auto restFollowers = world.followers().positions();
        auto settings = world.params();
        settings.windVelocity = {2.f,0.f,0.f};
        settings.windDrag = 1.f;
        ASSERT_TRUE(world.setParams(settings));
        world.setMotion(true);
        world.setRunning(true);
        for (int i = 0; i < 30; ++i) {
            ASSERT_TRUE(world.update(world.params().timeStep));
            EXPECT_EQ(world.stats().nonFiniteCount,0u);
            EXPECT_FLOAT_EQ(world.maxRootError(),0.f);
            EXPECT_LT(world.stats().maxRelativeLengthError,0.01f);
        }
        EXPECT_EQ(world.stats().steps,30u);
        EXPECT_GT(glm::length(world.strands().particles().positions.back()-restGuides.back()),1.e-5f);
        EXPECT_GT(glm::length(world.followers().positions().back()-restFollowers.back()),1.e-5f);
        world.setRunning(false);
        EXPECT_FALSE(world.update(1.));
        EXPECT_EQ(world.stats().steps,30u);
        world.setMotion(false);
        world.reset();
        for (size_t i = 0; i < restGuides.size(); ++i)
            EXPECT_LT(glm::length(world.strands().particles().positions[i]-restGuides[i]),1.e-6f);
        for (size_t i = 0; i < restFollowers.size(); ++i)
            EXPECT_LT(glm::length(world.followers().positions()[i]-restFollowers[i]),3.e-6f);
        Physics::HairRootPose pose;
        pose.position = {0.3f,0.f,0.f};
        ASSERT_TRUE(world.setRigPose(pose,true));
        EXPECT_FLOAT_EQ(world.stats().maxSpeed,0.f);
        for (size_t i = 0; i < restFollowers.size(); ++i)
            EXPECT_LT(glm::length(world.followers().positions()[i]-restFollowers[i]-pose.position),3.e-6f);
    }
    world.clear();
    EXPECT_EQ(registry.count(SceneComponentKind::Hair),0);
}

TEST(HairWorldTest, CountsAreStagedIndependentAndPersistAcrossClear) {
    HairWorld w;
    ASSERT_TRUE(w.setPreset(HairPreset::LongHair));
    ASSERT_TRUE(w.stepOnce());
    auto counts = w.countParams();
    counts.longHairGuides = 7;
    counts.longHairFollowers = 19;
    counts.shortFurGuides = 11;
    counts.shortFurFollowers = 3;
    ASSERT_TRUE(w.setCountParams(counts));
    EXPECT_EQ(w.strands().strandCount(),48u);
    EXPECT_EQ(w.followers().strandCount(),384u);
    EXPECT_EQ(w.stats().steps,1u);
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        ASSERT_TRUE(w.setPreset(preset));
        const size_t guides = preset == HairPreset::LongHair ? 7 : 11;
        const size_t followers = preset == HairPreset::LongHair ? 19 : 3;
        const size_t vertices = preset == HairPreset::LongHair ? 24 : 4;
        EXPECT_EQ(w.strands().strandCount(),guides);
        EXPECT_EQ(w.stats().particleCount,guides*vertices);
        EXPECT_EQ(w.followers().strandCount(),followers);
        EXPECT_EQ(w.followers().particleCount(),followers*vertices);
        const auto wire = w.buildWireData();
        EXPECT_EQ(wire.positions.size(),(guides+followers)*vertices*3);
        EXPECT_EQ(wire.indices.size(),(guides+followers)*(vertices-1)*2);
        ASSERT_TRUE(w.stepOnce());
        EXPECT_EQ(w.stats().nonFiniteCount,0u);
        w.reset();
        EXPECT_EQ(w.followers().strandCount(),followers);
    }
    ASSERT_TRUE(w.setPreset(HairPreset::Single));
    EXPECT_EQ(w.strands().strandCount(),1u);
    EXPECT_EQ(w.followers().strandCount(),0u);
    w.clear();
    ASSERT_TRUE(w.setPreset(HairPreset::LongHair));
    EXPECT_EQ(w.strands().strandCount(),7u);
    EXPECT_EQ(w.followers().strandCount(),19u);
}

TEST(HairWorldTest, FollowerDensityDoesNotChangeGuideMotionAndCanBeZero) {
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        HairWorld a, b;
        auto counts = a.countParams();
        counts.longHairGuides = counts.shortFurGuides = 1;
        counts.longHairFollowers = counts.shortFurFollowers = 0;
        ASSERT_TRUE(a.setCountParams(counts));
        counts.longHairFollowers = counts.shortFurFollowers = 17;
        ASSERT_TRUE(b.setCountParams(counts));
        ASSERT_TRUE(a.setPreset(preset));
        ASSERT_TRUE(b.setPreset(preset));
        EXPECT_EQ(a.followers().particleCount(),0u);
        EXPECT_EQ(b.followers().strandCount(),17u);
        auto p = a.params();
        p.windVelocity = {2.f,0.f,0.f}; p.windDrag = 1.f;
        ASSERT_TRUE(a.setParams(p));
        ASSERT_TRUE(b.setParams(p));
        for (int step = 0; step < 5; ++step) {
            ASSERT_TRUE(a.stepOnce());
            ASSERT_TRUE(b.stepOnce());
            ASSERT_EQ(a.strands().particleCount(),b.strands().particleCount());
            for (size_t i = 0; i < a.strands().particleCount(); ++i)
                EXPECT_LT(glm::length(a.strands().particles().positions[i]-b.strands().particles().positions[i]),1.e-7f);
        }
        EXPECT_EQ(a.buildWireData().positions.size(),a.strands().particleCount()*3);
        a.reset();
        EXPECT_EQ(a.followers().strandCount(),0u);
    }
}

TEST(HairCommandDispatcherTest, CountSettingsValidateIntegersAndPreserveStateOnFailure) {
    HairWorld w;
    HairCommandDispatcher d;
    d.setWorld(&w);
    EXPECT_EQ(d.route("HairPreset:LongHair"),"OK");
    EXPECT_EQ(d.route("HairStep"),"OK");
    EXPECT_EQ(d.route("SetHairGenerationParam:longHairGuides,5"),"OK");
    EXPECT_EQ(d.route("SetHairGenerationParam:longHairFollowers,13"),"OK");
    EXPECT_EQ(d.route("SetHairGenerationParam:shortFurGuides,1"),"OK");
    EXPECT_EQ(d.route("SetHairGenerationParam:shortFurFollowers,0"),"OK");
    EXPECT_EQ(d.route("GetHairStat:guides"),"48");
    EXPECT_EQ(d.route("GetHairStat:steps"),"1");
    for (const char* command : {"SetHairGenerationParam:longHairGuides,0",
        "SetHairGenerationParam:shortFurGuides,-1", "SetHairGenerationParam:longHairFollowers,-1",
        "SetHairGenerationParam:shortFurFollowers,0.5", "SetHairGenerationParam:longHairGuides,10001",
        "SetHairGenerationParam:shortFurFollowers,10001", "SetHairGenerationParam:longHairFollowers,1e100",
        "SetHairGenerationParam:longHairFollowers,nan", "SetHairGenerationParam:longHairGuides,5,6"}) {
        const auto result = d.route(command);
        ASSERT_TRUE(result.has_value());
        EXPECT_EQ(result->find("Error:"),0u) << command;
    }
    EXPECT_EQ(d.route("GetHairGenerationParam:longHairGuides"),"5");
    EXPECT_EQ(d.route("GetHairGenerationParam:longHairFollowers"),"13");
    EXPECT_EQ(d.route("GetHairGenerationParam:shortFurGuides"),"1");
    EXPECT_EQ(d.route("GetHairGenerationParam:shortFurFollowers"),"0");
    EXPECT_EQ(d.route("GetHairStat:steps"),"1");
    EXPECT_EQ(d.route("HairPreset:LongHair"),"OK");
    EXPECT_EQ(d.route("GetHairStat:particles"),"120");
    EXPECT_EQ(d.route("GetHairStat:followers"),"13");
    EXPECT_EQ(d.route("HairPreset:ShortFur"),"OK");
    EXPECT_EQ(d.route("GetHairStat:particles"),"4");
    EXPECT_EQ(d.route("GetHairStat:followers"),"0");
    auto counts = w.countParams();
    counts.longHairGuides = counts.shortFurGuides = 10000;
    counts.longHairFollowers = counts.shortFurFollowers = 10000;
    EXPECT_TRUE(w.setCountParams(counts));
    counts.longHairGuides = 10001;
    EXPECT_FALSE(w.setCountParams(counts));
    EXPECT_EQ(w.countParams().longHairGuides,10000);
    EXPECT_EQ(w.strands().particleCount(),4u);
}

TEST(HairWorldTest, FollowersRenderSeparateStrandsAndTrackStepResetTeleportAndClear) {
    for (auto preset : {HairPreset::LongHair,HairPreset::ShortFur}) {
        HairWorld w;
        ASSERT_TRUE(w.setPreset(preset));
        const auto guideCount = w.strands().strandCount();
        const auto guideParticles = w.strands().particleCount();
        const auto initial = w.followers().positions();
        const auto count = w.followers().particlesPerStrand();
        ASSERT_EQ(w.followers().strandCount(),guideCount*8);
        ASSERT_EQ(initial.size(),guideParticles*8);
        const auto wire = w.buildWireData();
        EXPECT_EQ(wire.positions.size(),guideParticles*9*3);
        EXPECT_EQ(wire.colors.size(),guideParticles*9*4);
        EXPECT_EQ(wire.indices.size(),guideCount*9*(count-1)*2);
        for (size_t i = 0; i < wire.indices.size(); i += 2) {
            EXPECT_EQ(wire.indices[i]/count,wire.indices[i+1]/count);
            EXPECT_LT(wire.indices[i+1],guideParticles*9);
        }
        auto p = w.params();
        p.windVelocity = {2.f,0.f,0.f}; p.windDrag = 1.f;
        ASSERT_TRUE(w.setParams(p));
        for (int i = 0; i < 5; ++i) ASSERT_TRUE(w.stepOnce());
        EXPECT_GT(glm::length(w.followers().positions().back()-initial.back()),1.e-5f);
        EXPECT_EQ(w.stats().particleCount,guideParticles);
        w.reset();
        for (size_t i = 0; i < initial.size(); ++i)
            EXPECT_LT(glm::length(w.followers().positions()[i]-initial[i]),2.e-6f);
        Physics::HairRootPose pose;
        pose.position = {1.f,0.2f,0.f};
        pose.rotation = glm::angleAxis(0.5f,Math::Vector3df(0.f,1.f,0.f));
        ASSERT_TRUE(w.setRigPose(pose,true));
        const Math::Vector3df pivot(0.f,1.05f,0.f);
        for (size_t i = 0; i < initial.size(); ++i) {
            const auto expected = pivot+pose.position+pose.rotation*(initial[i]-pivot);
            EXPECT_LT(glm::length(w.followers().positions()[i]-expected),3.e-6f);
        }
        ASSERT_TRUE(w.setPreset(HairPreset::Single));
        EXPECT_EQ(w.followers().strandCount(),0u);
        EXPECT_EQ(w.followers().particleCount(),0u);
        w.clear();
        EXPECT_TRUE(w.buildWireData().positions.empty());
    }
}

TEST(HairWorldTest, StylePresetsSetGeometryAndRestoreSolverSettings) {
    HairWorld w;
    ASSERT_TRUE(w.setPreset(HairPreset::StrongWind));
    ASSERT_TRUE(w.setPreset(HairPreset::ShortFur));
    EXPECT_EQ(w.strands().strandCount(), 96u);
    EXPECT_EQ(w.strands().particleCount(), 384u);
    EXPECT_EQ(w.colliderCount(), 0u);
    EXPECT_FALSE(w.isRunning());
    EXPECT_FALSE(w.motionEnabled());
    EXPECT_FLOAT_EQ(w.params().windDrag, 0.f);
    EXPECT_FLOAT_EQ(glm::length(w.params().windVelocity), 0.f);
    const float furShape = w.params().shapeCompliance;
    for (const auto& r : w.strands().ranges()) {
        float length = 0.f;
        for (size_t j = 1; j < r.count; ++j)
            length += glm::length(w.strands().particles().positions[r.offset+j] -
                                  w.strands().particles().positions[r.offset+j-1]);
        EXPECT_NEAR(length, 0.06f, 1.e-6f);
        const auto axisPoint = Math::Vector3df(0.f,1.05f,std::clamp(r.root.position.z,-0.4f,0.4f));
        const auto normal = glm::normalize(r.root.position-axisPoint);
        EXPECT_GT(glm::dot(w.strands().particles().positions[r.offset+r.count-1]-r.root.position,normal),0.059f);
    }
    ASSERT_TRUE(w.setPreset(HairPreset::LongHair));
    EXPECT_EQ(w.strands().strandCount(), 48u);
    EXPECT_EQ(w.strands().particleCount(), 1152u);
    EXPECT_GT(w.params().shapeCompliance, furShape);
    EXPECT_EQ(w.params().numIterations, 8);
    const auto& r = w.strands().ranges().front();
    float length = 0.f;
    for (size_t j = 1; j < r.count; ++j)
        length += glm::length(w.strands().particles().positions[r.offset+j] -
                              w.strands().particles().positions[r.offset+j-1]);
    EXPECT_NEAR(length, 1.2f, 1.e-5f);
    EXPECT_LT(w.strands().particles().positions[r.offset+r.count-1].y, r.root.position.y);
}

TEST(HairWorldTest, StylePresetsRemainStableInWindAndResetToRest) {
    for (const auto preset : {HairPreset::LongHair, HairPreset::ShortFur}) {
        HairWorld w;
        ASSERT_TRUE(w.setPreset(preset));
        const auto initial = w.strands().particles().positions;
        auto p = w.params();
        p.windVelocity = {2.f, 0.f, 0.f};
        p.windDrag = 1.f;
        ASSERT_TRUE(w.setParams(p));
        for (int i = 0; i < 60; ++i) {
            ASSERT_TRUE(w.stepOnce());
            EXPECT_LT(w.stats().maxRelativeLengthError, 0.01f);
            EXPECT_EQ(w.stats().nonFiniteCount, 0u);
            EXPECT_FLOAT_EQ(w.maxRootError(), 0.f);
        }
        EXPECT_GT(glm::length(w.strands().particles().positions.back()-initial.back()), 1.e-5f);
        w.reset();
        EXPECT_EQ(w.stats().steps, 0u);
        EXPECT_FLOAT_EQ(w.stats().maxSpeed, 0.f);
        for (size_t i = 0; i < initial.size(); ++i)
            EXPECT_LT(glm::length(w.strands().particles().positions[i]-initial[i]), 1.e-6f);
    }
}

TEST(HairCommandDispatcherTest, StylePresetSwitchClearsMotionWindAndClock) {
    HairWorld w;
    HairCommandDispatcher d;
    d.setWorld(&w);
    EXPECT_EQ(d.route("HairPreset:StrongWind"), "OK");
    EXPECT_EQ(d.route("SetHairMotion:true"), "OK");
    EXPECT_EQ(d.route("HairStep"), "OK");
    EXPECT_EQ(d.route("HairPreset:ShortFur"), "OK");
    EXPECT_EQ(d.route("GetHairStat:particles"), "384");
    EXPECT_EQ(d.route("GetHairStat:colliders"), "0");
    EXPECT_EQ(d.route("GetHairStat:steps"), "0");
    EXPECT_EQ(d.route("IsHairMotion"), "false");
    EXPECT_EQ(d.route("GetHairParam:windX"), "0");
    EXPECT_EQ(d.route("HairPreset:LongHair"), "OK");
    EXPECT_EQ(d.route("GetHairStat:particles"), "1152");
    EXPECT_EQ(d.route("GetHairParam:iterations"), "8");
}

TEST(HairWorldTest, FixedClockPauseStepResetAndCatchup) {
    HairWorld w;
    ASSERT_TRUE(w.setPreset(HairPreset::Single));
    EXPECT_FALSE(w.update(1.));
    EXPECT_EQ(w.stats().steps, 0u);
    ASSERT_TRUE(w.stepOnce());
    w.reset();
    w.setRunning(true);
    const double dt = w.params().timeStep;
    EXPECT_FALSE(w.update(dt*0.5));
    EXPECT_TRUE(w.update(dt*0.5));
    EXPECT_EQ(w.stats().steps, 1u);
    w.update(10.);
    EXPECT_EQ(w.stats().steps, 9u);
    EXPECT_NEAR(w.droppedTime(), 10.-8.*dt, 1.e-8);
    w.setRunning(false);
    w.update(100.);
    w.setRunning(true);
    EXPECT_FALSE(w.update(dt*0.5));
    w.reset();
    EXPECT_EQ(w.stats().steps, 0u);
    EXPECT_EQ(w.droppedTime(), 0.);
}

TEST(HairWorldTest, WireIndicesDoNotJoinDifferentStrandsAndClearUnregisters) {
    SceneComponentRegistry registry;
    HairWorld w;
    w.setComponentRegistry(&registry);
    w.setPreset(HairPreset::Bundle);
    EXPECT_EQ(registry.count(SceneComponentKind::Hair), 1);
    const auto wire = w.buildWireData();
    EXPECT_EQ(wire.indices.size(), 48u*23u*2u);
    EXPECT_EQ(wire.positions.size(), 48u*24u*3u);
    EXPECT_EQ(wire.colors.size(), 48u*24u*4u);
    for (size_t i = 3; i < wire.colors.size(); i += 4) EXPECT_FLOAT_EQ(wire.colors[i], 1.f);
    for (size_t i = 0; i < wire.indices.size(); i += 2)
        EXPECT_EQ(wire.indices[i]/24u, wire.indices[i+1]/24u);
    w.clear();
    EXPECT_EQ(registry.count(SceneComponentKind::Hair), 0);
    EXPECT_TRUE(w.buildWireData().indices.empty());
    EXPECT_EQ(w.stats().particleCount, 0u);
}

TEST(HairCommandDispatcherTest, CommandsAndInvalidInputsUseSameWorld) {
    HairWorld w;
    HairCommandDispatcher d;
    d.setWorld(&w);
    bool page = false;
    d.setPageHooks([&](bool v) { page = v; }, [&] { return page; });
    EXPECT_FALSE(d.route("OtherCommand").has_value());
    EXPECT_EQ(d.route("SetHairPage:true"), "OK");
    EXPECT_EQ(d.route("IsHairPage"), "true");
    EXPECT_EQ(d.route("HairPreset:Single"), "OK");
    EXPECT_EQ(d.route("GetHairStat:particles"), "24");
    EXPECT_EQ(d.route("HairStep:60"), "OK");
    EXPECT_EQ(d.route("GetHairStat:steps"), "60");
    EXPECT_EQ(d.route("GetHairStat:rootError"), "0");
    for (const char* command : {"HairStep:1.5", "HairStep:0", "HairStep:1001", "HairStep:nan",
        "HairPreset:Unknown", "SetHairParam:substeps,0", "SetHairParam:iterations,1.5",
        "SetHairParam:shapeCompliance,nan", "SetHairRunning:perhaps", "HairReset:extra"}) {
        const auto result = d.route(command);
        ASSERT_TRUE(result.has_value());
        EXPECT_EQ(result->find("Error:"), 0u) << command;
    }
    EXPECT_EQ(d.route("SetHairParam:substeps,8"), "OK");
    EXPECT_EQ(d.route("GetHairParam:substeps"), "8");
    EXPECT_EQ(d.route("SetHairRunning:true"), "OK");
    EXPECT_EQ(d.route("IsHairRunning"), "true");
    EXPECT_EQ(d.route("SetHairRunning:false"), "OK");
    EXPECT_EQ(d.route("IsHairRunning"), "false");
    EXPECT_EQ(d.route("HairReset"), "OK");
    EXPECT_EQ(d.route("GetHairStat:steps"), "0");
    EXPECT_EQ(d.route("HairClear"), "OK");
    EXPECT_EQ(d.route("GetHairStat:particles"), "0");
}

TEST(HairCommandDispatcherTest, BodyMotionWindAndTeleportShareValidatedState) {
    HairWorld world;
    HairCommandDispatcher dispatcher;
    dispatcher.setWorld(&world);
    EXPECT_EQ(dispatcher.route("HairPreset:StrongWind"), "OK");
    EXPECT_EQ(dispatcher.route("GetHairStat:colliders"), "2");
    EXPECT_EQ(dispatcher.route("GetHairParam:windX"), "12");
    EXPECT_EQ(dispatcher.route("SetHairMotion:true"), "OK");
    EXPECT_EQ(dispatcher.route("IsHairMotion"), "true");
    EXPECT_EQ(dispatcher.route("SetHairMotion:false"), "OK");
    EXPECT_EQ(dispatcher.route("SetHairFriction:0.6"), "OK");
    EXPECT_NEAR(world.friction(), .6f, 1.e-6f);
    EXPECT_EQ(dispatcher.route("SetHairRootPose:0.05,0,0,1,0,0,0"), "OK");
    EXPECT_EQ(dispatcher.route("HairStep"), "OK");
    EXPECT_EQ(dispatcher.route("GetHairStat:rootError"), "0");
    EXPECT_EQ(dispatcher.route("HairTeleport:1,0,0,1,0,0,0"), "OK");
    EXPECT_EQ(dispatcher.route("GetHairStat:maxSpeed"), "0");
    for (const char* command : {"SetHairRootPose:0,0,0,0,0,0,0", "HairTeleport:0,0,0",
        "SetHairRootPose:0,0,0,1,0,0,nan", "SetHairRootPose:0,0,0,1,0,0,0,1",
        "SetHairFriction:2", "SetHairMotion:maybe", "SetHairParam:windDrag,-1"}) {
        const auto result = dispatcher.route(command);
        ASSERT_TRUE(result.has_value());
        EXPECT_EQ(result->find("Error:"), 0u) << command;
    }
    EXPECT_FLOAT_EQ(world.rigPose().position.x, 1.f);
    EXPECT_NEAR(world.friction(), .6f, 1.e-6f);
}
