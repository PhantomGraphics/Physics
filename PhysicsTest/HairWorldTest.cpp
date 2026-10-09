#include "pch.h"
#include "../PhysicsView/HairWorld.h"
#include "../PhysicsView/HairCommandDispatcher.h"
#include <algorithm>

using namespace Phantom;

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
