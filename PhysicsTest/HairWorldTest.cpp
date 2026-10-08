#include "pch.h"
#include "../PhysicsView/HairWorld.h"
#include "../PhysicsView/HairCommandDispatcher.h"

using namespace Phantom;

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
