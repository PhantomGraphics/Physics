#include "pch.h"
#include "../PhysicsView/CommandParse.h"
#include "../PhysicsView/HairWorld.h"
#include "../PhysicsView/HairCommandDispatcher.h"

#include <limits>

using namespace Phantom;
using namespace Phantom::CommandParse;

TEST(CommandParseTest, FloatRequiresWholeTextAndFiniteValue) {
    float v = -7.0f;
    EXPECT_TRUE(parseFloat("1.5", v));
    EXPECT_FLOAT_EQ(v, 1.5f);
    EXPECT_TRUE(parseFloat("-2e-3", v));
    EXPECT_FLOAT_EQ(v, -2e-3f);

    v = 42.0f;
    for (const char* bad : {"", "abc", "1.5x", " 1", "1 ", "+1", "1,2", "nan", "inf", "-inf", "1e999"}) {
        EXPECT_FALSE(parseFloat(bad, v)) << bad;
        EXPECT_FLOAT_EQ(v, 42.0f) << "out-parameter must be untouched on failure: " << bad;
    }
}

TEST(CommandParseTest, DoubleRequiresWholeTextAndFiniteValue) {
    double v = 0.0;
    EXPECT_TRUE(parseDouble("0.25", v));
    EXPECT_DOUBLE_EQ(v, 0.25);
    for (const char* bad : {"", "0.25abc", "nan", "inf", "1e999"})
        EXPECT_FALSE(parseDouble(bad, v)) << bad;
}

TEST(CommandParseTest, IntRejectsFractionsTrailingGarbageAndOverflow) {
    int v = 5;
    EXPECT_TRUE(parseInt("-12", v));
    EXPECT_EQ(v, -12);
    v = 5;
    for (const char* bad : {"", "1.5", "12x", " 3", "+3", "2147483648", "-2147483649"}) {
        EXPECT_FALSE(parseInt(bad, v)) << bad;
        EXPECT_EQ(v, 5) << bad;
    }
}

TEST(CommandParseTest, RangeVariantsAreInclusiveAndLeaveOutputOnFailure) {
    float f = 9.0f;
    EXPECT_TRUE(parseFloatIn("0", 0.0f, 0.5f, f));
    EXPECT_TRUE(parseFloatIn("0.5", 0.0f, 0.5f, f));
    EXPECT_FLOAT_EQ(f, 0.5f);
    EXPECT_FALSE(parseFloatIn("0.51", 0.0f, 0.5f, f));
    EXPECT_FALSE(parseFloatIn("-0.01", 0.0f, 0.5f, f));
    EXPECT_FLOAT_EQ(f, 0.5f);

    int i = 9;
    EXPECT_TRUE(parseIntIn("1", 1, 4, i));
    EXPECT_FALSE(parseIntIn("0", 1, 4, i));
    EXPECT_FALSE(parseIntIn("5", 1, 4, i));
    EXPECT_EQ(i, 1);
    EXPECT_TRUE(parseCount("3", 10, i));
    EXPECT_FALSE(parseCount("0", 10, i));
    EXPECT_FALSE(parseCount("11", 10, i));
}

TEST(CommandParseTest, SplitKeepsEmptyFields) {
    const auto parts = split("a::b", ':');
    ASSERT_EQ(parts.size(), 3u);
    EXPECT_EQ(parts[0], "a");
    EXPECT_EQ(parts[1], "");
    EXPECT_EQ(parts[2], "b");
    EXPECT_EQ(split("", ',').size(), 1u);
}

TEST(CommandParseTest, FloatListIsAllOrNothingAndExactCount) {
    float out[3] = {1.f, 2.f, 3.f};
    EXPECT_TRUE(parseFloats(split("4:5:6", ':'), 3, out));
    EXPECT_FLOAT_EQ(out[0], 4.f);
    EXPECT_FLOAT_EQ(out[2], 6.f);

    // Wrong count (too few or too many), or one bad element: nothing is written.
    for (const char* bad : {"7:8", "7:8:9:10", "7:8:x", "7:nan:9", "7::9"}) {
        EXPECT_FALSE(parseFloats(split(bad, ':'), 3, out)) << bad;
        EXPECT_FLOAT_EQ(out[0], 4.f) << bad;
        EXPECT_FLOAT_EQ(out[1], 5.f) << bad;
        EXPECT_FLOAT_EQ(out[2], 6.f) << bad;
    }
}

TEST(CommandParseTest, BoolAcceptsOnlyTheDocumentedSpellings) {
    bool b = false;
    EXPECT_TRUE(parseBool("true", b));  EXPECT_TRUE(b);
    EXPECT_TRUE(parseBool("0", b));     EXPECT_FALSE(b);
    EXPECT_TRUE(parseBool("1", b));     EXPECT_TRUE(b);
    EXPECT_TRUE(parseBool("false", b)); EXPECT_FALSE(b);
    b = true;
    for (const char* bad : {"", "yes", "True", "2", "-1"}) {
        EXPECT_FALSE(parseBool(bad, b)) << bad;
        EXPECT_TRUE(b) << bad;
    }
}

TEST(CommandParseTest, HairParamRejectsInvalidValuesWithoutChangingSettings) {
    HairWorld world;
    HairCommandDispatcher d;
    d.setWorld(&world);
    ASSERT_TRUE(world.setPreset(HairPreset::ShortFur));

    EXPECT_EQ(d.route("SetHairParam:timeStep,0.01"), "OK");
    const auto timeStep = d.route("GetHairParam:timeStep");
    ASSERT_TRUE(timeStep);

    for (auto cmd : {"SetHairParam:timeStep,0", "SetHairParam:timeStep,-0.01", "SetHairParam:timeStep,nan",
                     "SetHairParam:timeStep,0.01x", "SetHairParam:stretchCompliance,-1",
                     "SetHairParam:dampingRate,-0.5", "SetHairParam:windDrag,-1",
                     "SetHairParam:collisionRadius,-0.1", "SetHairParam:iterations,1.5",
                     "SetHairParam:substeps,0"}) {
        const auto r = d.route(cmd);
        ASSERT_TRUE(r) << cmd;
        EXPECT_EQ(r->find("Error:"), 0u) << cmd << " -> " << *r;
    }
    EXPECT_EQ(d.route("GetHairParam:timeStep"), timeStep);

    // Gravity and wind components are signed vectors and stay accepted.
    EXPECT_EQ(d.route("SetHairParam:gravityY,-9.8"), "OK");
    EXPECT_EQ(d.route("SetHairParam:windX,-2"), "OK");
}
