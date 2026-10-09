#include "pch.h"
#include "../Physics/HairGenerator.h"
#include "../PhysicsView/HairWorld.h"
#include "../PhysicsView/HairCommandDispatcher.h"
#include <algorithm>
#include <limits>

using namespace Phantom;
using namespace Phantom::Physics;

namespace {
bool generate(HairStrands& hair, int style, HairVariationParams variation, HairRootPose pose = {}) {
    if (style == 0) {
        HairGeneratorParams p;
        p.root = pose; p.variation = variation;
        return generateHairBundle(hair,p);
    }
    HairSurfaceParams p;
    p.pose = pose; p.variation = variation;
    if (style == 2) { p.surface = HairSurface::Capsule; p.length = 0.06f; p.particlesPerStrand = 4; }
    return generateHairSurface(hair,p);
}
float arcLength(const HairStrands& hair, const HairStrandRange& r) {
    float length = 0.f;
    for (size_t j = 1; j < r.count; ++j)
        length += glm::length(hair.restPositions()[r.offset+j]-hair.restPositions()[r.offset+j-1]);
    return length;
}
}

TEST(HairGeneratorTest, VariationsAreReproducibleBoundedAndRigidPoseEquivariant) {
    for (int style = 0; style < 3; ++style) {
        HairVariationParams v{1.f,0.2f,0.3f,1234};
        HairStrands a,b,c;
        ASSERT_TRUE(generate(a,style,v));
        ASSERT_TRUE(generate(b,style,v));
        HairRootPose pose{{1.f,2.f,-3.f},glm::angleAxis(0.7f,Math::Vector3df(0.f,1.f,0.f))};
        ASSERT_TRUE(generate(c,style,v,pose));
        float minLength = 100.f, maxLength = 0.f;
        const float nominal = style == 2 ? 0.06f : 1.2f;
        for (const auto& r : a.ranges()) {
            const float length = arcLength(a,r);
            EXPECT_GE(length,nominal*0.7f-1.e-6f);
            EXPECT_LE(length,nominal*1.3f+1.e-6f);
            minLength = std::min(minLength,length); maxLength = std::max(maxLength,length);
            EXPECT_EQ(a.particles().inverseMasses[r.offset],0.f);
            EXPECT_LT(glm::length(a.particles().positions[r.offset]-r.root.position),1.e-6f);
            if (style != 0) {
                auto center = Math::Vector3df(0.f);
                if (style == 2) center.z = std::clamp(r.root.position.z,-0.4f,0.4f);
                EXPECT_NEAR(glm::length(r.root.position-center),0.32f,1.e-6f);
                if (style == 1) {
                    const float angle = std::acos(r.root.position.y/0.32f);
                    EXPECT_GE(angle,0.35f); EXPECT_LE(angle,1.3f);
                }
            }
        }
        EXPECT_GT(maxLength-minLength,nominal*0.3f);
        for (size_t i = 0; i < a.particleCount(); ++i) {
            EXPECT_EQ(a.particles().positions[i],b.particles().positions[i]);
            EXPECT_LT(glm::length(c.particles().positions[i]-(pose.position+pose.rotation*a.particles().positions[i])),3.e-6f);
        }
        ++v.seed;
        ASSERT_TRUE(generate(b,style,v));
        EXPECT_GT(glm::length(a.particles().positions.back()-b.particles().positions.back()),1.e-5f);
    }
}

TEST(HairGeneratorTest, VariationControlsIndependentlyChangeRootsShapeAndLength) {
    for (int style = 0; style < 3; ++style) {
        HairStrands base,changed;
        ASSERT_TRUE(generate(base,style,{}));
        // Changing seed alone has no effect with zero variation.
        ASSERT_TRUE(generate(changed,style,{0.f,0.f,0.f,8765}));
        EXPECT_EQ(base.particles().positions,changed.particles().positions);
        for (int control = 0; control < 3; ++control) {
            HairVariationParams v;
            if (control == 0) v.rootJitter = 1.f;
            if (control == 1) v.shapeVariation = 0.3f;
            if (control == 2) v.lengthVariation = 0.4f;
            ASSERT_TRUE(generate(changed,style,v));
            const auto& r = base.ranges().front();
            const auto& next = changed.ranges().front();
            if (control == 0) EXPECT_GT(glm::length(r.root.position-next.root.position),1.e-5f);
            else EXPECT_EQ(r.root.position,next.root.position);
            if (control < 2) EXPECT_NEAR(arcLength(base,r),arcLength(changed,next),1.e-5f);
            else EXPECT_GT(std::abs(arcLength(base,r)-arcLength(changed,next)),1.e-5f);
            if (control == 1) EXPECT_GT(glm::length(base.restPositions()[r.offset+r.count-1]-
                                                  changed.restPositions()[next.offset+next.count-1]),1.e-5f);
        }
    }
}

TEST(HairGeneratorTest, InvalidVariationsPreserveExistingGeometry) {
    for (int style = 0; style < 3; ++style) {
        HairStrands hair;
        ASSERT_TRUE(generate(hair,style,{}));
        const auto revision = hair.revision();
        const auto positions = hair.particles().positions;
        for (auto v : {HairVariationParams{-0.1f,0.f,0.f,0}, HairVariationParams{1.1f,0.f,0.f,0},
                       HairVariationParams{0.f,-0.1f,0.f,0}, HairVariationParams{0.f,1.1f,0.f,0},
                       HairVariationParams{0.f,0.f,-0.1f,0}, HairVariationParams{0.f,0.f,1.f,0},
                       HairVariationParams{0.f,std::numeric_limits<float>::quiet_NaN(),0.f,0}}) {
            EXPECT_FALSE(generate(hair,style,v));
            EXPECT_EQ(hair.revision(),revision);
            EXPECT_EQ(hair.particles().positions,positions);
        }
    }
}

TEST(HairWorldTest, GenerationCommandsApplyOnNextPresetAndRejectInvalidValues) {
    HairWorld world;
    HairCommandDispatcher d;
    d.setWorld(&world);
    ASSERT_EQ(d.route("HairPreset:LongHair"),"OK");
    const auto initial = world.strands().particles().positions;
    for (auto cmd : {"SetHairGenerationParam:rootJitter,0.8", "SetHairGenerationParam:shapeVariation,0.2",
                     "SetHairGenerationParam:lengthVariation,0.3", "SetHairGenerationParam:seed,4294967295"})
        ASSERT_EQ(d.route(cmd),"OK");
    EXPECT_EQ(world.strands().particles().positions,initial);
    EXPECT_EQ(d.route("GetHairGenerationParam:seed"),"4294967295");
    EXPECT_EQ(world.generationParams().seed,UINT32_MAX);
    for (auto cmd : {"SetHairGenerationParam:seed,-1", "SetHairGenerationParam:seed,4294967296",
                     "SetHairGenerationParam:seed,1.5", "SetHairGenerationParam:lengthVariation,1",
                     "SetHairGenerationParam:rootJitter,nan", "SetHairGenerationParam:shapeVariation,inf"}) {
        auto response = d.route(cmd);
        ASSERT_TRUE(response); EXPECT_EQ(response->find("Error:"),0u);
    }
    ASSERT_EQ(d.route("HairPreset:LongHair"),"OK");
    const auto varied = world.strands().particles().positions;
    EXPECT_NE(varied,initial);
    world.reset();
    for (size_t i = 0; i < varied.size(); ++i)
        EXPECT_LT(glm::length(world.strands().particles().positions[i]-varied[i]),1.e-6f);
    ASSERT_EQ(d.route("HairPreset:LongHair"),"OK");
    EXPECT_EQ(world.strands().particles().positions,varied);
    ASSERT_TRUE(world.stepOnce()); EXPECT_EQ(world.stats().nonFiniteCount,0u);
    EXPECT_LT(world.stats().maxRelativeLengthError,0.01f);
}
