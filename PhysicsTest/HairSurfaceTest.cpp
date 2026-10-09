#include "pch.h"
#include "../Physics/HairGenerator.h"
#include <algorithm>
#include <limits>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

TEST(HairGeneratorTest, SurfaceRootsLengthsAndNormalsMatchAnalyticGeometry) {
    for (auto surface : {HairSurface::Scalp, HairSurface::Capsule}) {
        HairSurfaceParams p;
        p.surface = surface;
        p.strands = 96;
        if (surface == HairSurface::Capsule) { p.length = 0.06f; p.particlesPerStrand = 4; }
        HairStrands hair;
        ASSERT_TRUE(generateHairSurface(hair,p));
        int negativeCap = 0, positiveCap = 0, cylinder = 0;
        for (const auto& r : hair.ranges()) {
            const auto root = r.root.position-p.pose.position;
            auto center = Vector3df(0.f);
            if (surface == HairSurface::Capsule) {
                center.z = std::clamp(root.z,-p.capsuleHalfLength,p.capsuleHalfLength);
                if (root.z < -p.capsuleHalfLength) ++negativeCap;
                else if (root.z > p.capsuleHalfLength) ++positiveCap;
                else ++cylinder;
            } else {
                const float angle = std::acos(root.y/p.radius);
                EXPECT_GE(angle,p.scalpMinAngle);
                EXPECT_LE(angle,p.scalpMaxAngle);
            }
            EXPECT_NEAR(glm::length(root-center),p.radius,1.e-6f);
            const auto normal = glm::normalize(root-center);
            EXPECT_LT(glm::length(r.root.rotation*Vector3df(0.f,1.f,0.f)-normal),1.e-5f);
            EXPECT_FLOAT_EQ(hair.particles().inverseMasses[r.offset],0.f);
            float length = 0.f;
            for (size_t j = 1; j < r.count; ++j) {
                const auto point = hair.particles().positions[r.offset+j]-p.pose.position;
                auto nearest = Vector3df(0.f);
                if (surface == HairSurface::Capsule)
                    nearest.z = std::clamp(point.z,-p.capsuleHalfLength,p.capsuleHalfLength);
                EXPECT_GE(glm::length(point-nearest),p.radius-1.e-5f);
                length += glm::length(hair.particles().positions[r.offset+j]-hair.particles().positions[r.offset+j-1]);
                if (surface == HairSurface::Capsule)
                    EXPECT_NEAR(glm::dot(point-root,normal),p.length*j/(r.count-1),1.e-6f);
            }
            EXPECT_NEAR(length,p.length,1.e-5f);
        }
        if (surface == HairSurface::Capsule) {
            // Area fractions are r/(2*(r+h)) per cap and h/(r+h) for the cylinder.
            EXPECT_NEAR(negativeCap,p.strands*p.radius/(2.f*(p.radius+p.capsuleHalfLength)),1.f);
            EXPECT_NEAR(positiveCap,negativeCap,1);
            EXPECT_NEAR(cylinder,p.strands*p.capsuleHalfLength/(p.radius+p.capsuleHalfLength),1.f);
        }
    }
}

TEST(HairGeneratorTest, SurfaceGenerationIsDeterministicAndRigidPoseEquivariant) {
    for (auto surface : {HairSurface::Scalp,HairSurface::Capsule}) {
        HairSurfaceParams p;
        p.surface = surface;
        p.pose = {};
        HairStrands a,b,transformed;
        ASSERT_TRUE(generateHairSurface(a,p));
        ASSERT_TRUE(generateHairSurface(b,p));
        p.pose.position = {1.f,2.f,-3.f};
        p.pose.rotation = 2.f*glm::angleAxis(0.7f,glm::normalize(Vector3df(1.f,2.f,3.f)));
        ASSERT_TRUE(generateHairSurface(transformed,p));
        for (size_t i = 0; i < a.particleCount(); ++i) {
            EXPECT_FLOAT_EQ(glm::length(a.particles().positions[i]-b.particles().positions[i]),0.f);
            const auto expected = p.pose.position+glm::normalize(p.pose.rotation)*a.particles().positions[i];
            EXPECT_LT(glm::length(expected-transformed.particles().positions[i]),2.e-6f);
        }
    }
}

TEST(HairGeneratorTest, InvalidSurfaceParametersPreserveOutput) {
    HairStrands hair;
    ASSERT_TRUE(generateHairSurface(hair,{}));
    const auto revision = hair.revision();
    const auto initial = hair.particles().positions;
    for (int caseIndex = 0; caseIndex < 12; ++caseIndex) {
        HairSurfaceParams p;
        switch (caseIndex) {
        case 0: p.strands = 0; break;
        case 1: p.particlesPerStrand = 1; break;
        case 2: p.strands = 10000; p.particlesPerStrand = 4096; break;
        case 3: p.radius = 0.f; break;
        case 4: p.length = std::numeric_limits<float>::quiet_NaN(); break;
        case 5: p.capsuleHalfLength = -1.f; break;
        case 6: p.scalpMinAngle = p.scalpMaxAngle; break;
        case 7: p.scalpMaxAngle = 2.f; break;
        case 8: p.pose.rotation = {0.f,0.f,0.f,0.f}; break;
        case 9: p.pose.position.x = std::numeric_limits<float>::infinity(); break;
        case 10: p.surface = static_cast<HairSurface>(99); break;
        case 11: p.radius = std::numeric_limits<float>::infinity(); break;
        }
        EXPECT_FALSE(generateHairSurface(hair,p)) << caseIndex;
        EXPECT_EQ(hair.revision(),revision);
        EXPECT_EQ(hair.particleCount(),initial.size());
        EXPECT_FLOAT_EQ(glm::length(hair.particles().positions.back()-initial.back()),0.f);
    }
}

TEST(HairGeneratorTest, SingleGuideAndZeroLengthCapsuleAxisRemainValid) {
    HairSurfaceParams p;
    p.strands = 1;
    p.particlesPerStrand = 2;
    p.length = 0.06f;
    p.surface = HairSurface::Capsule;
    p.capsuleHalfLength = 0.f;
    HairStrands hair;
    ASSERT_TRUE(generateHairSurface(hair,p));
    EXPECT_EQ(hair.particleCount(),2u);
    EXPECT_NEAR(glm::length(hair.ranges()[0].root.position-p.pose.position),p.radius,1.e-6f);
    EXPECT_NEAR(glm::length(hair.particles().positions[1]-hair.particles().positions[0]),p.length,1.e-6f);
    p.surface = HairSurface::Scalp;
    ASSERT_TRUE(generateHairSurface(hair,p));
    EXPECT_NEAR(glm::length(hair.particles().positions[1]-hair.particles().positions[0]),p.length,1.e-6f);
}
