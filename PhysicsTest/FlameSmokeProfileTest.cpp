#include <gtest/gtest.h>
#include "../PhysicsView/FlameSmokeProfile.h"
#include "../PhysicsView/FlameSmokeShadow.h"

using Phantom::FlameSmokeProfile;
using Phantom::FlameSmokeShadow;

TEST(FlameSmokeProfile, VolumeAndProjectedOpticalDepthPreserveSmokeAmount)
{
    constexpr int count=20000;
    for (double profile : {0.0,0.25,0.5,1.0}) {
        double mass=0,projected=0;
        for (int i=0;i<count;++i) {
            const double r=(i+0.5)/count;
            mass+=3*r*r*FlameSmokeProfile::density(r,profile)/count;
            const double chord=std::sqrt(1-r*r);
            projected+=2*r*FlameSmokeProfile::column(r*r,-chord,chord,profile)/count;
        }
        EXPECT_NEAR(mass,1.0,1e-8);
        EXPECT_NEAR(projected,2.0/3.0,2e-7);
    }
    EXPECT_DOUBLE_EQ(FlameSmokeProfile::density(1,1),0);
    EXPECT_LT(FlameSmokeProfile::density(0.99,1),0.0001);
}

TEST(FlameSmokeProfile, RadialSamplesHaveTheExpectedMassMoments)
{
    constexpr int count=20000;
    for (double profile : {0.0,0.5,1.0}) {
        double squaredRadius=0;
        for (int i=0;i<count;++i) {
            const double r=FlameSmokeProfile::sampleRadius((i+0.5)/count,profile);
            ASSERT_GE(r,0); ASSERT_LE(r,1);
            squaredRadius+=r*r/count;
        }
        // Uniform volume: E[r^2]=3/5. Poly6: r^2 ~ Beta(3/2,4), E[r^2]=3/11.
        EXPECT_NEAR(squaredRadius,(1-profile)*3.0/5.0+profile*3.0/11.0,2e-6);
    }
    EXPECT_DOUBLE_EQ(FlameSmokeProfile::sampleRadius(0,1),0);
    EXPECT_DOUBLE_EQ(FlameSmokeProfile::sampleRadius(1,1),1);
}

TEST(FlameSmokeProfile, CentralAndGrazingRayDepthAgreeWithProjection)
{
    const FlameSmokeShadow::Puff puff{{0,0,0},2,2};
    EXPECT_NEAR(FlameSmokeShadow::rayDepth(puff,{0,-2,0},{0,1,0},1),6,1e-6);
    EXPECT_NEAR(FlameSmokeShadow::rayDepth(puff,{0,0,0},{0,1,0},1),3,1e-6);
    for (float impact : {0.2f,0.5f,0.9f,0.99f}) {
        const double expected=6*std::pow(1-double(impact)*impact,3.5);
        EXPECT_NEAR(FlameSmokeShadow::rayDepth(puff,{impact,-2,0},{0,1,0},1),expected,1e-6);
    }
}

TEST(FlameSmokeProfile, PartialRayDepthMatchesIndependentDensityIntegration)
{
    const FlameSmokeShadow::Puff puff{{0,0,0},2,3};
    const glm::vec3 direction=glm::normalize(glm::vec3(1,0.15f,0.05f));
    constexpr int count=100000;
    constexpr double step=4.0/count;
    for (float profile : {0.0f,0.5f,1.0f}) {
        for (const glm::vec3 origin : {glm::vec3(-2,0.3f,0.1f),glm::vec3(0,0.2f,0.1f),glm::vec3(2,0,0)}) {
            double expected=0;
            for (int i=0;i<count;++i) {
                const auto p=glm::dvec3(origin)+glm::dvec3(direction)*((i+0.5)*step);
                expected+=(3.0/2.0)*FlameSmokeProfile::density(glm::length(p),profile)*step;
            }
            EXPECT_NEAR(FlameSmokeShadow::rayDepth(puff,origin,direction,profile),expected,7e-5);
        }
    }
}

TEST(FlameSmokeProfile, BvhUsesTheSelectedDensityProfile)
{
    FlameSmokeShadow shadow;
    shadow.build({{{0,0,0},2,4},{{0,3,0},2,2}},1);
    EXPECT_NEAR(shadow.opticalDepth(0,{0,1,0}),6,1e-6);
    shadow.build({{{0,0,0},2,4},{{0,3,0},2,2}},0);
    EXPECT_NEAR(shadow.opticalDepth(0,{0,1,0}),2,1e-6);
}
