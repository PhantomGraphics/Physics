#include <gtest/gtest.h>
#include "../PhysicsView/FlameLightSampler.h"

TEST(FlameLightSampler, ImportanceWeightsPreserveTotalFlux)
{
    Phantom::FlameLightSampler sampler;
    sampler.add({{1,0,0},{1,2,3},0.1f});
    sampler.add({{2,0,0},{3,6,9},0.2f});
    for (double u : {0.0,0.1,0.249,0.25,0.9,1.0}) {
        const auto light=sampler.sample(u);
        EXPECT_NEAR(light.flux.r,4,1e-5f);
        EXPECT_NEAR(light.flux.g,8,1e-5f);
        EXPECT_NEAR(light.flux.b,12,1e-5f);
    }
    EXPECT_FLOAT_EQ(sampler.sample(0.1).position.x,1);
    EXPECT_FLOAT_EQ(sampler.sample(0.9).position.x,2);
}

TEST(FlameLightSampler, ColoredSourcesConvergeToSum)
{
    Phantom::FlameLightSampler sampler;
    sampler.add({{0,0,0},{2,0,0},0.1f});
    sampler.add({{0,1,0},{0,3,1},0.1f});
    glm::dvec3 sum(0);
    constexpr unsigned count=100000;
    for (unsigned i=0;i<count;++i)
        sum+=glm::dvec3(sampler.sample(Phantom::FlameLightSampler::uniform(i)).flux);
    sum/=count;
    EXPECT_NEAR(sum.r,2,0.03);
    EXPECT_NEAR(sum.g,3,0.03);
    EXPECT_NEAR(sum.b,1,0.01);
}

TEST(FlameLightSampler, NonEmittingSourcesLeaveSamplerEmpty)
{
    Phantom::FlameLightSampler sampler;
    sampler.add({{0,0,0},{0,0,0},1});
    sampler.add({{0,0,0},{1,1,1},0});
    EXPECT_TRUE(sampler.empty());
    EXPECT_FLOAT_EQ(sampler.sample(0.5).flux.r,0);
}
