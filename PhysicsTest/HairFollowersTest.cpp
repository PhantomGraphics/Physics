#include "pch.h"
#include "../Physics/HairFollowers.h"
#include "../Physics/HairGenerator.h"
#include "../Physics/HairSolver.h"
#include <limits>

using namespace Phantom::Physics;
using Phantom::Math::Vector3df;

namespace {
HairStrands threeGuides() {
    std::vector<HairStrandInput> inputs;
    for (int s = 0; s < 3; ++s) {
        HairStrandInput in;
        in.root.position = {2.f*s,0.f,0.f};
        const int count = 2+s*2;
        for (int j = 0; j < count; ++j)
            in.restPositions.push_back({0.f,-(s+1.f)*j/(count-1),0.f});
        inputs.push_back(in);
    }
    HairStrands guides;
    guides.initialize(inputs);
    return guides;
}
}

TEST(HairFollowersTest, NearestGuidesBlendUnequalParticleCountsAtNormalizedCoordinates) {
    const auto guides = threeGuides();
    const HairRootPose root{{1.f,0.f,0.f},{1.f,0.f,0.f,0.f}};
    HairFollowers followers;
    ASSERT_TRUE(followers.initialize(guides,{root},5));
    EXPECT_EQ(followers.strandCount(),1u);
    const auto& b = followers.bindings()[0];
    EXPECT_EQ(b.guides[0],0u);
    EXPECT_EQ(b.guides[1],1u);
    EXPECT_EQ(b.guides[2],2u);
    EXPECT_NEAR(b.weights[0],3.f/7.f,1.e-6f);
    EXPECT_NEAR(b.weights[1],3.f/7.f,1.e-6f);
    EXPECT_NEAR(b.weights[2],1.f/7.f,1.e-6f);
    for (size_t j = 0; j < 5; ++j) {
        const auto point = followers.positions()[j];
        EXPECT_NEAR(point.x,1.f,1.e-6f);
        EXPECT_NEAR(point.y,-12.f/7.f*j/4.f,1.e-6f);
        EXPECT_FLOAT_EQ(point.z,0.f);
    }
}

TEST(HairFollowersTest, CoincidentRootReproducesGuideAndSingleGuideUsesFollowerFrame) {
    HairGeneratorParams p;
    p.strands = 1; p.particlesPerStrand = 5; p.curvature = 0.4f;
    HairStrands guides;
    ASSERT_TRUE(generateHairBundle(guides,p));
    HairFollowers followers;
    const auto root = guides.ranges()[0].root;
    ASSERT_TRUE(followers.initialize(guides,{root},5));
    for (size_t i = 0; i < 5; ++i)
        EXPECT_LT(glm::length(followers.positions()[i]-guides.particles().positions[i]),1.e-6f);
    auto other = root;
    other.position.x += 0.1f;
    other.rotation = glm::angleAxis(0.7f,Vector3df(0.f,0.f,1.f));
    ASSERT_TRUE(followers.initialize(guides,{other},5));
    for (size_t i = 0; i < 5; ++i)
        EXPECT_LT(glm::length(followers.positions()[i]-(other.position+other.rotation*guides.restPositions()[i])),1.e-6f);
}

TEST(HairFollowersTest, RootFramesKeepCapsuleFollowersOutwardAndDeterministic) {
    HairSurfaceParams p;
    p.surface = HairSurface::Capsule; p.strands = 12;
    p.length = 0.06f; p.particlesPerStrand = 4;
    HairStrands guides,seeds;
    ASSERT_TRUE(generateHairSurface(guides,p));
    p.strands = 60;
    ASSERT_TRUE(generateHairSurface(seeds,p));
    std::vector<HairRootPose> roots;
    for (const auto& r : seeds.ranges()) roots.push_back(r.root);
    HairFollowers a,b;
    ASSERT_TRUE(a.initialize(guides,roots,4));
    ASSERT_TRUE(b.initialize(guides,roots,4));
    for (size_t f = 0; f < roots.size(); ++f) {
        for (size_t j = 0; j < 4; ++j) {
            const auto expected = roots[f].position+roots[f].rotation*Vector3df(0.f,p.length*j/3.f,0.f);
            EXPECT_LT(glm::length(a.positions()[f*4+j]-expected),1.e-6f);
            EXPECT_FLOAT_EQ(glm::length(a.positions()[f*4+j]-b.positions()[f*4+j]),0.f);
        }
    }
}

TEST(HairFollowersTest, RigidTeleportTransformsAllVerticesWithoutRebinding) {
    auto guides = threeGuides();
    HairFollowers followers;
    ASSERT_TRUE(followers.initialize(guides,{{{1.f,0.f,0.f},{1.f,0.f,0.f,0.f}}},7));
    const auto initial = followers.positions();
    const auto binding = followers.bindings()[0];
    HairSolver solver;
    ASSERT_TRUE(solver.setStrands(&guides));
    const auto originalRoots = guides.ranges();
    const auto rotation = glm::angleAxis(0.8f,glm::normalize(Vector3df(1.f,2.f,3.f)));
    const Vector3df translation(2.f,-1.f,3.f);
    for (size_t s = 0; s < guides.strandCount(); ++s) {
        auto root = originalRoots[s].root;
        root.position = translation+rotation*root.position;
        root.rotation = rotation*root.rotation;
        ASSERT_TRUE(solver.setRootPose(s,root,true));
    }
    ASSERT_TRUE(followers.update(guides));
    for (size_t i = 0; i < initial.size(); ++i)
        EXPECT_LT(glm::length(followers.positions()[i]-(translation+rotation*initial[i])),2.e-6f);
    EXPECT_EQ(followers.bindings()[0].guides,binding.guides);
    EXPECT_EQ(followers.bindings()[0].weights,binding.weights);
}

TEST(HairFollowersTest, WindAndResetFollowGuidesWithoutChangingSimulationTopology) {
    HairGeneratorParams p;
    p.strands = 2; p.particlesPerStrand = 8; p.length = 0.3f;
    HairStrands guides;
    ASSERT_TRUE(generateHairBundle(guides,p));
    auto root = guides.ranges()[0].root;
    root.position = (root.position+guides.ranges()[1].root.position)*0.5f;
    HairFollowers followers;
    ASSERT_TRUE(followers.initialize(guides,{root},8));
    const auto initial = followers.positions();
    const auto revision = guides.revision();
    HairSolver solver;
    ASSERT_TRUE(solver.setStrands(&guides));
    auto settings = solver.params();
    settings.windVelocity = {5.f,0.f,0.f}; settings.windDrag = 2.f;
    ASSERT_TRUE(solver.setParams(settings));
    for (int i = 0; i < 30; ++i) {
        ASSERT_TRUE(solver.step());
        ASSERT_TRUE(followers.update(guides));
    }
    EXPECT_GT(glm::length(followers.positions().back()-initial.back()),0.01f);
    EXPECT_EQ(guides.revision(),revision);
    EXPECT_EQ(solver.stats().particleCount,16u);
    EXPECT_EQ(solver.stats().steps,30u);
    for (size_t j = 0; j < 8; ++j) {
        const auto expected = 0.5f*(guides.particles().positions[j]+guides.particles().positions[8+j]);
        EXPECT_LT(glm::length(followers.positions()[j]-expected),1.e-6f);
    }
    solver.reset();
    ASSERT_TRUE(followers.update(guides));
    for (size_t i = 0; i < initial.size(); ++i)
        EXPECT_LT(glm::length(followers.positions()[i]-initial[i]),1.e-6f);
}

TEST(HairFollowersTest, InvalidInputAndStaleOrDifferentSourcePreserveOutput) {
    auto guides = threeGuides();
    HairFollowers followers;
    const HairRootPose root{{1.f,0.f,0.f},{1.f,0.f,0.f,0.f}};
    ASSERT_TRUE(followers.initialize(guides,{root},4));
    const auto initial = followers.positions();
    auto invalid = root;
    invalid.position.x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(followers.initialize(guides,{invalid},4));
    invalid = root; invalid.rotation = {0.f,0.f,0.f,0.f};
    EXPECT_FALSE(followers.initialize(guides,{invalid},4));
    EXPECT_FALSE(followers.initialize(guides,{root},1));
    EXPECT_FALSE(followers.initialize(guides,{root},4097));
    EXPECT_FALSE(followers.initialize(guides,std::vector<HairRootPose>(1000,root),4096));
    HairStrands empty;
    EXPECT_FALSE(followers.initialize(empty,{root},4));
    const auto other = threeGuides();
    EXPECT_FALSE(followers.update(other));
    HairGeneratorParams p;
    ASSERT_TRUE(generateHairBundle(guides,p));
    EXPECT_FALSE(followers.update(guides));
    ASSERT_EQ(followers.positions().size(),initial.size());
    for (size_t i = 0; i < initial.size(); ++i)
        EXPECT_FLOAT_EQ(glm::length(followers.positions()[i]-initial[i]),0.f);
    ASSERT_TRUE(followers.initialize(empty,{},4));
    EXPECT_EQ(followers.strandCount(),0u);
    EXPECT_TRUE(followers.positions().empty());
}
