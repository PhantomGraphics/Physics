#include "pch.h"
#include "../Physics/RigidBody.h"
#include "../Physics/BroadPhase.h"
#include <algorithm>
#include <random>

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

void configureSphere(RigidBody& body, SphereShape& shape,
                      const Vector3df& pos, float radius, float mass) {
    shape.radius = radius;
    body.setShape(&shape);
    body.setMass(mass);
    body.position = pos;
}

} // namespace

TEST(BroadPhaseTest, OverlappingPair_IsFound) {
    SphereShape sa, sb;
    RigidBody a, b;
    configureSphere(a, sa, { 0.f, 0.f, 0.f }, 0.5f, 1.f);
    configureSphere(b, sb, { 0.8f, 0.f, 0.f }, 0.5f, 1.f);  // overlap = 0.2

    std::vector<RigidBody*> bodies{ &a, &b };
    std::vector<std::pair<int, int>> pairs;
    BroadPhase::detect(bodies, pairs);

    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0], std::make_pair(0, 1));
}

TEST(BroadPhaseTest, NonOverlappingPair_IsExcluded) {
    SphereShape sa, sb;
    RigidBody a, b;
    configureSphere(a, sa, { 0.f, 0.f, 0.f }, 0.5f, 1.f);
    configureSphere(b, sb, { 10.f, 0.f, 0.f }, 0.5f, 1.f);  // far apart

    std::vector<RigidBody*> bodies{ &a, &b };
    std::vector<std::pair<int, int>> pairs;
    BroadPhase::detect(bodies, pairs);

    EXPECT_TRUE(pairs.empty());
}

TEST(BroadPhaseTest, StaticStaticPair_IsExcluded) {
    SphereShape sa, sb;
    RigidBody a, b;
    configureSphere(a, sa, { 0.f, 0.f, 0.f }, 0.5f, 0.f);   // static
    configureSphere(b, sb, { 0.8f, 0.f, 0.f }, 0.5f, 0.f);  // static, overlapping

    std::vector<RigidBody*> bodies{ &a, &b };
    std::vector<std::pair<int, int>> pairs;
    BroadPhase::detect(bodies, pairs);

    EXPECT_TRUE(pairs.empty());
}

TEST(BroadPhaseTest, ShapelessBody_IsExcluded) {
    SphereShape sa, sc;
    RigidBody a, b, c;
    configureSphere(a, sa, { 0.f, 0.f, 0.f }, 0.5f, 1.f);
    b.setMass(1.f);
    b.position = { 0.1f, 0.f, 0.f };  // would overlap `a` if it had a shape
    configureSphere(c, sc, { 0.2f, 0.f, 0.f }, 0.5f, 1.f);  // overlaps `a`, not `b`

    // `b` sits at index 1 in the middle -- confirms indices of `a`(0) and `c`(2)
    // are reported as-is (no remapping) once `b` is skipped for having no shape.
    std::vector<RigidBody*> bodies{ &a, &b, &c };
    std::vector<std::pair<int, int>> pairs;
    BroadPhase::detect(bodies, pairs);

    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0], std::make_pair(0, 2));
}

TEST(BroadPhaseTest, NullptrSlot_IsSkipped) {
    SphereShape sa, sc;
    RigidBody a, c;
    configureSphere(a, sa, { 0.f, 0.f, 0.f }, 0.5f, 1.f);
    configureSphere(c, sc, { 0.2f, 0.f, 0.f }, 0.5f, 1.f);

    std::vector<RigidBody*> bodies{ &a, nullptr, &c };
    std::vector<std::pair<int, int>> pairs;

    EXPECT_NO_FATAL_FAILURE(BroadPhase::detect(bodies, pairs));

    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0], std::make_pair(0, 2));
}

TEST(BroadPhaseTest, LargerRandomScene_MatchesBruteForce) {
    constexpr int kBodyCount = 40;
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> posDist(-5.f, 5.f);
    std::uniform_real_distribution<float> radiusDist(0.2f, 1.0f);
    std::uniform_int_distribution<int> massDist(0, 3);  // ~1/4 of bodies are static

    std::vector<SphereShape> shapes(kBodyCount);
    std::vector<RigidBody> bodyStorage(kBodyCount);
    std::vector<RigidBody*> bodies;
    bodies.reserve(kBodyCount);

    for (int i = 0; i < kBodyCount; ++i) {
        configureSphere(bodyStorage[i], shapes[i],
                        { posDist(rng), posDist(rng), posDist(rng) },
                        radiusDist(rng),
                        massDist(rng) == 0 ? 0.f : 1.f);
        bodies.push_back(&bodyStorage[i]);
    }

    std::vector<std::pair<int, int>> bvhPairs;
    BroadPhase::detect(bodies, bvhPairs);

    std::vector<std::pair<int, int>> bruteForcePairs;
    for (int i = 0; i < kBodyCount; ++i) {
        if (!bodies[i]->shape) continue;
        Box3df aabbA = bodies[i]->getAABB();
        for (int j = i + 1; j < kBodyCount; ++j) {
            if (!bodies[j]->shape) continue;
            if (bodies[i]->isStatic() && bodies[j]->isStatic()) continue;
            Box3df aabbB = bodies[j]->getAABB();
            if (aabbA.intersects(aabbB))
                bruteForcePairs.emplace_back(i, j);
        }
    }

    std::sort(bvhPairs.begin(), bvhPairs.end());
    std::sort(bruteForcePairs.begin(), bruteForcePairs.end());

    EXPECT_EQ(bvhPairs, bruteForcePairs);
}
