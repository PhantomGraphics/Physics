#include "pch.h"
#include "../Core/Emitter.h"
#include "../Core/OutflowRegion.h"
#include "../Core/ShapeBoundarySet.h"
#include "../PhysicsView/FluidSeed.h"

#include <random>

using namespace Phantom;
using namespace Phantom::Physics;
using Math::Box3df;
using Math::Vector3df;

namespace {

// Minimal particle storage with the surface removeParticlesInRegions() relies on.
struct FakeSoA {
    std::vector<Vector3df> positions;
    std::vector<int> ids;
    size_t size() const { return positions.size(); }
    void swapAndPop(size_t i) {
        positions[i] = positions.back(); positions.pop_back();
        ids[i] = ids.back(); ids.pop_back();
    }
};

struct SpawnLog {
    std::vector<Vector3df> positions;
    std::vector<Vector3df> velocities;
};

Emitter makeEmitter(float rate, float speed, float jitter)
{
    Emitter e;
    e.center = Vector3df(0, 5, 0);
    e.radius = 0.3f;
    e.particleRadius = 0.1f;
    e.direction = Vector3df(0, -2, 0);   // deliberately not normalized
    e.speed = speed;
    e.speedJitter = jitter;
    e.rate = rate;
    return e;
}

} // namespace

TEST(EmitFromEmittersTest, SpawnsAlongNormalizedDirectionAndConsumesTheJitterStream)
{
    std::vector<Emitter> emitters{ makeEmitter(300.0f, 4.0f, 0.5f) };
    std::mt19937 rng(1234);
    std::mt19937 reference(1234);
    SpawnLog log;

    emitFromEmitters(emitters, 0.01f, 1000, rng,
        [&] { return log.positions.size(); },
        [&](const Vector3df& pos, const Emitter&) { log.positions.push_back(pos); log.velocities.emplace_back(); },
        [&](const Vector3df& v) { log.velocities.back() = v; });

    ASSERT_EQ(log.positions.size(), 3u);   // rate * dt = 3
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
    for (const auto& v : log.velocities) {
        const float speed = 4.0f + 0.5f * dist(reference);
        EXPECT_FLOAT_EQ(v.x, 0.0f);
        EXPECT_FLOAT_EQ(v.y, -speed);
        EXPECT_FLOAT_EQ(v.z, 0.0f);
    }
    // One draw per particle and none more: the next draw of both engines agrees.
    EXPECT_EQ(rng(), reference());
}

TEST(EmitFromEmittersTest, CapacityLimitsSpawnAndAnEmitterAtCapacityKeepsItsAccumulator)
{
    std::vector<Emitter> emitters{ makeEmitter(1000.0f, 1.0f, 0.0f), makeEmitter(1000.0f, 1.0f, 0.0f) };
    std::mt19937 rng(7);
    SpawnLog log;
    auto run = [&](int maxParticles) {
        emitFromEmitters(emitters, 0.01f, maxParticles, rng,
            [&] { return log.positions.size(); },
            [&](const Vector3df& pos, const Emitter&) { log.positions.push_back(pos); log.velocities.emplace_back(); },
            [&](const Vector3df& v) { log.velocities.back() = v; });
    };

    run(4);   // the first emitter would emit 10 but only 4 fit; the second is at capacity
    EXPECT_EQ(log.positions.size(), 4u);
    EXPECT_FLOAT_EQ(emitters[1].accumulator, 0.0f) << "an emitter at capacity must not accumulate";
    EXPECT_GE(emitters[0].accumulator, 6.0f) << "the first emitter keeps its unspent fractional/whole carry";
}

TEST(RemoveParticlesInRegionsTest, DeletesOnlyParticlesInsideAnyRegion)
{
    FakeSoA soa;
    soa.positions = { {0, 0, 0}, {5, 0, 0}, {10, 0, 0}, {5.5f, 0, 0}, {-3, 0, 0} };
    soa.ids = { 0, 1, 2, 3, 4 };
    const std::vector<OutflowRegion> regions{
        { Box3df(Vector3df(4, -1, -1), Vector3df(6, 1, 1)) },
        { Box3df(Vector3df(-4, -1, -1), Vector3df(-2, 1, 1)) } };

    removeParticlesInRegions(regions, soa);

    ASSERT_EQ(soa.size(), 2u);
    std::vector<int> survivors = soa.ids;
    std::sort(survivors.begin(), survivors.end());
    EXPECT_EQ(survivors, (std::vector<int>{ 0, 2 }));
}

TEST(RemoveParticlesInRegionsTest, EmptyRegionListIsANoOp)
{
    FakeSoA soa;
    soa.positions = { {0, 0, 0} };
    soa.ids = { 0 };
    removeParticlesInRegions({}, soa);
    EXPECT_EQ(soa.size(), 1u);
}

TEST(ShapeBoundarySetTest, TypedListsLayerAndVisitInFixedOrder)
{
    ShapeBoundarySet set;
    EXPECT_TRUE(set.empty());

    set.setBox(Box3df(Vector3df(-1, -1, -1), Vector3df(1, 1, 1)));
    std::vector<SphereBoundary> spheres;
    spheres.emplace_back(Vector3df(0, 0, 0), 0.5f, 0.1f);
    set.setSpheres(std::move(spheres));
    auto extra = std::make_shared<PlaneBoundary>(Vector3df(0, 1, 0), 0.0f);
    set.add(extra);
    set.add(nullptr);   // ignored

    std::vector<const IShapeBoundary*> visited;
    set.forEach([&](const IShapeBoundary& b) { visited.push_back(&b); });
    ASSERT_EQ(visited.size(), 6u + 1u + 1u);       // 6 box planes, 1 sphere, 1 generic
    EXPECT_NE(dynamic_cast<const PlaneBoundary*>(visited.front()), nullptr);
    EXPECT_NE(dynamic_cast<const SphereBoundary*>(visited[6]), nullptr);   // spheres after planes
    EXPECT_EQ(visited.back(), extra.get());                                // generic shapes last

    // Replacing the planes leaves the other lists alone.
    set.setPlanes({});
    size_t count = 0;
    set.forEach([&](const IShapeBoundary&) { ++count; });
    EXPECT_EQ(count, 2u);

    set.clear();
    EXPECT_TRUE(set.empty());
}

TEST(FluidSeedTest, GridFillsBoundsInZYXOrderAtTheGivenSpacing)
{
    const Box3df bounds(Vector3df(0, 0, 0), Vector3df(1, 1, 0));
    const auto seed = makeFluidSeed({}, {}, bounds, 1.0f);
    ASSERT_EQ(seed.positions.size(), 4u);
    EXPECT_TRUE(seed.velocities.empty());
    EXPECT_EQ(seed.positions[0], Vector3df(0, 0, 0));
    EXPECT_EQ(seed.positions[1], Vector3df(1, 0, 0));   // x fastest
    EXPECT_EQ(seed.positions[2], Vector3df(0, 1, 0));
    EXPECT_EQ(seed.positions[3], Vector3df(1, 1, 0));

    EXPECT_TRUE(makeFluidSeed({}, {}, bounds, 0.0f).positions.empty());
}

TEST(FluidSeedTest, CustomPositionsReplaceTheGridAndVelocitiesMayBeShorter)
{
    const std::vector<Vector3df> positions{ {1, 2, 3}, {4, 5, 6}, {7, 8, 9} };
    const std::vector<Vector3df> velocities{ {0, 1, 0} };
    const auto seed = makeFluidSeed(positions, velocities,
        Box3df(Vector3df(0, 0, 0), Vector3df(10, 10, 10)), 1.0f);
    ASSERT_EQ(seed.positions.size(), 3u);

    std::vector<Vector3df> created, assigned;
    spawnSeed(seed,
        [&](const Vector3df& p) { created.push_back(p); },
        [&](const Vector3df& v) { assigned.push_back(v); });
    EXPECT_EQ(created, positions);
    ASSERT_EQ(assigned.size(), 1u);
    EXPECT_EQ(assigned[0], Vector3df(0, 1, 0));
}
