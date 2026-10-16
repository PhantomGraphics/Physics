#include "pch.h"
#include "../PhysicsView/RenderDomainPolicy.h"
#include "../PhysicsView/SoftBodyWireData.h"
#include "../Core/ClothBody.h"
#include "../Core/RigidBody.h"
#include "../Core/ICollisionShape.h"

using namespace Phantom;
using namespace Phantom::Physics;

TEST(RenderDomainPolicyTest, FluidIsDrawnByExactlyOneOfSsfrAndPoints)
{
    for (bool ssfr : {false, true}) {
        RenderDomainInputs in;
        in.useSSFR = ssfr;
        const auto p = planRenderDomains(in);
        EXPECT_EQ(p.ssfr, ssfr);
        EXPECT_EQ(p.fluidPoints, !ssfr);
        EXPECT_TRUE(p.hair);
        EXPECT_TRUE(p.background);
        EXPECT_FALSE(p.cloudVolume);
    }
}

TEST(RenderDomainPolicyTest, CloudPageOwnsTheViewport)
{
    for (bool volume : {false, true}) {
        RenderDomainInputs in;
        in.useSSFR = true;                  // ignored on the cloud page
        in.cloudPageActive = true;
        in.cloudVolumeMode = volume;
        in.flameInScene = true;
        in.flameFixedCarriers = true;
        in.rigidMode = BodyRenderMode::Both;
        in.softMode = BodyRenderMode::Both;
        const auto p = planRenderDomains(in);

        EXPECT_FALSE(p.ssfr);
        EXPECT_FALSE(p.flame);
        EXPECT_FALSE(p.flameDebug);
        EXPECT_FALSE(p.hair);
        EXPECT_FALSE(p.rigidWire);
        EXPECT_FALSE(p.rigidShaded);
        EXPECT_FALSE(p.softWire);
        EXPECT_FALSE(p.softShaded);
        EXPECT_FALSE(p.background);
        EXPECT_EQ(p.cloudVolume, volume);
        EXPECT_EQ(p.fluidPoints, !volume);  // points are the cloud's diagnostic view
    }
}

TEST(RenderDomainPolicyTest, FlameAndItsDebugViewFollowTheScene)
{
    RenderDomainInputs in;
    EXPECT_FALSE(planRenderDomains(in).flame);

    in.flameInScene = true;
    auto p = planRenderDomains(in);
    EXPECT_TRUE(p.flame);
    EXPECT_FALSE(p.flameDebug);          // needs fixed carriers

    in.flameFixedCarriers = true;
    p = planRenderDomains(in);
    EXPECT_TRUE(p.flameDebug);

    in.flameInScene = false;             // carriers alone draw nothing
    EXPECT_FALSE(planRenderDomains(in).flameDebug);
}

TEST(RenderDomainPolicyTest, BodyModesSelectWireShadedOrBoth)
{
    struct Case { BodyRenderMode mode; bool wire; bool shaded; };
    for (const Case c : { Case{ BodyRenderMode::Wireframe, true, false },
                          Case{ BodyRenderMode::Shaded, false, true },
                          Case{ BodyRenderMode::Both, true, true } }) {
        RenderDomainInputs in;
        in.rigidMode = c.mode;
        in.softMode = BodyRenderMode::Wireframe;
        const auto p = planRenderDomains(in);
        EXPECT_EQ(p.rigidWire, c.wire);
        EXPECT_EQ(p.rigidShaded, c.shaded);
        EXPECT_TRUE(p.softWire);         // the soft mode is independent of the rigid one
        EXPECT_FALSE(p.softShaded);
    }
}

namespace {

ClothBody makeCloth()
{
    ClothBodyParams p;
    p.rows = 3;
    p.cols = 3;
    p.width = 1.f;
    p.height = 1.f;
    p.pinTopLeft = false;
    p.pinTopRight = false;
    p.pinTopEdge = false;
    p.origin = { 0.f, 1.f, 0.f };
    return ClothBody(p);
}

} // namespace

TEST(SoftBodyWireDataTest, EdgesAreColoredByStretchAndIndexed)
{
    SoftBodySolver solver;
    auto cloth = makeCloth();
    solver.addBody(&cloth);

    const size_t edges = cloth.getMesh().edges.size();
    ASSERT_GT(edges, 0u);

    auto wire = buildSoftBodyWireData(solver);
    ASSERT_EQ(wire.positions.size(), edges * 2 * 3);
    ASSERT_EQ(wire.colors.size(), edges * 2 * 4);
    ASSERT_EQ(wire.indices.size(), edges * 2);
    for (size_t i = 0; i < wire.indices.size(); ++i) EXPECT_EQ(wire.indices[i], i);

    // A freshly built cloth is at rest length: green, opaque.
    for (size_t v = 0; v < wire.indices.size(); ++v) {
        EXPECT_NEAR(wire.colors[v * 4 + 0], 0.f, 1e-4f);
        EXPECT_NEAR(wire.colors[v * 4 + 1], 1.f, 1e-4f);
        EXPECT_NEAR(wire.colors[v * 4 + 2], 0.f, 1e-4f);
        EXPECT_FLOAT_EQ(wire.colors[v * 4 + 3], 1.f);
    }

    // Stretch one particle far away: its edges turn red.
    cloth.getMesh().particles.positions[0] += Math::Vector3df(5.f, 0.f, 0.f);
    wire = buildSoftBodyWireData(solver);
    float maxRed = 0.f;
    for (size_t v = 0; v < wire.indices.size(); ++v) maxRed = std::max(maxRed, wire.colors[v * 4 + 0]);
    EXPECT_GT(maxRed, 0.9f);
}

TEST(SoftBodyWireDataTest, SphereColliderAddsThreeGreatCirclesAndBoxColliderTwelveEdges)
{
    SoftBodySolver solver;
    auto cloth = makeCloth();
    solver.addBody(&cloth);
    const size_t base = buildSoftBodyWireData(solver).indices.size();

    solver.params().sphereEnabled = true;
    EXPECT_EQ(buildSoftBodyWireData(solver).indices.size(), base + 3 * 32 * 2);

    BoxShape box;
    box.halfExtents = { 0.5f, 0.5f, 0.5f };
    RigidBody rb;
    rb.setShape(&box);
    solver.addRigidBodyCollider(&rb);
    ASSERT_EQ(solver.rigidColliderBodies().size(), 1u);
    EXPECT_EQ(buildSoftBodyWireData(solver).indices.size(), base + 3 * 32 * 2 + 12 * 2);
}
