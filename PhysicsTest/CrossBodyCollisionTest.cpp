#include "pch.h"
#include <algorithm>
#include "../Physics/CrossBodyCollision.h"
#include "../Physics/SoftBodySolver.h"
#include "../Physics/JellyBody.h"

using namespace Phantom::Physics;
using namespace Phantom::Math;

namespace {

SoftMesh makeParticles(std::initializer_list<Vector3df> positions) {
    SoftMesh mesh;
    for (const auto& pos : positions) {
        SoftParticle p;
        p.position    = pos;
        p.predicted   = pos;
        p.velocity    = {};
        p.force       = {};
        p.inverseMass = 1.f;
        mesh.particles.push_back(p);
    }
    return mesh;
}

} // namespace

// ------------------------------------------------- basic push-apart ----------

TEST(CrossBodyCollisionTest, OverlappingParticlesDifferentBodies_PushedApart) {
    SoftMesh meshA = makeParticles({ { 0.f, 0.f, 0.f } });
    SoftMesh meshB = makeParticles({ { 0.01f, 0.f, 0.f } });

    CrossBodyCollision cc;
    CrossBodyCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    cc.resolve({ &meshA, &meshB }, p, 1.f / 60.f);

    float dist = glm::length(meshB.particles.positions[0] - meshA.particles.positions[0]);
    EXPECT_NEAR(dist, p.thickness, 1e-4f);
}

TEST(CrossBodyCollisionTest, FarApartParticles_Unchanged) {
    SoftMesh meshA = makeParticles({ { 0.f, 0.f, 0.f } });
    SoftMesh meshB = makeParticles({ { 1.f, 0.f, 0.f } });

    CrossBodyCollision cc;
    CrossBodyCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    cc.resolve({ &meshA, &meshB }, p, 1.f / 60.f);

    EXPECT_NEAR(meshA.particles.positions[0].x, 0.f, 1e-6f);
    EXPECT_NEAR(meshB.particles.positions[0].x, 1.f, 1e-6f);
}

// ------------------------------------------------- same-body pairs exempt ----

TEST(CrossBodyCollisionTest, SameBodyOverlap_NotAffected) {
    // 同一 mesh 内の重なりは対象外（SelfCollision が別途担当）
    SoftMesh meshA = makeParticles({ { 0.f, 0.f, 0.f }, { 0.01f, 0.f, 0.f } });
    SoftMesh meshB = makeParticles({ { 5.f, 0.f, 0.f } });

    CrossBodyCollision cc;
    CrossBodyCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    cc.resolve({ &meshA, &meshB }, p, 1.f / 60.f);

    float dist = glm::length(meshA.particles.positions[1] - meshA.particles.positions[0]);
    EXPECT_NEAR(dist, 0.01f, 1e-6f);
}

// ------------------------------------------------- pinned particle -----------

TEST(CrossBodyCollisionTest, PinnedParticle_DoesNotMove) {
    SoftMesh meshA = makeParticles({ { 0.f, 0.f, 0.f } });
    meshA.particles.inverseMasses[0] = 0.f;  // pinned
    SoftMesh meshB = makeParticles({ { 0.01f, 0.f, 0.f } });

    CrossBodyCollision cc;
    CrossBodyCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    cc.resolve({ &meshA, &meshB }, p, 1.f / 60.f);

    EXPECT_NEAR(meshA.particles.positions[0].x, 0.f, 1e-6f);
    float dist = glm::length(meshB.particles.positions[0] - meshA.particles.positions[0]);
    EXPECT_NEAR(dist, p.thickness, 1e-4f);
}

// ------------------------------------------------- velocity consistency -----

TEST(CrossBodyCollisionTest, VelocityAdjusted_MatchesPositionCorrection) {
    SoftMesh meshA = makeParticles({ { 0.f, 0.f, 0.f } });
    SoftMesh meshB = makeParticles({ { 0.01f, 0.f, 0.f } });
    const float dt = 1.f / 60.f;

    CrossBodyCollision cc;
    CrossBodyCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    cc.resolve({ &meshA, &meshB }, p, dt);

    Vector3df moveA = meshA.particles.positions[0] - Vector3df{ 0.f, 0.f, 0.f };
    EXPECT_NEAR(glm::length(meshA.particles.velocities[0] - moveA / dt), 0.f, 1e-4f);
}

// NOTE: SoftBodySolver_CrossBodyEnabled_PreventsInterpenetration and
// SoftBodySolver_CrossBodyDisabled_AllowsInterpenetration (300-step
// world.step() loops) moved to
// Physics/PhysicsView/scenarios/cross_body_collision_enabled.json and
// cross_body_collision_disabled.json, which use the new
// SoftBodyPreset::TwoJelliesStacked + SetCrossBodyCollisionEnabled: +
// GetSoftMinInterBodyDistance:0:1.
