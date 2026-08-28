#include "pch.h"
#include <algorithm>
#include "../Physics/SelfCollision.h"
#include "../Physics/XPBDSolver.h"

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

TEST(SelfCollisionTest, OverlappingUnconnectedParticles_PushedApart) {
    SoftMesh mesh = makeParticles({ { 0.f, 0.f, 0.f }, { 0.01f, 0.f, 0.f } });

    SelfCollision sc;
    SelfCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    sc.update(mesh, p);

    float dist = glm::length(mesh.particles.predicted[1] - mesh.particles.predicted[0]);
    EXPECT_NEAR(dist, p.thickness, 1e-4f);
}

TEST(SelfCollisionTest, FarApartParticles_Unchanged) {
    SoftMesh mesh = makeParticles({ { 0.f, 0.f, 0.f }, { 1.f, 0.f, 0.f } });

    SelfCollision sc;
    SelfCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    sc.update(mesh, p);

    EXPECT_NEAR(mesh.particles.predicted[0].x, 0.f, 1e-6f);
    EXPECT_NEAR(mesh.particles.predicted[1].x, 1.f, 1e-6f);
}

// ------------------------------------------------- structural edges exempt ---

TEST(SelfCollisionTest, ConnectedParticles_NotPushedApart) {
    SoftMesh mesh = makeParticles({ { 0.f, 0.f, 0.f }, { 0.01f, 0.f, 0.f } });
    mesh.edges.push_back({ 0, 1, 0.01f });
    mesh.numStructuralEdges = 1;

    SelfCollision sc;
    SelfCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    sc.update(mesh, p);

    float dist = glm::length(mesh.particles.predicted[1] - mesh.particles.predicted[0]);
    EXPECT_NEAR(dist, 0.01f, 1e-6f);
}

// ------------------------------------------------- pinned particle -----------

TEST(SelfCollisionTest, PinnedParticle_DoesNotMove) {
    SoftMesh mesh = makeParticles({ { 0.f, 0.f, 0.f }, { 0.01f, 0.f, 0.f } });
    mesh.particles.inverseMasses[0] = 0.f;  // pinned

    SelfCollision sc;
    SelfCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    sc.update(mesh, p);

    EXPECT_NEAR(mesh.particles.predicted[0].x, 0.f, 1e-6f);
    float dist = glm::length(mesh.particles.predicted[1] - mesh.particles.predicted[0]);
    EXPECT_NEAR(dist, p.thickness, 1e-4f);
}

// ------------------------------------------------- multiple particles --------

TEST(SelfCollisionTest, ThreeCollidingParticles_SpreadOutAfterIterations) {
    SoftMesh mesh = makeParticles({ { 0.f, 0.f, 0.f }, { 0.01f, 0.f, 0.f }, { 0.02f, 0.f, 0.f } });

    SelfCollision sc;
    SelfCollision::Params p;
    p.thickness = 0.05f;
    p.cellSize  = 0.1f;
    for (int i = 0; i < 10; ++i) sc.update(mesh, p);

    float d01 = glm::length(mesh.particles.predicted[1] - mesh.particles.predicted[0]);
    float d12 = glm::length(mesh.particles.predicted[2] - mesh.particles.predicted[1]);
    float d02 = glm::length(mesh.particles.predicted[2] - mesh.particles.predicted[0]);
    EXPECT_GT(std::min({ d01, d12, d02 }), 0.01f);  // 初期間隔より広がっている
}

// ------------------------------------------------- XPBDSolver integration ----

TEST(SelfCollisionTest, XPBDSolver_SelfCollisionEnabled_PreventsOverlap) {
    // ほぼ重なった2粒子（微小オフセット）が自由落下する（内部制約なし）。
    // 完全一致(dist=0)は分離方向が定義できないため意図的に除外している。
    SoftMesh mesh = makeParticles({ { 0.f, 1.f, 0.f }, { 0.001f, 1.f, 0.f } });

    XPBDSolver solver;
    solver.setMesh(&mesh);
    solver.params().selfCollisionEnabled   = true;
    solver.params().selfCollisionThickness = 0.1f;
    solver.params().selfCollisionCellSize  = 0.2f;

    for (int i = 0; i < 30; ++i) solver.step();

    float dist = glm::length(mesh.particles.positions[1] - mesh.particles.positions[0]);
    EXPECT_GT(dist, 0.05f);
}

TEST(SelfCollisionTest, XPBDSolver_SelfCollisionDisabled_ParticlesStayOverlapped) {
    SoftMesh mesh = makeParticles({ { 0.f, 1.f, 0.f }, { 0.f, 1.f, 0.f } });

    XPBDSolver solver;
    solver.setMesh(&mesh);
    // selfCollisionEnabled はデフォルト false

    for (int i = 0; i < 30; ++i) solver.step();

    float dist = glm::length(mesh.particles.positions[1] - mesh.particles.positions[0]);
    EXPECT_NEAR(dist, 0.f, 1e-5f);
}
