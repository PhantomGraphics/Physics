#include "pch.h"
#include "SoftBodyWorld.h"

#include <algorithm>
#include <set>
#include <utility>

namespace Phantom {

SoftBodyWorld::SoftBodyWorld(Physics::PhysicsSolver& solver) : physicsSolver_(solver) {
    applyPreset();
}

void SoftBodyWorld::setPreset(SoftBodyPreset p) {
    preset_ = p;
    getWorld().setRunning(false);
    getWorld().clearBodies();
    getWorld().clearRigidBodyColliders();
    bodyPtrs_.clear();
    ownedBodies_.clear();
    applyPreset();
}

void SoftBodyWorld::step() {
    getWorld().step();
}

void SoftBodyWorld::stepForced() {
    getWorld().stepUnconditional();
}

void SoftBodyWorld::reset() {
    getWorld().reset();
}

bool SoftBodyWorld::isRunning() const {
    return physicsSolver_.softSolver().isRunning();
}

void SoftBodyWorld::setRunning(bool v) {
    getWorld().setRunning(v);
}

float SoftBodyWorld::getMinParticlePositionY() const {
    bool first = true;
    float minY = 0.f;
    for (const auto* b : bodyPtrs_) {
        if (!b) continue;
        for (const auto& pos : b->getMesh().particles.positions) {
            if (first) { minY = pos.y; first = false; }
            else minY = std::min(minY, pos.y);
        }
    }
    return minY;
}

float SoftBodyWorld::getMaxParticlePositionY() const {
    bool first = true;
    float maxY = 0.f;
    for (const auto* b : bodyPtrs_) {
        if (!b) continue;
        for (const auto& pos : b->getMesh().particles.positions) {
            if (first) { maxY = pos.y; first = false; }
            else maxY = std::max(maxY, pos.y);
        }
    }
    return maxY;
}

float SoftBodyWorld::getParticlePositionY(int bodyIndex, int particleIndex) const {
    if (bodyIndex < 0 || static_cast<size_t>(bodyIndex) >= bodyPtrs_.size()) return 0.f;
    const auto* b = bodyPtrs_[bodyIndex];
    if (!b) return 0.f;
    const auto& particles = b->getMesh().particles;
    if (particleIndex < 0 || static_cast<size_t>(particleIndex) >= particles.size()) return 0.f;
    return particles.positions[particleIndex].y;
}

float SoftBodyWorld::getBodyTotalVolume(int bodyIndex) const {
    if (bodyIndex < 0 || static_cast<size_t>(bodyIndex) >= bodyPtrs_.size()) return 0.f;
    const auto* jelly = dynamic_cast<const Physics::JellyBody*>(bodyPtrs_[bodyIndex]);
    return jelly ? jelly->getTotalVolume() : 0.f;
}

float SoftBodyWorld::getMinInterBodyDistance(int bodyIndexA, int bodyIndexB) const {
    if (bodyIndexA < 0 || static_cast<size_t>(bodyIndexA) >= bodyPtrs_.size()) return 0.f;
    if (bodyIndexB < 0 || static_cast<size_t>(bodyIndexB) >= bodyPtrs_.size()) return 0.f;
    const auto* a = bodyPtrs_[bodyIndexA];
    const auto* b = bodyPtrs_[bodyIndexB];
    if (!a || !b) return 0.f;

    float minDist = 1e9f;
    for (const auto& pa : a->getMesh().particles.positions)
        for (const auto& pb : b->getMesh().particles.positions)
            minDist = std::min(minDist, glm::length(pb - pa));
    return minDist;
}

float SoftBodyWorld::getMinNonEdgeDistance(int bodyIndex) const {
    if (bodyIndex < 0 || static_cast<size_t>(bodyIndex) >= bodyPtrs_.size()) return 1e9f;
    const auto* b = bodyPtrs_[bodyIndex];
    if (!b) return 1e9f;
    const auto& mesh = b->getMesh();
    const auto& positions = mesh.particles.positions;
    const int n = static_cast<int>(positions.size());
    if (n < 2) return 1e9f;

    std::set<std::pair<int, int>> connected;
    for (const auto& e : mesh.edges)
        connected.insert(e.a < e.b ? std::make_pair(e.a, e.b) : std::make_pair(e.b, e.a));

    float minDist = 1e9f;
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            if (connected.count({ i, j })) continue;
            minDist = std::min(minDist, glm::length(positions[j] - positions[i]));
        }
    }
    return minDist;
}

void SoftBodyWorld::applyPreset() {
    auto& wp = getWorld().params();
    wp.sphereEnabled = false;
    wp.sphereCenter  = { 0.f, 0.f, 0.f };
    wp.sphereRadius  = 0.4f;
    // Reset per-preset so a prior SetCrossBodyCollisionEnabled: doesn't leak
    // into the next preset (mirrors sphereEnabled's reset above).
    wp.crossBodyCollisionEnabled   = false;
    wp.crossBodyCollisionThickness = 0.05f;
    wp.crossBodyCollisionCellSize  = 0.1f;

    switch (preset_) {
    case SoftBodyPreset::ClothTwoPin: {
        Physics::ClothBodyParams p;
        p.rows        = 20;
        p.cols        = 20;
        p.width       = 2.f;
        p.height      = 2.f;
        p.pinTopLeft  = true;
        p.pinTopRight = true;
        p.pinTopEdge  = false;
        spawnBody<Physics::ClothBody>(p);
        break;
    }
    case SoftBodyPreset::ClothTopEdge: {
        Physics::ClothBodyParams p;
        p.rows        = 20;
        p.cols        = 20;
        p.width       = 2.f;
        p.height      = 2.f;
        p.pinTopLeft  = false;
        p.pinTopRight = false;
        p.pinTopEdge  = true;
        spawnBody<Physics::ClothBody>(p);
        break;
    }
    case SoftBodyPreset::ClothWithSphere: {
        wp.sphereEnabled = true;
        wp.sphereRadius  = 0.4f;
        // 球は床(y=0)に接する高さに置く
        wp.sphereCenter  = { 0.f, wp.sphereRadius, 0.f };

        Physics::ClothBodyParams p;
        p.rows        = 20;
        p.cols        = 20;
        p.width       = 2.f;
        p.height      = 2.f;
        p.pinTopLeft  = true;
        p.pinTopRight = true;
        p.pinTopEdge  = false;
        // クロスは球・床と初期状態で重ならないよう上方にオフセットして生成する。
        // 重なったまま開始すると衝突補正が瞬間移動になり、その位置ずれが
        // そのまま速度化されて爆発的な振動を起こす（要調査で確認済み）。
        // addBody 後に位置をずらすと XPBDSolver::setMesh が記憶する reset 用の
        // restPositions_ が古い(重なった)座標のままになるため、生成時点で
        // オフセットする必要がある。
        p.origin = { 0.f, 1.5f, 0.f };
        spawnBody<Physics::ClothBody>(p);
        break;
    }
    case SoftBodyPreset::ClothOnBox: {
        // 床(y=0)の上に静的な RigidBody(Box) を置く（RigidBodyColliderTest.
        // SoftBodyWorld_ClothRestsOnRigidBox のパラメータに合わせる）
        rigidBoxShape_.halfExtents = { 5.f, 0.2f, 5.f };
        rigidBox_.position    = { 0.f, 0.f, 0.f };
        rigidBox_.orientation = { 1.f, 0.f, 0.f, 0.f };
        rigidBox_.setShape(&rigidBoxShape_);
        wp.sphereEnabled = false;
        getWorld().addRigidBodyCollider(&rigidBox_);

        Physics::ClothBodyParams p;
        p.rows        = 4;
        p.cols        = 4;
        p.pinTopLeft  = false;
        p.pinTopRight = false;
        p.origin      = { 0.f, 1.f, 0.f };
        spawnBody<Physics::ClothBody>(p);
        break;
    }
    case SoftBodyPreset::JellySelfOverlap: {
        // A normal, stable JellyDrop cube, plus two extra marker particles
        // appended to its mesh afterward, seeded 0.001 apart and connected
        // to nothing (no edges/tetrahedra reference them) -- the exact
        // configuration SelfCollisionTest.
        // XPBDSolver_SelfCollisionEnabled_PreventsOverlap already validates
        // at the unit level. Both markers get identical gravity/floor/damping
        // treatment every step, so unlike trying to force an entire
        // structural mesh into self-overlap (an existing cloth's own corners
        // gathered together reliably explodes the XPBD distance-constraint
        // solver; two whole overlapping JellyBody cubes settle without ever
        // re-approaching each other once resting on the shared floor, so
        // self-collision never has anything left to correct by the time the
        // scenario samples it -- both tried first, see the internal design notes and
        // 40_soft_self_collision.json) any change in the markers' separation
        // can only be self-collision's own doing.
        Physics::JellyBodyParams p;
        p.nx = 3; p.ny = 3; p.nz = 3;
        p.w  = 1.f; p.h = 1.f; p.d = 1.f;
        p.offset = { 0.f, 1.5f, 0.f };
        auto* jelly = spawnBody<Physics::JellyBody>(p);

        auto& mesh = jelly->getMesh();
        Physics::SoftParticle m0, m1;
        m0.position = m0.predicted = { 3.f, 1.5f, 0.f };      // far from the jelly cube: no interaction with it
        m1.position = m1.predicted = { 3.001f, 1.5f, 0.f };   // 0.001 apart, same as the unit test
        m0.inverseMass = m1.inverseMass = 1.f;
        mesh.particles.push_back(m0);
        mesh.particles.push_back(m1);
        break;
    }
    case SoftBodyPreset::RopeHanging: {
        Physics::RopeBodyParams p;
        p.n        = 30;
        p.length   = 3.f;
        p.pinStart = true;
        p.pinEnd   = false;
        spawnBody<Physics::RopeBody>(p);
        break;
    }
    case SoftBodyPreset::RopePendulum: {
        Physics::RopeBodyParams p;
        p.n        = 20;
        p.length   = 2.f;
        p.pinStart = true;
        p.pinEnd   = false;
        auto* rope = spawnBody<Physics::RopeBody>(p);

        // Reposition chain horizontally to create a pendulum
        auto& mesh = rope->getMesh();
        const int   n   = p.n;
        const float seg = p.length / static_cast<float>(n - 1);
        for (int i = 0; i < n; ++i) {
            Math::Vector3df pos = { i * seg, 0.f, 0.f };
            mesh.particles.positions[i]  = pos;
            mesh.particles.predicted[i]  = pos;
        }
        break;
    }
    case SoftBodyPreset::RopeBothEndsPinned: {
        Physics::RopeBodyParams p;
        p.n        = 8;
        p.length   = 2.f;
        p.pinStart = true;
        p.pinEnd   = true;
        spawnBody<Physics::RopeBody>(p);
        break;
    }
    case SoftBodyPreset::JellyDrop: {
        Physics::JellyBodyParams p;
        p.nx = 3; p.ny = 3; p.nz = 3;
        p.w  = 1.f; p.h = 1.f; p.d = 1.f;
        p.offset = { 0.f, 1.5f, 0.f };
        spawnBody<Physics::JellyBody>(p);
        break;
    }
    case SoftBodyPreset::JellyOnBox: {
        // 床(y=0)の上に静的な RigidBody(Box) を置く
        rigidBoxShape_.halfExtents = { 0.6f, 0.15f, 0.6f };
        rigidBox_.position    = { 0.f, 0.15f, 0.f };
        rigidBox_.orientation = { 1.f, 0.f, 0.f, 0.f };
        rigidBox_.setShape(&rigidBoxShape_);
        getWorld().addRigidBodyCollider(&rigidBox_);

        Physics::JellyBodyParams p;
        p.nx = 3; p.ny = 3; p.nz = 3;
        p.w  = 1.f; p.h = 1.f; p.d = 1.f;
        // 箱の上面(y=0.3)と初期状態で重ならないよう十分上方から生成する。
        // ClothWithSphere と同じ理由（重なり開始→衝突補正の瞬間移動→速度爆発）を避けるため。
        p.offset = { 0.f, 1.2f, 0.f };
        spawnBody<Physics::JellyBody>(p);
        break;
    }
    case SoftBodyPreset::TwoJelliesStacked: {
        // CrossBodyCollisionTest's makeCube(offset): two 0.6-side cubes, one
        // at the origin, one 0.65 above it (overlapping by 0.05 initially).
        Physics::JellyBodyParams bottom;
        bottom.nx = bottom.ny = bottom.nz = 1;
        bottom.w = bottom.h = bottom.d = 0.6f;
        bottom.offset = { 0.f, 0.f, 0.f };
        spawnBody<Physics::JellyBody>(bottom);

        Physics::JellyBodyParams top;
        top.nx = top.ny = top.nz = 1;
        top.w = top.h = top.d = 0.6f;
        top.offset = { 0.f, 0.65f, 0.f };
        spawnBody<Physics::JellyBody>(top);
        break;
    }
    case SoftBodyPreset::Mixed: {
        Physics::ClothBodyParams cp;
        cp.rows        = 8;
        cp.cols        = 8;
        cp.width       = 1.f;
        cp.height      = 1.f;
        cp.pinTopLeft  = true;
        cp.pinTopRight = true;
        cp.origin      = { -1.8f, 1.f, 0.f };
        spawnBody<Physics::ClothBody>(cp);

        Physics::RopeBodyParams rp;
        rp.n        = 15;
        rp.length   = 1.5f;
        rp.pinStart = true;
        rp.pinEnd   = false;
        auto* rope = spawnBody<Physics::RopeBody>(rp);
        {
            auto& mesh = rope->getMesh();
            for (size_t i = 0; i < mesh.particles.size(); ++i) {
                mesh.particles.positions[i].x += 1.8f;
                mesh.particles.positions[i].y += 1.5f;
                mesh.particles.predicted[i]    = mesh.particles.positions[i];
            }
        }

        Physics::JellyBodyParams jp;
        jp.nx = 2; jp.ny = 2; jp.nz = 2;
        jp.w  = 0.6f; jp.h = 0.6f; jp.d = 0.6f;
        jp.offset = { 0.f, 1.5f, 0.f };
        spawnBody<Physics::JellyBody>(jp);
        break;
    }
    }
}

SoftBodyWorld::WireData SoftBodyWorld::buildWireData() const {
    auto raw = getWorld().buildWireData();

    WireData out;
    const auto n = static_cast<uint32_t>(raw.positions.size());
    out.positions.reserve(n * 3);
    out.colors.reserve(n * 4);
    out.indices.reserve(n);

    for (const auto& v : raw.positions) {
        out.positions.push_back(v.x);
        out.positions.push_back(v.y);
        out.positions.push_back(v.z);
    }
    for (const auto& c : raw.colors) {
        out.colors.push_back(c.r);
        out.colors.push_back(c.g);
        out.colors.push_back(c.b);
        out.colors.push_back(c.a);
    }
    for (uint32_t i = 0; i < n; ++i)
        out.indices.push_back(i);

    return out;
}

} // namespace Phantom
