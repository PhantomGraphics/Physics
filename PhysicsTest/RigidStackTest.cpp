#include "pch.h"
#include "../Physics/RigidBody.h"
#include "../Physics/RigidBodySolver.h"
#include <cmath>
#include <memory>
#include <vector>

using namespace Phantom::Physics;
using namespace Phantom::Math;

// Same setup as the PhysicsView "Stacking" preset: five unit boxes stacked
// exactly touching on a floor. A stack of identical boxes must stay a stack.
TEST(RigidBodySolverTest, BoxStack_FiveUnitBoxesStayStacked)
{
    RigidBodySolver world;
    world.timeStep = 0.016f;

    PlaneShape floorShape;
    floorShape.normal = {0.f, 1.f, 0.f};
    floorShape.offset = 0.f;
    BoxShape boxShape;
    boxShape.halfExtents = {0.5f, 0.5f, 0.5f};

    std::vector<std::unique_ptr<RigidBody>> pool;
    auto floor = std::make_unique<RigidBody>();
    floor->setShape(&floorShape);
    floor->setMass(0.f);
    floor->restitution = 0.3f;
    floor->friction    = 0.5f;
    world.addBody(floor.get());
    pool.push_back(std::move(floor));

    std::vector<RigidBody*> boxes;
    for (int i = 0; i < 5; ++i) {
        auto b = std::make_unique<RigidBody>();
        b->setShape(&boxShape);
        b->setMass(1.f);
        b->position    = {0.f, 0.5f + i * 1.0f, 0.f};
        b->restitution = 0.3f;
        b->friction    = 0.5f;
        world.addBody(b.get());
        boxes.push_back(b.get());
        pool.push_back(std::move(b));
    }
    world.saveSnapshot();
    world.setRunning(true);

    for (int step = 0; step < 300; ++step) {
        world.step();
        for (int i = 0; i < 5; ++i) {
            ASSERT_TRUE(std::isfinite(boxes[i]->position.y)) << "step " << step;
            // Neither a lateral slide nor a topple.
            ASSERT_LT(std::abs(boxes[i]->position.x), 0.1f) << "box " << i << " step " << step;
            ASSERT_LT(std::abs(boxes[i]->position.z), 0.1f) << "box " << i << " step " << step;
        }
    }
    for (int i = 0; i < 5; ++i) {
        EXPECT_NEAR(boxes[i]->position.y, 0.5f + i, 0.1f) << "box " << i;
        EXPECT_LT(glm::length(boxes[i]->linearVelocity), 0.5f) << "box " << i;
    }
}
