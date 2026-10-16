#pragma once

#include "RigidBody.h"
#include "CollisionPair.h"
#include "CGLib/Util/UnCopyable.h"

#include <vector>
#include <utility>

namespace Phantom {
namespace Physics {

class RigidBodySolver : private UnCopyable {
public:
    struct Params {
        Math::Vector3df gravity          = { 0.f, -9.8f, 0.f };
        int             solverIterations = 10;
        float           baumgarteBeta    = 0.2f;
        float           slop             = 0.005f;
    };

    float timeStep = 0.016f;

    /** @brief Registers a rigid body. Non-owning; caller must keep it alive. */
    void addBody(RigidBody* body) { bodies_.push_back(body); }

    /** @brief Removes all registered bodies/contacts/snapshots (does not delete the bodies; caller owns them). */
    void clearBodies();

    /** @deprecated Alias for clearBodies() (kept for source compatibility). */
    void clear() { clearBodies(); }

    void saveSnapshot();

    Params&       params()       { return params_; }
    const Params& params() const { return params_; }

    bool isRunning() const  { return running_; }
    void setRunning(bool v) { running_ = v; }

    /** @brief Advances one step if isRunning(), no-op otherwise (interactive Play/Pause). */
    void step();

    /**
     * @brief Advances exactly one step regardless of isRunning() -- for a
     * scenario/manual-driven "Step" command, so it doesn't depend on the
     * Play/Pause state (and doesn't require flipping isRunning(), which
     * would also make the interactive per-frame loop advance the sim).
     */
    void stepUnconditional();

    void reset();

    const std::vector<RigidBody*>&                    getBodies()   const { return bodies_; }
    const std::vector<ContactManifold>&               getContacts() const { return contacts_; }

private:
    Params                        params_;
    bool                          running_ = false;
    std::vector<RigidBody*>       bodies_;
    std::vector<ContactManifold>  contacts_;

    struct BodySnapshot {
        Math::Vector3df  position;
        Math::Quaternion orientation;
        Math::Vector3df  linearVelocity;
        Math::Vector3df  angularVelocity;
    };
    std::vector<BodySnapshot> snapshots_;

    void applyGravity(float dt);
    void broadPhase(std::vector<std::pair<int, int>>& pairs);
    void narrowPhase(const std::vector<std::pair<int, int>>& pairs);
    void prepareContacts();
    void solveContacts(float dt);
    void integratePositions(float dt);
    void updateInertias();
};

} // namespace Physics
} // namespace Phantom
