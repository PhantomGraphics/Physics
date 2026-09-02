#include "pch.h"
#include "FluidWorld.h"

#include "../../CGLib/File/File/STLFileReader.h"

namespace {

// TODO 本来不要．Vector3dfはglm::vec3である．
glm::vec3 toGlm(const Phantom::Math::Vector3df& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

template <class Fn>
void forEachFluidSeedInBounds(const Phantom::Math::Box3df& bounds, float diameter, Fn&& fn)
{
    using Phantom::Math::Vector3df;
    const Vector3df bmin = bounds.getMin();
    const Vector3df bmax = bounds.getMax();
    if (diameter <= 0.0f) return;

    for (float z = bmin.z; z <= bmax.z + 1e-4f; z += diameter) {
        for (float y = bmin.y; y <= bmax.y + 1e-4f; y += diameter) {
            for (float x = bmin.x; x <= bmax.x + 1e-4f; x += diameter) {
                fn(Vector3df(x, y, z));
            }
        }
    }
}

} // namespace

namespace Phantom {

using namespace Phantom::Math;
using namespace Phantom::Physics;

FluidWorld::FluidWorld() : rigid_(physicsSolver_) {
}

void FluidWorld::setCustomInitialData(std::vector<Vector3df> positions,
                                        std::vector<Vector3df> velocities)
{
    customPositions_ = std::move(positions);
    customVelocities_ = std::move(velocities);
}

void FluidWorld::reset()
{
    // Multi-region seeding (addFluidSourceBox()/addFluidSourceSphere())
    // overrides customPositions_ every reset() when any regions are
    // registered -- rebuilt fresh each time, same as the rest of this
    // function rebuilds the fluid/solver from params_ fresh each time.
    // A scene that never registers a region leaves customPositions_ exactly
    // as setCustomInitialData() (or nothing) left it.
    if (!sourceRegions_.empty()) {
        customPositions_ = buildSourceRegionPositions();
        customVelocities_.clear();
    }

    clear();
    switch (type_) {
    case SimulationType::DFSPH:    createDFSPH();    break;
    case SimulationType::PBSPH:    createPBSPH();    break;
    case SimulationType::WCSPH:     createWCSPH();     break;
    case SimulationType::GPU_CSPH: createGPUCSPH();  break;
    }
    // clear() destroyed the previous fluidSolver_ (and with it every registered
    // RigidBoundary*, including meshBoundary_); the fresh one built above needs it back.
    reregisterMeshBoundary();
    reregisterBoundarySpheres();
    // Wall damping is solver state like the boundaries above, so the fresh
    // solver needs it too (no-op for solvers that don't implement it).
    if (fluidSolver_) fluidSolver_->setBoundaryDampingRatio(params_.boundaryDampingRatio);
}

void FluidWorld::addEmitter(const Physics::Emitter& e)
{
    // Force particleRadius to match the scene's own particle radius,
    // overriding whatever the caller passed. WCSPH/DFSPH/PBSPH all derive a
    // spawned particle's SPH mass from its radius (see Emitter::
    // particleRadius's doc comment), and this codebase's density/pressure
    // calibration (createDFSPH()/createWCSPH()/createPBSPH() below) assumes
    // every particle in a fluid shares params_.radius -- a mismatched
    // emitter radius previously caused the solver to diverge within about a
    // second of emission (~1:8000 mass ratio at the old 0.05 default vs.
    // this scene's default radius of 1.0). This is the single choke point
    // every emitter-adding caller (CommandDispatcher, ControlPanel) goes
    // through, so none of them can reintroduce that mismatch.
    Physics::Emitter sceneScaled = e;
    sceneScaled.particleRadius = params_.radius;

    switch (type_) {
    case SimulationType::DFSPH:    if (dfsphFluid_) dfsphFluid_->addEmitter(sceneScaled); break;
    case SimulationType::PBSPH:    if (pbsphFluid_) pbsphFluid_->addEmitter(sceneScaled); break;
    case SimulationType::WCSPH:    if (csphFluid_)  csphFluid_->addEmitter(sceneScaled);  break;
    case SimulationType::GPU_CSPH: break;
    }
}

void FluidWorld::clearEmitters()
{
    switch (type_) {
    case SimulationType::DFSPH:    if (dfsphFluid_) dfsphFluid_->clearEmitters(); break;
    case SimulationType::PBSPH:    if (pbsphFluid_) pbsphFluid_->clearEmitters(); break;
    case SimulationType::WCSPH:    if (csphFluid_)  csphFluid_->clearEmitters();  break;
    case SimulationType::GPU_CSPH: break;
    }
}

const std::vector<Physics::Emitter>& FluidWorld::getEmitters() const
{
    static const std::vector<Physics::Emitter> kEmpty;
    switch (type_) {
    case SimulationType::DFSPH:    return dfsphFluid_ ? dfsphFluid_->getEmitters() : kEmpty;
    case SimulationType::PBSPH:    return pbsphFluid_ ? pbsphFluid_->getEmitters() : kEmpty;
    case SimulationType::WCSPH:    return csphFluid_  ? csphFluid_->getEmitters()  : kEmpty;
    case SimulationType::GPU_CSPH: return kEmpty;
    }
    return kEmpty;
}

void FluidWorld::updateEmitters(const float dt)
{
    switch (type_) {
    case SimulationType::DFSPH:    if (dfsphFluid_) dfsphFluid_->updateEmitters(dt); break;
    case SimulationType::PBSPH:    if (pbsphFluid_) pbsphFluid_->updateEmitters(dt); break;
    case SimulationType::WCSPH:    if (csphFluid_)  csphFluid_->updateEmitters(dt);  break;
    case SimulationType::GPU_CSPH: break;
    }
}

void FluidWorld::addOutflowRegion(const Physics::OutflowRegion& r)
{
    switch (type_) {
    case SimulationType::DFSPH:    if (dfsphFluid_) dfsphFluid_->addOutflowRegion(r); break;
    case SimulationType::PBSPH:    if (pbsphFluid_) pbsphFluid_->addOutflowRegion(r); break;
    case SimulationType::WCSPH:    if (csphFluid_)  csphFluid_->addOutflowRegion(r);  break;
    case SimulationType::GPU_CSPH: break;
    }
}

void FluidWorld::clearOutflowRegions()
{
    switch (type_) {
    case SimulationType::DFSPH:    if (dfsphFluid_) dfsphFluid_->clearOutflowRegions(); break;
    case SimulationType::PBSPH:    if (pbsphFluid_) pbsphFluid_->clearOutflowRegions(); break;
    case SimulationType::WCSPH:    if (csphFluid_)  csphFluid_->clearOutflowRegions();  break;
    case SimulationType::GPU_CSPH: break;
    }
}

const std::vector<Physics::OutflowRegion>& FluidWorld::getOutflowRegions() const
{
    static const std::vector<Physics::OutflowRegion> kEmpty;
    switch (type_) {
    case SimulationType::DFSPH:    return dfsphFluid_ ? dfsphFluid_->getOutflowRegions() : kEmpty;
    case SimulationType::PBSPH:    return pbsphFluid_ ? pbsphFluid_->getOutflowRegions() : kEmpty;
    case SimulationType::WCSPH:    return csphFluid_  ? csphFluid_->getOutflowRegions()  : kEmpty;
    case SimulationType::GPU_CSPH: return kEmpty;
    }
    return kEmpty;
}

void FluidWorld::updateOutflow()
{
    switch (type_) {
    case SimulationType::DFSPH:    if (dfsphFluid_) dfsphFluid_->removeOutflowParticles(); break;
    case SimulationType::PBSPH:    if (pbsphFluid_) pbsphFluid_->removeOutflowParticles(); break;
    case SimulationType::WCSPH:    if (csphFluid_)  csphFluid_->removeOutflowParticles();  break;
    case SimulationType::GPU_CSPH: break;
    }
}

void FluidWorld::stepFluidOnly()
{
    auto t0 = std::chrono::high_resolution_clock::now();

    if (type_ == SimulationType::GPU_CSPH) {
        if (gpuSolver_) gpuSolver_->simulate();
    } else if (fluidSolver_) {
        updateEmitters(params_.timeStep);

        // Per-type (dt, maxIter) selection preserves each solver's
        // previously hardcoded step magnitude -- ISPHSolver unifies
        // simulate()'s signature, not what values are physically meaningful
        // for a given algorithm (DFSPH's dt is dead -- see DFSPHSolver's
        // class doc -- so its value here is arbitrary).
        float dt = params_.timeStep;
        int maxIter = 3;
        fluidSolver_->simulate(dt, maxIter);

        updateOutflow();
    }

    updateWhiteWater(params_.timeStep);

    auto t1 = std::chrono::high_resolution_clock::now();
    lastStepTimeMs_ = std::chrono::duration<float, std::milli>(t1 - t0).count();
}

void FluidWorld::step()
{
    const bool doFluid = isRunning();
    const bool doRigid = rigid_.isRunning();
    const bool doSoft  = softCouplingEnabled_ && softWorld_ && softWorld_->isRunning();

    const bool rigidCoupled = couplingEnabled_     && doFluid && doRigid && supportsOneWayCoupling();
    const bool softCoupled  = softCouplingEnabled_  && doFluid && doSoft && supportsTwoWayCoupling();

    if (rigidCoupled || softCoupled) {
        if (rigidCoupled) physicsSolver_.rigidFluidSolver().syncBoundaries();
        if (softCoupled)  physicsSolver_.softFluidSolver().syncBoundaries(getActiveKernel(), getActiveRestDensity());

        stepFluidOnly();

        if (rigidCoupled) physicsSolver_.rigidFluidSolver().step(coupledTimeStep());
        else if (doRigid) rigid_.step();

        if (softCoupled) physicsSolver_.softFluidSolver().step(coupledTimeStep());
        else if (doSoft) softWorld_->step();
    } else {
        if (doFluid) stepFluidOnly();
        if (doRigid) rigid_.step();
        if (doSoft)  softWorld_->step();
    }
}

void FluidWorld::stepOnce()
{
    const bool rigidCoupled = couplingEnabled_ && supportsOneWayCoupling();
    const bool softCoupled  = softCouplingEnabled_ && softWorld_ && supportsTwoWayCoupling();

    if (rigidCoupled) physicsSolver_.rigidFluidSolver().syncBoundaries();
    if (softCoupled)  physicsSolver_.softFluidSolver().syncBoundaries(getActiveKernel(), getActiveRestDensity());

    stepFluidOnly();

    if (rigidCoupled) physicsSolver_.rigidFluidSolver().stepForced(coupledTimeStep());
    else              rigid_.stepForced();

    // Soft-body stepping when NOT soft-coupled remains the caller's
    // responsibility (see FluidCommandDispatcher's "Step"/"Step:N" handlers),
    // exactly as before this coupling was added.
    if (softCoupled) physicsSolver_.softFluidSolver().stepForced(coupledTimeStep());
}

float FluidWorld::coupledTimeStep() const
{
    // A coupled frame exchanges forces between the fluid and the rigid/soft
    // world *within* that frame, so both sides have to advance by the same dt
    // -- otherwise the momentum the fluid hands over is integrated over a
    // different span than the one it was accumulated over. The three worlds
    // keep their own independent Play/Pause and their own timeStep for the
    // uncoupled path (see this class's doc comment), and those defaults do not
    // agree (fluid 0.01 vs RigidBodySolver/SoftBodySolver 0.016), so a coupled
    // frame used to advance the rigid body 1.6x further through the fluid than
    // the fluid itself advanced -- reading exactly like the rigid body
    // "accelerating past free fall" (docs/issue/CODEBASE_ISSUES.md 1.6).
    // The fluid's dt wins: it is the one the SPH solvers were configured with
    // (createDFSPH()/createWCSPH()/... pass it to setTimeStep()/setBoundary()).
    return params_.timeStep;
}

void FluidWorld::setCouplingEnabled(bool v)
{
    couplingEnabled_ = v;
    if (couplingEnabled_) {
        refreshCoupling();
    } else {
        teardownCoupling();
    }
}

void FluidWorld::setCouplingMode(CouplingMode mode)
{
    couplingMode_ = mode;
    if (couplingEnabled_) refreshCoupling();
}

void FluidWorld::teardownCoupling()
{
    physicsSolver_.rigidFluidSolver().clearBindings();
    clearRigidBoundaries();
    clearRigidBoundaryParticles();
    // clearRigidBoundaries() above is a blanket clear on the solver (no
    // selective-remove API exists -- see ISPHSolver::clearRigidBoundaries()'s
    // doc comment), so it also drops meshBoundary_'s registration even though
    // the mesh boundary is independent of rigid-fluid coupling. Restore it.
    reregisterMeshBoundary();
}

void FluidWorld::refreshCoupling()
{
    teardownCoupling();
    if (!couplingEnabled_) return;
    if (!supportsOneWayCoupling()) return;  // e.g. GPU_CSPH: no CPU solver to bind to

    auto& rfw = physicsSolver_.rigidFluidSolver();
    for (RigidBody* body : rfw.rigidWorld().getBodies()) {
        // Static bodies (floors) are excluded: sustained SDF-penalty contact
        // between a resting fluid pool and a static plane diverges badly
        // (mirrors this world's own domain-boundary planes, which have a
        // similar known, pre-existing per-iteration over-application bug
        // under sustained contact -- see FluidPoolStabilityTest). Coupling here is only
        // reliable for transient contact with *dynamic* bodies passing
        // through; keep the fluid's own domain boundary far from the pool
        // instead of relying on a coupled floor for vertical containment.
        if (body->isStatic() || body->shape == nullptr) continue;

        CouplingMode mode = couplingMode_;
        if (mode == CouplingMode::TwoWay &&
            (!supportsTwoWayCoupling() || body->shape->getType() == ShapeType::Plane)) {
            mode = CouplingMode::OneWay;  // graceful fallback: solver or shape doesn't support Track B
        }

        auto& binding = rfw.bind(body, body->shape, mode);

        // One-Way and Two-Way are alternative boundary models, not layers:
        // RigidBoundary is an SDF penalty that pushes fluid out and returns
        // *nothing* to the body, so registering it alongside the Akinci
        // boundary particles both double-counts the boundary and injects
        // momentum that never comes back -- the fluid gets driven away while
        // the body feels only the (now unbalanced) particle reaction.
        // Registering exactly one of the two keeps the exchange symmetric.
        bool twoWayReady = false;
        if (mode == CouplingMode::TwoWay) {
            SPHKernel* kernel = getActiveKernel();
            if (kernel != nullptr) {
                binding.particles.sample(*body->shape, couplingSpacing_);
                binding.particles.computePsi(*kernel, getActiveRestDensity());
                addRigidBoundaryParticles(&binding.particles);
                twoWayReady = true;
            }
        }
        // Falls back to the SDF penalty when Two-Way could not be set up
        // (no kernel yet), so the body never silently stops being a boundary.
        if (!twoWayReady) {
            addRigidBoundary(&binding.boundary);
        }
    }
}

void FluidWorld::setSoftCouplingEnabled(bool v)
{
    softCouplingEnabled_ = v;
    if (softCouplingEnabled_) {
        refreshSoftCoupling();
    } else {
        teardownSoftCoupling();
    }
}

void FluidWorld::teardownSoftCoupling()
{
    if (softWorld_) physicsSolver_.softFluidSolver().clearBindings();
    clearSoftBoundaryParticles();
}

void FluidWorld::refreshSoftCoupling()
{
    teardownSoftCoupling();
    if (!softCouplingEnabled_) return;
    if (!softWorld_) return;
    if (!supportsTwoWayCoupling()) return;  // e.g. GPU_CSPH/CSPH: no Track B to bind to

    auto& sfw = physicsSolver_.softFluidSolver();
    for (auto* body : softWorld_->getBodyPointers()) {
        if (body == nullptr) continue;
        auto& binding = sfw.bind(body);
        addSoftBoundaryParticles(&binding.particles);
    }
}

size_t FluidWorld::getParticleCount() const
{
    switch (type_) {
    case SimulationType::DFSPH:
        return dfsphFluid_ ? dfsphFluid_->getParticles().size() : 0;
    case SimulationType::PBSPH:
        return pbsphFluid_ ? pbsphFluid_->getParticles().size() : 0;
    case SimulationType::WCSPH:
        return csphFluid_ ? csphFluid_->getParticles().size() : 0;
    case SimulationType::GPU_CSPH:
        return gpuSolver_ ? static_cast<size_t>(gpuSolver_->getNumParticles()) : 0;
    }
    return 0;
}

std::vector<glm::vec3> FluidWorld::getParticlePositions() const
{
    std::vector<glm::vec3> positions;
    positions.reserve(getParticleCount());

    switch (type_) {
    case SimulationType::DFSPH:
        if (dfsphFluid_) {
            for (const auto& pos : dfsphFluid_->getParticles().positions) {
                positions.emplace_back(pos.x, pos.y, pos.z);
            }
        }
        break;
    case SimulationType::PBSPH:
        if (pbsphFluid_) {
            for (const auto& pos : pbsphFluid_->getParticles().positions) {
                positions.emplace_back(pos.x, pos.y, pos.z);
            }
        }
        break;
    case SimulationType::WCSPH:
        if (csphFluid_) {
            for (const auto& pos : csphFluid_->getParticles().positions) {
                positions.emplace_back(pos.x, pos.y, pos.z);
            }
        }
        break;
    case SimulationType::GPU_CSPH:
        // GPU_CSPH path is rendered directly from GPU buffer.
        break;
    }

    return positions;
}

std::vector<float> FluidWorld::getParticleDensities() const
{
    switch (type_) {
    case SimulationType::DFSPH:
        return dfsphFluid_ ? dfsphFluid_->getParticles().densities : std::vector<float>{};
    case SimulationType::PBSPH:
        return pbsphFluid_ ? pbsphFluid_->getParticles().densities : std::vector<float>{};
    case SimulationType::WCSPH:
        return csphFluid_ ? csphFluid_->getParticles().densities : std::vector<float>{};
    case SimulationType::GPU_CSPH:
        break;
    }
    return {};
}

std::vector<glm::vec3> FluidWorld::getParticleVelocities() const
{
    std::vector<glm::vec3> velocities;
    velocities.reserve(getParticleCount());

    switch (type_) {
    case SimulationType::DFSPH:
        if (dfsphFluid_) {
            for (const auto& vel : dfsphFluid_->getParticles().velocities) {
                velocities.emplace_back(vel.x, vel.y, vel.z);
            }
        }
        break;
    case SimulationType::PBSPH:
        if (pbsphFluid_) {
            for (const auto& vel : pbsphFluid_->getParticles().velocities) {
                velocities.emplace_back(vel.x, vel.y, vel.z);
            }
        }
        break;
    case SimulationType::WCSPH:
        if (csphFluid_) {
            for (const auto& vel : csphFluid_->getParticles().velocities) {
                velocities.emplace_back(vel.x, vel.y, vel.z);
            }
        }
        break;
    case SimulationType::GPU_CSPH:
        // GPU_CSPH has no CPU-side velocity readback (see header doc).
        break;
    }

    return velocities;
}

std::vector<glm::vec3> FluidWorld::getSprayPositions() const
{
    std::vector<glm::vec3> out;
    const auto& spray = whiteWater_.getSpray();
    out.reserve(spray.size());
    for (const auto& p : spray) {
        out.push_back(toGlm(p.pos));
    }
    return out;
}

std::vector<float> FluidWorld::getSprayLives() const
{
    std::vector<float> out;
    const auto& spray = whiteWater_.getSpray();
    out.reserve(spray.size());
    for (const auto& p : spray) {
        out.push_back(p.life);
    }
    return out;
}

std::vector<glm::vec3> FluidWorld::getFoamPositions() const
{
    std::vector<glm::vec3> out;
    const auto& foam = whiteWater_.getFoam();
    out.reserve(foam.size());
    for (const auto& p : foam) {
        out.push_back(toGlm(p.pos));
    }
    return out;
}

std::vector<float> FluidWorld::getFoamLives() const
{
    std::vector<float> out;
    const auto& foam = whiteWater_.getFoam();
    out.reserve(foam.size());
    for (const auto& p : foam) {
        out.push_back(p.life);
    }
    return out;
}

void FluidWorld::addRigidBoundary(RigidBoundary* b) {
    if (fluidSolver_) fluidSolver_->addRigidBoundary(b);
}

void FluidWorld::clearRigidBoundaries() {
    if (fluidSolver_) fluidSolver_->clearRigidBoundaries();
}

void FluidWorld::addRigidBoundaryParticles(RigidBoundaryParticles* p) {
    if (fluidSolver_) fluidSolver_->addRigidBoundaryParticles(p);
}

void FluidWorld::clearRigidBoundaryParticles() {
    if (fluidSolver_) fluidSolver_->clearRigidBoundaryParticles();
}

void FluidWorld::addSoftBoundaryParticles(SoftBoundaryParticles* p) {
    if (fluidSolver_) fluidSolver_->addSoftBoundaryParticles(p);
}

bool FluidWorld::loadMeshBoundary(const std::filesystem::path& stlPath, float voxelSize)
{
    File::STLFileReader reader;
    const bool ok = File::STLFileReader::isBinary(stlPath)
        ? reader.readBinary(stlPath)
        : reader.readAscii(stlPath);
    if (!ok) return false;

    const File::STLFile stl = reader.getSTL();
    std::vector<Triangle3df> triangles;
    triangles.reserve(stl.faces.size());
    for (const auto& face : stl.faces) triangles.push_back(face.triangle);
    if (triangles.empty()) return false;

    meshBoundaryShape_ = std::make_unique<MeshBoundaryShape>(triangles, voxelSize);
    meshBoundary_.setShape(meshBoundaryShape_.get());
    meshBoundary_.syncKinematic(Vector3df(0.f, 0.f, 0.f), Quaternion(1.f, 0.f, 0.f, 0.f));

    reregisterMeshBoundary();
    return true;
}

void FluidWorld::clearMeshBoundary()
{
    // No selective-remove API on ISPHSolver (see teardownCoupling()'s comment) --
    // clearing the shape makes getBoundaryForce() a permanent no-op instead
    // (RigidBoundary::getBoundaryForce() returns zero when shape_ == nullptr),
    // so the still-registered meshBoundary_ pointer stays harmless.
    meshBoundary_.setShape(nullptr);
    meshBoundaryShape_.reset();
}

void FluidWorld::reregisterMeshBoundary()
{
    if (!meshBoundaryShape_) return;
    if (fluidSolver_) fluidSolver_->addRigidBoundary(&meshBoundary_);
}

void FluidWorld::addBoundarySphere(const SphereBoundary& s)
{
    boundarySpheres_.push_back(s);
    reregisterBoundarySpheres();
}

void FluidWorld::clearBoundarySpheres()
{
    boundarySpheres_.clear();
    reregisterBoundarySpheres();
}

void FluidWorld::reregisterBoundarySpheres()
{
    // setBoundarySpheres() replaces the solver's whole list every call (see
    // SphereBoundary.h/ISPHSolver.h), so this is safe to call even with an
    // empty boundarySpheres_ (clears whatever the fresh solver started with).
    if (fluidSolver_) fluidSolver_->setBoundarySpheres(boundarySpheres_, params_.timeStep);
}

void FluidWorld::addFluidSourceBox(const Box3df& bounds)
{
    SourceRegion r;
    r.isSphere = false;
    r.box = bounds;
    sourceRegions_.push_back(r);
}

void FluidWorld::addFluidSourceSphere(const Vector3df& center, float radius)
{
    SourceRegion r;
    r.isSphere = true;
    r.center = center;
    r.radius = radius;
    sourceRegions_.push_back(r);
}

void FluidWorld::clearFluidSources()
{
    sourceRegions_.clear();
}

std::vector<Vector3df> FluidWorld::buildSourceRegionPositions() const
{
    std::vector<Vector3df> out;
    const float diameter = params_.radius * 2.0f;
    for (const auto& r : sourceRegions_) {
        if (!r.isSphere) {
            forEachFluidSeedInBounds(r.box, diameter, [&](const Vector3df& pos) {
                out.push_back(pos);
            });
            continue;
        }
        const Box3df bounds(
            Vector3df(r.center.x - r.radius, r.center.y - r.radius, r.center.z - r.radius),
            Vector3df(r.center.x + r.radius, r.center.y + r.radius, r.center.z + r.radius));
        const float radiusSq = r.radius * r.radius;
        forEachFluidSeedInBounds(bounds, diameter, [&](const Vector3df& pos) {
            const Vector3df d = pos - r.center;
            if (d.x * d.x + d.y * d.y + d.z * d.z <= radiusSq) out.push_back(pos);
        });
    }
    return out;
}

void FluidWorld::clearSoftBoundaryParticles() {
    if (fluidSolver_) fluidSolver_->clearSoftBoundaryParticles();
}

SPHKernel* FluidWorld::getActiveKernel() const {
    switch (type_) {
    case SimulationType::DFSPH: return dfsphFluid_ ? dfsphFluid_->getKernel() : nullptr;
    case SimulationType::PBSPH: return pbsphFluid_ ? pbsphFluid_->getKernel() : nullptr;
    // WCSPHFluid gained its own kernel in Phase 4 of
    // docs/todo/PLAN_physics_ownership_and_coupling_unification.md (mirrors
    // DFSPHFluid/PBSPHFluid, replacing WCSPHSolver::simulate()'s old
    // throwaway local SPHKernel), so it can now be reported here too.
    case SimulationType::WCSPH: return csphFluid_ ? csphFluid_->getKernel() : nullptr;
    default: return nullptr;
    }
}

float FluidWorld::getActiveRestDensity() const {
    switch (type_) {
    case SimulationType::DFSPH: return dfsphFluid_ ? dfsphFluid_->getDensity()     : params_.density;
    case SimulationType::PBSPH: return pbsphFluid_ ? pbsphFluid_->getRestDensity() : params_.density;
    case SimulationType::WCSPH: return csphFluid_ ? csphFluid_->getDensity()       : params_.density;
    default: return params_.density;
    }
}

void FluidWorld::clear()
{
    fluidSolver_.reset();

    dfsphFluid_.reset();

    pbsphFluid_.reset();

    csphFluid_.reset();

    gpuSolver_.reset();

    whiteWater_.clear();
}

void FluidWorld::createDFSPH()
{
    dfsphFluid_ = std::make_unique<DFSPHFluid>();
    dfsphFluid_->setEffectLength(params_.effectLength);
    dfsphFluid_->density = params_.density;
    dfsphFluid_->viscosityCoe = params_.viscosity;
    // Used only by DFSPHSolver's Two-Way rigid/soft boundary coupling force
    // (addRigidBoundaryParticlePressure/addSoftBoundaryParticlePressure),
    // via the same Tait-EOS-style linearized pressure formula (pressureCoe *
    // (rho - restDensity)) as WCSPHFluid::pressureCoe -- DFSPH's own
    // divergence-free pressure solve doesn't use pressureCoe at all. A raw
    // fixed value means something different at every scene scale for the
    // same reason as WCSPH's (see createWCSPH() below and
    // docs/todo/PLAN_sph_scale_invariance.md section 4/Phase 1), so this
    // reuses WCSPHFluid::estimatePressureCoe() -- a pure function of
    // effectLength/pressureCoeScale, not tied to WCSPH's own instance state
    // -- instead of params_.stiffness.
    dfsphFluid_->pressureCoe = WCSPHFluid::estimatePressureCoe(
        params_.effectLength, params_.pressureCoeScale);

    const float diameter = params_.radius * 2.0f;
    const float mass = diameter * diameter * diameter;

    dfsphFluid_->density = DFSPHSolver::calculateRestDensity(
        params_.effectLength, params_.radius, mass, dfsphFluid_.get());

    dfsphFluid_->setMaxParticles(params_.maxParticles);

    if (!customPositions_.empty()) {
        for (size_t i = 0; i < customPositions_.size(); ++i) {
            dfsphFluid_->createParticle(customPositions_[i], params_.radius, mass);
            if (i < customVelocities_.size())
                dfsphFluid_->getParticles().velocities.back() = customVelocities_[i];
        }
    } else {
        forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
            dfsphFluid_->createParticle(pos, params_.radius, mass);
        });
    }

    auto solver = std::make_unique<DFSPHSolver>();
    // DFSPH treats this as the adaptive integration ceiling; simulate(dt)
    // still advances the caller-requested frame duration.
    solver->setMaxSubstep(params_.timeStep);
    solver->setBoundary(params_.boundary, params_.timeStep);
    solver->setExternalForce(params_.gravity);
    solver->add(dfsphFluid_.get());
    fluidSolver_ = std::move(solver);
}

void FluidWorld::createPBSPH()
{
    pbsphFluid_ = std::make_unique<PBSPHFluid>();
    pbsphFluid_->setEffectLength(params_.effectLength);
    pbsphFluid_->setRestDensity(params_.density);
    pbsphFluid_->setStiffness(params_.stiffness);
    pbsphFluid_->setVicsosity(params_.viscosity);
    pbsphFluid_->setMaxParticles(params_.maxParticles);

    const float diameter = params_.radius * 2.0f;

    if (!customPositions_.empty()) {
        for (size_t i = 0; i < customPositions_.size(); ++i) {
            pbsphFluid_->createParticle(customPositions_[i], params_.radius);
            if (i < customVelocities_.size())
                pbsphFluid_->getParticles().velocities.back() = customVelocities_[i];
        }
    } else {
        forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
            pbsphFluid_->createParticle(pos, params_.radius);
        });
    }

    // Domain container walls: see PBSPHSolver::setBoundary()'s doc comment
    // for how this stays a cheap, O(1)-per-particle penalty (no boundary-
    // particle sampling -- that was tried and works, but its O(boundary
    // sample count) cost is a real hang/OOM risk whenever params_.boundary
    // is set large relative to the fluid, e.g. this codebase's own
    // "-1000..1000, roomy domain box" scenario convention).
    auto solver = std::make_unique<PBSPHSolver>();
    solver->setTimeStep(params_.timeStep);
    solver->setBoundary(params_.boundary, params_.timeStep);
    solver->setExternalForce(params_.gravity);
    solver->add(pbsphFluid_.get());
    fluidSolver_ = std::move(solver);
}

void FluidWorld::createWCSPH()
{
    csphFluid_ = std::make_unique<WCSPHFluid>();
    csphFluid_->setVicosityCoe(params_.viscosity);
    csphFluid_->setTensionCoe(params_.tension);
    // setEffectLength() must precede setPressureCoeFromScale(), which reads
    // this fluid's own effectLength (see WCSPHFluid::setPressureCoeFromScale()'s
    // doc comment).
    csphFluid_->setEffectLength(params_.effectLength);
    csphFluid_->setDensity(params_.density);
    // Scale-invariant in place of a raw params_.stiffness: derives pressureCoe
    // from effectLength * pressureCoeScale so the UI's "Pressure Coe Scale"
    // knob keeps meaning the same thing regardless of scene scale
    // (docs/todo/PLAN_sph_scale_invariance.md section 4/Phase 1).
    csphFluid_->setPressureCoeFromScale(params_.pressureCoeScale);
    csphFluid_->setMaxParticles(params_.maxParticles);

    const float diameter = params_.radius * 2.0f;

    if (!customPositions_.empty()) {
        for (size_t i = 0; i < customPositions_.size(); ++i) {
            csphFluid_->createParticle(customPositions_[i], params_.radius);
            if (i < customVelocities_.size())
                csphFluid_->getParticles().velocities.back() = customVelocities_[i];
        }
    } else {
        forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
            csphFluid_->createParticle(pos, params_.radius);
        });
    }

    auto solver = std::make_unique<WCSPHSolver>();
    solver->setEffectLength(params_.effectLength);
    solver->setTimeStep(params_.timeStep);
    solver->setBoundary(params_.boundary, params_.timeStep);
    solver->setExternalForce(params_.gravity);
    solver->add(csphFluid_.get());
    fluidSolver_ = std::move(solver);
}

void FluidWorld::createGPUCSPH()
{
    if (!vkCtx_ || !vkPool_) {
        // setVulkanContext() must be called before switching to GPU_CSPH
        return;
    }

    const float diameter = params_.radius * 2.0f;
    const float mass     = diameter * diameter * diameter;

    std::vector<Vector3df> positions;
    std::vector<float>     masses;
    {
        const auto bmin = params_.fluidBounds.getMin();
        const auto bmax = params_.fluidBounds.getMax();
        const int nx = std::max(1, static_cast<int>(std::floor((bmax.x - bmin.x) / diameter)) + 1);
        const int ny = std::max(1, static_cast<int>(std::floor((bmax.y - bmin.y) / diameter)) + 1);
        const int nz = std::max(1, static_cast<int>(std::floor((bmax.z - bmin.z) / diameter)) + 1);
        positions.reserve(static_cast<size_t>(nx) * ny * nz);
        masses.reserve(positions.capacity());
    }

    forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
        positions.push_back(pos);
        masses.push_back(mass);
    });

    Phantom::Physics::CSPHSolverVk::Params p;
    p.effectLength = params_.effectLength;
    p.timeStep     = params_.timeStep;
    p.restDensity  = params_.density;
    p.pressureCoe  = params_.stiffness;
    p.viscosityCoe = params_.viscosity;
    p.boundary     = params_.boundary;
    p.gravity      = params_.gravity;

    gpuSolver_ = std::make_unique<Phantom::Physics::CSPHSolverVk>();
    gpuSolver_->setParams(p);
    if (!gpuSolver_->build(*vkCtx_, *vkPool_)) {
        gpuSolver_.reset();
        return;
    }
    gpuSolver_->upload(positions, masses);
}

void FluidWorld::updateWhiteWater(float dt)
{
    std::vector<Vector3df> pos;
    std::vector<Vector3df> vel;
    std::vector<float> density;
    float restDensity = params_.density;

    switch (type_) {
    case SimulationType::DFSPH:
        if (!dfsphFluid_) break;
        restDensity = dfsphFluid_->getDensity();
        pos.reserve(dfsphFluid_->getParticles().size());
        vel.reserve(dfsphFluid_->getParticles().size());
        density.reserve(dfsphFluid_->getParticles().size());
        for (size_t i = 0; i < dfsphFluid_->getParticles().size(); ++i) {
            pos.push_back(dfsphFluid_->getParticles().positions[i]);
            vel.push_back(dfsphFluid_->getParticles().velocities[i]);
            density.push_back(dfsphFluid_->getParticles().densities[i]);
        }
        break;
    case SimulationType::PBSPH:
        if (!pbsphFluid_) break;
        restDensity = pbsphFluid_->getRestDensity();
        pos.reserve(pbsphFluid_->getParticles().size());
        vel.reserve(pbsphFluid_->getParticles().size());
        density.reserve(pbsphFluid_->getParticles().size());
        for (size_t i = 0; i < pbsphFluid_->getParticles().size(); ++i) {
            pos.push_back(pbsphFluid_->getParticles().positions[i]);
            vel.push_back(pbsphFluid_->getParticles().velocities[i]);
            density.push_back(pbsphFluid_->getParticles().densities[i]);
        }
        break;
    case SimulationType::WCSPH:
        if (!csphFluid_) break;
        restDensity = csphFluid_->getDensity();
        pos.reserve(csphFluid_->getParticles().size());
        vel.reserve(csphFluid_->getParticles().size());
        density.reserve(csphFluid_->getParticles().size());
        for (size_t i = 0; i < csphFluid_->getParticles().size(); ++i) {
            pos.push_back(csphFluid_->getParticles().positions[i]);
            vel.push_back(csphFluid_->getParticles().velocities[i]);
            density.push_back(csphFluid_->getParticles().densities[i]);
        }
        break;
    case SimulationType::GPU_CSPH:
        whiteWater_.clear();
        return;
    }

    if (!pos.empty()) {
        whiteWater_.generate(pos, vel, density, restDensity, dt);
        whiteWater_.advance(dt);
    }
}

} // namespace Phantom
