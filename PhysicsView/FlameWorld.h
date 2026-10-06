#pragma once

#include "../Physics/FlameFluid.h"
#include "../Physics/FlameSolver.h"
#include "../Physics/FlameSolidCoupler.h"

#include <glm/glm.hpp>

#include <memory>
#include <functional>
#include <cmath>

namespace Phantom {
class RigidBodyWorld;

/**
 * @brief Flame (reacting hot-gas SPH) domain for FluidApp, folded in from the
 * former standalone FlameView/FlameApp. It is drawn in the shared scene next to
 * the fluid / rigid / soft domains and the glTF background (it no longer owns
 * the viewport while its control page is open).
 *
 * Owns gas, finite-fuel static solids and their non-owning coupler. With no
 * solids, the historical burner and its display transform are unchanged.
 * Coupled steps use <= 0.004 s substeps, a common gas/solid clock, and an
 * independent FlamePhysicalTransform for both solid and gas placement.
 * Optional stable rigid-body handles follow position/orientation/velocity.
 * While bound, Flame owns the rigid world's clock; forced rigid Steps delegate
 * here and interactive rigid updates are suppressed to avoid double stepping.
 * Deforming geometry and rigid-fluid-flame three-way coupling remain separate.
 */
class FlameWorld {
public:
    FlameWorld();
    ~FlameWorld();
    void setRigidWorld(RigidBodyWorld* world);
    RigidBodyWorld* rigidWorld() const { return rigidWorld_; }
    bool bindRigidBody(uint64_t bodyId, std::size_t rigidIndex);
    void setCanBindRigid(std::function<bool()> callback) { canBindRigid_=std::move(callback); }
    bool unbindRigidBody(uint64_t bodyId);
    uint64_t rigidBodyId(uint64_t bodyId) const;
    bool hasRigidBindings() const { return !rigidBindings_.empty(); }
    void syncRigidBindings();

    /** @brief Rebuilds the fluid/solver and re-seeds the initial scene. */
    void reset();

    /** Rebuilds a deterministic fixed-carrier sphere. False leaves the scene intact. */
    bool sphericalPreset(bool combustion, float radius = 0.6f, float spacing = 0.08f);

    /**
     * @brief Whether the flame is part of the shared scene (drawn together with
     * the fluid / rigid / soft / glTF background). False at startup and after
     * clear() so the app still opens on an empty scene; becomes true when the
     * user opens the Flame page, presses Play / Step, or calls reset().
     */
    bool isPopulated() const { return populated_; }
    void setPopulated(bool p) { populated_ = p; }

    /** @brief Empty-scene reset (File > New): fresh initial scene, stopped, not drawn. */
    void clear();

    void setRunning(bool r);
    bool isRunning() const;

    /** @brief Advances the flame sim by one fixed step (getTimeStep()) if running. */
    void step();

    /** @brief One fixed-size step regardless of isRunning() (manual "Step"). */
    void stepOnce();

    /**
     * @brief Fixed simulation step (seconds, default 1/60). Survives reset() --
     * it belongs to the world, not to the rebuilt fluid -- so a scenario can
     * compare dt=1/60 against dt=1/120 on otherwise identical scenes.
     */
    bool setTimeStep(float dt) { if(!std::isfinite(dt) || dt<=0 || dt>1) return false; timeStep_=dt; return true; }
    float getTimeStep() const   { return timeStep_; }

    /** @brief Simulated seconds since the last reset(). */
    float getSimTime() const { return simTime_; }

    Phantom::Physics::FlameFluid&       fluid()        { return *fluid_; }
    const Phantom::Physics::FlameFluid& fluid() const  { return *fluid_; }
    Phantom::Physics::FlameSolver&      solver()       { return *solver_; }

    uint64_t addBody(Physics::CombustibleBody::Shape shape, const glm::vec3& center,
                     const glm::vec3& halfExtent, double fuel, int material = 1, int resolution = 2);
    bool removeBody(uint64_t id);
    Physics::CombustibleBody* findBody(uint64_t id);
    const std::vector<std::unique_ptr<Physics::CombustibleBody>>& bodies() const { return bodies_; }
    Physics::FlameSolidCoupler& coupler() { return coupler_; }
    const Physics::FlamePhysicalTransform& physicalTransform() const { return physicalTransform_; }
    bool combustionPreset(int resolution = 2, double fuelMass = 0.006);
    void stopSource();
    double gasFuelMass() const;
    int solidDebugColor = 0; // 0 char, 1 temperature, 2 fuel

    /** @brief Display-only parameters (were FlameApp members); never affect the sim. */
    struct RenderParams {
        int carrierDebug = 0; // 0 radiance, 1 temperature dots, 2 velocity vectors
        bool sphereWire = true;
        // Sprite diameters in *simulation* units (multiplied by renderScale
        // for display). Projected with each particle's own depth (plan A7);
        // the defaults reproduce the former fixed 14 px / 20 px sprites at
        // the default camera distance (120) and 720 px viewport.
        float particleSize      = 0.16f;
        float smokeParticleSize = 0.23f;
        bool  pbvrMode          = false;

        // Emission (plan Phase 3): blackbody radiance, 1 at the reference
        // temperature and HDR above it; ACES downstream rolls the core to white.
        float exposure               = 4.0f;
        bool  autoReferenceTemperature = true;   ///< track the (smoothed) hottest particle
        float referenceTemperature   = 2000.0f;  ///< used when auto is off
        // White balance (chromatic adaptation, Bradford): 0 = off (colorimetric
        // D65 -- a ~1500 K flame is nearly pure red), 1 = auto (adapt to the
        // radiance reference, i.e. the smoothed hottest temperature: the core
        // renders near-white, cooler gas orange/red), 2 = manual.
        int   whiteBalance            = 1;
        float whiteBalanceTemperature = 2000.0f; ///< manual white (K)
        float whiteBalanceDegree      = 0.8f;    ///< adaptation degree D (1 = full, 0 = none)
        // Absorption: smoke optical density = envelope * soot * smokeOpacityScale,
        // optical depth = smokeExtinction * density.
        float smokeOpacityScale = 1.0f;
        float smokeExtinction   = 2.0f;
        // Hot soot re-emits in proportion to what it absorbs (Kirchhoff).
        // Relative to one optically-thin gas sprite -- which sums over
        // overlapping neighbours -- an opaque soot sample needs ~the typical
        // overlap count to match, hence > 1.
        float smokeGlow         = 4.0f;
        float smokeAlbedo       = 0.03f;         ///< grey albedo x ambient light
        float smokeShadowStrength = 1.0f;       ///< PBVR self-shadow; 0 = off
        float smokeShadowAmbient = 0.25f;       ///< unoccluded fill fraction, 0..1
        float smokeFlameLight = 1.0f;           ///< PBVR flame illumination gain, 0=off
        float objectFlameLight = 1.0f;          ///< PBVR diffuse illumination of opaque objects

        // PBVR (plan Phase 4, FlamePBVRPass).
        bool  pbvrAdaptive       = true;  ///< EnsembleLodController picks R from GPU time
        int   pbvrEnsembles      = 4;     ///< R when not adaptive (1..8)
        int   pbvrTargetEnsembles = 64;   ///< static-frame convergence target
        float pbvrTemporalFrames = 4.0f;  ///< EMA length while running (0 = off)
        float pbvrBudgetMs       = 8.0f;
        float pbvrSubdivision    = 2.0f;  ///< sub-particle diameter = puff / subdivision
        float pbvrMinSubPixels   = 1.5f;  ///< never smaller than this on screen
        float pbvrDensityScale   = 1.0f;

        // Heat haze (FlameHazePass): screen-space refraction shimmer of whatever
        // is behind the flame. Display only, driven by the emitters' temperature.
        bool  hazeEnabled   = true;
        float hazeStrength  = 10.0f;  ///< peak displacement (px at full heat); 0 = off
        float hazeExtent    = 3.5f;   ///< haze sprite size relative to a flame sprite
        float hazeFrequency = 24.0f;  ///< noise cells per screen height
        float hazeRiseSpeed = 0.35f;  ///< screen heights / s the pattern climbs

        // Display transform onto FluidApp's shared FluidRenderer camera space.
        float     renderScale  = 12.0f;
        // Default: the flame's base sits at the world origin on the floor plane
        // (y = 0), where the rigid / soft presets and the glTF background live.
        glm::vec3 renderOffset { 0.0f, 0.0f, 0.0f };
    };

    RenderParams&       render()       { return render_; }
    const RenderParams& render() const { return render_; }

private:
    std::function<bool()> canBindRigid_;
    RigidBodyWorld* rigidWorld_=nullptr; // owner must outlive this FlameWorld
    struct RigidBinding { uint64_t bodyId, rigidId; };
    std::vector<RigidBinding> rigidBindings_;
    void releaseRigidClock();
    std::vector<std::unique_ptr<Physics::CombustibleBody>> bodies_;
    Physics::FlameSolidCoupler coupler_;
    Physics::FlamePhysicalTransform physicalTransform_;
    uint64_t nextBodyId_ = 1;
    std::unique_ptr<Phantom::Physics::FlameFluid>  fluid_;
    std::unique_ptr<Phantom::Physics::FlameSolver> solver_;
    bool         running_ = false;
    bool         populated_ = false;
    float        timeStep_ = 1.0f / 60.0f;
    float        simTime_  = 0.0f;
    RenderParams render_;

    void buildScene();
};

} // namespace Phantom
