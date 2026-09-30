#pragma once

#include "../Physics/CloudParticle.h"
#include "../Physics/CloudSolver.h"

#include <cstdint>
#include <string>
#include <vector>

namespace Phantom {

/**
 * @brief Standalone cloud (moist-air SPH) scene for PhysicsView
 * (docs/todo/PLAN_cloud_sph_pbvr.md Phase 2). Owns the particle state, the CPU
 * reference solver, the sources and the water ledger; like FlameWorld it is an
 * uncoupled subsystem with its own Play/Pause/Step/Reset. Vulkan / ImGui free.
 *
 * Time model: simulation time advances by `timeScale` simulated seconds per
 * real second while running. Each update() spends at most `maxSubstepsPerFrame`
 * solver steps; if the requested time cannot be covered the remainder is dropped
 * and counted in overloadedFrames()/droppedSimTime() (never silently absorbed).
 * Pause performs no update at all.
 */
class CloudWorld {
public:
    struct Config {
        Physics::CloudSolverParams solver;
        Physics::CloudThermoParams thermo;
        Physics::CloudEnvironmentParams environment;
        uint32_t seed = 1;
        double jitter = 0.02;             ///< Initial lattice jitter as a fraction of spacing (seeded).
        double timeScale = 1.0;           ///< Simulated seconds per real second.
        double fixedTimeStep = 0.0;       ///< > 0 forces dt (must be <= the stable limit); 0 = automatic.
        int maxSubstepsPerFrame = 8;
        double qcThreshold = 1.0e-5;      ///< Cloud base/top definition.
    };

    CloudWorld();

    /** @brief Rebuilds the air lattice from the config; keeps the source list. seed < 0 keeps the config seed. */
    void reset(int64_t seed = -1);

    Config& config() { return config_; }
    const Config& config() const { return config_; }

    void setRunning(bool r) { running_ = r; }
    bool isRunning() const { return running_; }

    /** @brief Real-time update: does nothing while paused. */
    void update(double realDt);
    /** @brief Advances `simSeconds` regardless of isRunning() (manual Step / headless). */
    void advance(double simSeconds);
    /** @brief One solver step of the automatic (or fixed) dt regardless of isRunning(). */
    void stepOnce();

    /** @brief Sources are addressed by index; the list survives reset(). */
    size_t addSource(const Physics::CloudSourceParams& s);
    bool setSource(size_t index, const Physics::CloudSourceParams& s);
    bool removeSource(size_t index);
    void clearSources() { sources_.clear(); }
    const std::vector<Physics::CloudSourceParams>& sources() const { return sources_; }

    /**
     * @brief Changes the environment relative humidity and rescales the vapour of every
     * particle by the same factor of its local saturation deficit (an external
     * supply/removal, recorded in the ledger), so the interactive "humidity" knob
     * acts on the existing air instead of only the buoyancy reference.
     */
    void setRelativeHumidity(double rh);
    void setWind(const Math::Vector3dd& wind) { config_.environment.wind = wind; syncSolver(); }

    const Physics::CloudParticleSoA& particles() const { return soa_; }
    const Physics::CloudWaterLedger& ledger() const { return ledger_; }
    Physics::CloudStats stats() const;
    const Physics::CloudSolverDiagnostics& diagnostics() const { return solver_.diagnostics(); }

    double simTime() const { return simTime_; }
    uint64_t stepCount() const { return stepCount_; }
    uint64_t overloadedFrames() const { return overloadedFrames_; }
    double droppedSimTime() const { return droppedSimTime_; }

    /** @brief One CSV statistics row / its header (plan section 8 output). */
    static std::string csvHeader();
    std::string csvRow() const;

private:
    void syncSolver();
    double chooseDt(double remaining) const;
    /** @brief Runs up to maxSubsteps steps toward `simSeconds`; returns the simulated time left over. */
    double run(double simSeconds, int maxSubsteps);

    Config config_;
    Physics::CloudSolver solver_;
    Physics::CloudParticleSoA soa_;
    Physics::CloudWaterLedger ledger_;
    std::vector<Physics::CloudSourceParams> sources_;
    bool running_ = false;
    double simTime_ = 0.0;
    uint64_t stepCount_ = 0;
    uint64_t overloadedFrames_ = 0;
    double droppedSimTime_ = 0.0;
};

}
