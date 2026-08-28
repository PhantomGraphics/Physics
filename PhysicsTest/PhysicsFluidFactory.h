#pragma once

#include "../Physics/DFSPHFluid.h"
#include "../Physics/PBSPHFluid.h"
#include "../Physics/WCSPHFluid.h"
#include "../Physics/ISPHSolver.h"
#include "../Physics/SPHKernel.h"
#include "CGLib/Util/UnCopyable.h"
#include "CGLib/Math/Box3d.h"

#include <memory>
#include <vector>

namespace Phantom {
    namespace Physics {

        /**
         * @brief Builds a Fluid object of the requested SPH flavor together with a
         * matching ISPHSolver, and keeps the Fluid alive so its particle data can
         * still be queried afterward.
         *
         * This is deliberately separate from PhysicsSolver: PhysicsSolver's job is
         * to step an ISPHSolver it is handed (plus RigidBodySolver/RigidFluidSolver
         * coupling), not to know how to seed particles on a grid or which Fluid
         * subtype backs a given FluidType. A typical caller:
         *
         * @code
         * PhysicsFluidFactory factory;
         * factory.setFluidType(PhysicsFluidFactory::FluidType::DFSPH);
         * factory.params().fluidBounds = myBounds;
         * auto fluidSolver = factory.build();  // caller owns this (Tier 1 -- see Physics/CLAUDE.md)
         * physicsSolver.setFluidSolver(fluidSolver.get());
         * // ... physicsSolver.step() advances it; factory.getParticlePositions()
         * // still reads the live particle data for rendering/inspection. fluidSolver
         * // must outlive physicsSolver's use of it.
         * @endcode
         */
        class PhysicsFluidFactory : private UnCopyable {
        public:
            enum class FluidType { DFSPH, PBSPH, CSPH };

            struct Params {
                float radius = 1.0f;
                float effectLength = 2.25f;
                float density = 1.0f;
                float stiffness = 20.0f;
                float viscosity = 5.0f;
                // Only used to size the domain boundary's penalty response at
                // construction time -- unrelated to PhysicsSolver's own per-step dt.
                float boundaryTimeStep = 0.01f;
                Math::Vector3df gravity = { 0.f, -9.8f, 0.f };
                Math::Box3df boundary = Math::Box3df(
                    Math::Vector3df(0.f, 0.f, 0.f), Math::Vector3df(100.f, 100.f, 100.f));
                Math::Box3df fluidBounds = Math::Box3df(
                    Math::Vector3df(0.f, 0.f, 0.f), Math::Vector3df(80.f, 80.f, 80.f));
            };

            void      setFluidType(FluidType t) { fluidType_ = t; }
            FluidType getFluidType() const { return fluidType_; }

            Params& params() { return params_; }
            const Params& params() const { return params_; }

            // Overrides the uniform-grid particle seeding for the next build().
            // Pass empty vectors to revert to params().fluidBounds sampling.
            void setCustomInitialData(std::vector<Math::Vector3df> positions,
                std::vector<Math::Vector3df> velocities = {});

            /**
             * @brief Discards any previously built fluid and builds a fresh one (plus
             * a matching solver already registered with it) from fluidType()/params().
             * @return The new solver, owned by the caller. Keep it alive and pass
             *         its raw pointer to PhysicsSolver::setFluidSolver().
             */
            std::unique_ptr<ISPHSolver> build();

            size_t                        getParticleCount() const;
            std::vector<Math::Vector3df>  getParticlePositions() const;

            /** @brief Non-null only after build() with FluidType::DFSPH/PBSPH. */
            SPHKernel* getActiveKernel() const;
            float      getActiveRestDensity() const;

        private:
            FluidType fluidType_ = FluidType::DFSPH;
            Params    params_;

            std::unique_ptr<DFSPHFluid>  dfsphFluid_;
            std::unique_ptr<PBSPHFluid>  pbsphFluid_;
            std::unique_ptr<WCSPHFluid>   csphFluid_;

            std::vector<Math::Vector3df> customPositions_;
            std::vector<Math::Vector3df> customVelocities_;

            void clear();
            std::unique_ptr<ISPHSolver> buildDFSPH();
            std::unique_ptr<ISPHSolver> buildPBSPH();
            std::unique_ptr<ISPHSolver> buildWCSPH();
        };

    } // namespace Physics
} // namespace Phantom
