#include "pch.h"
#include "PhysicsFluidFactory.h"

#include "../Physics/DFSPHSolver.h"
#include "../Physics/DFSPHParticle.h"
#include "../Physics/PBSPHSolver.h"
#include "../Physics/PBSPHParticle.h"
#include "../Physics/WCSPHSolver.h"
#include "../Physics/WCSPHParticle.h"

namespace {

    using Phantom::Math::Vector3df;
    using Phantom::Math::Box3df;

    template <class Fn>
    void forEachFluidSeedInBounds(const Box3df& bounds, float diameter, Fn&& fn)
    {
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
    namespace Physics {

        void PhysicsFluidFactory::setCustomInitialData(std::vector<Math::Vector3df> positions,
            std::vector<Math::Vector3df> velocities)
        {
            customPositions_ = std::move(positions);
            customVelocities_ = std::move(velocities);
        }

        std::unique_ptr<ISPHSolver> PhysicsFluidFactory::build()
        {
            clear();
            switch (fluidType_) {
            case FluidType::DFSPH: return buildDFSPH();
            case FluidType::PBSPH: return buildPBSPH();
            case FluidType::CSPH:  return buildWCSPH();
            }
            return nullptr;
        }

        size_t PhysicsFluidFactory::getParticleCount() const
        {
            switch (fluidType_) {
            case FluidType::DFSPH: return dfsphFluid_ ? dfsphFluid_->getParticles().size() : 0;
            case FluidType::PBSPH: return pbsphFluid_ ? pbsphFluid_->getParticles().size() : 0;
            case FluidType::CSPH:  return csphFluid_ ? csphFluid_->getParticles().size() : 0;
            }
            return 0;
        }

        std::vector<Math::Vector3df> PhysicsFluidFactory::getParticlePositions() const
        {
            std::vector<Math::Vector3df> positions;
            positions.reserve(getParticleCount());

            switch (fluidType_) {
            case FluidType::DFSPH:
                if (dfsphFluid_) {
                    for (const auto& pos : dfsphFluid_->getParticles().positions)
                        positions.push_back(pos);
                }
                break;
            case FluidType::PBSPH:
                if (pbsphFluid_) {
                    for (const auto& pos : pbsphFluid_->getParticles().positions)
                        positions.push_back(pos);
                }
                break;
            case FluidType::CSPH:
                if (csphFluid_) {
                    for (const auto& pos : csphFluid_->getParticles().positions)
                        positions.push_back(pos);
                }
                break;
            }

            return positions;
        }

        SPHKernel* PhysicsFluidFactory::getActiveKernel() const
        {
            switch (fluidType_) {
            case FluidType::DFSPH: return dfsphFluid_ ? dfsphFluid_->getKernel() : nullptr;
            case FluidType::PBSPH: return pbsphFluid_ ? pbsphFluid_->getKernel() : nullptr;
            default: return nullptr;
            }
        }

        float PhysicsFluidFactory::getActiveRestDensity() const
        {
            switch (fluidType_) {
            case FluidType::DFSPH: return dfsphFluid_ ? dfsphFluid_->getDensity() : params_.density;
            case FluidType::PBSPH: return pbsphFluid_ ? pbsphFluid_->getRestDensity() : params_.density;
            default: return params_.density;
            }
        }

        void PhysicsFluidFactory::clear()
        {
            dfsphFluid_.reset();

            pbsphFluid_.reset();

            csphFluid_.reset();
        }

        std::unique_ptr<ISPHSolver> PhysicsFluidFactory::buildDFSPH()
        {
            dfsphFluid_ = std::make_unique<DFSPHFluid>();
            dfsphFluid_->setEffectLength(params_.effectLength);
            dfsphFluid_->density = params_.density;
            dfsphFluid_->viscosityCoe = params_.viscosity;

            const float diameter = params_.radius * 2.0f;
            const float mass = diameter * diameter * diameter;

            dfsphFluid_->density = DFSPHSolver::calculateRestDensity(
                params_.effectLength, params_.radius, mass, dfsphFluid_.get());

            if (!customPositions_.empty()) {
                for (size_t i = 0; i < customPositions_.size(); ++i) {
                    dfsphFluid_->createParticle(customPositions_[i], params_.radius, mass);
                    if (i < customVelocities_.size())
                        dfsphFluid_->getParticles().velocities.back() = customVelocities_[i];
                }
            }
            else {
                forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
                    dfsphFluid_->createParticle(pos, params_.radius, mass);
                    });
            }

            auto solver = std::make_unique<DFSPHSolver>();
            solver->setTimeStep(params_.boundaryTimeStep);
            solver->setBoundary(params_.boundary, params_.boundaryTimeStep);
            solver->setExternalForce(params_.gravity);
            solver->add(dfsphFluid_.get());
            return solver;
        }

        std::unique_ptr<ISPHSolver> PhysicsFluidFactory::buildPBSPH()
        {
            pbsphFluid_ = std::make_unique<PBSPHFluid>();
            pbsphFluid_->setEffectLength(params_.effectLength);
            pbsphFluid_->setRestDensity(params_.density);
            pbsphFluid_->setStiffness(params_.stiffness);
            pbsphFluid_->setVicsosity(params_.viscosity);

            const float diameter = params_.radius * 2.0f;

            if (!customPositions_.empty()) {
                for (size_t i = 0; i < customPositions_.size(); ++i) {
                    pbsphFluid_->createParticle(customPositions_[i], params_.radius);
                    if (i < customVelocities_.size())
                        pbsphFluid_->getParticles().velocities.back() = customVelocities_[i];
                }
            }
            else {
                forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
                    pbsphFluid_->createParticle(pos, params_.radius);
                    });
            }

            auto solver = std::make_unique<PBSPHSolver>();
            solver->setTimeStep(params_.boundaryTimeStep);
            solver->setBoundary(params_.boundary, params_.boundaryTimeStep);
            solver->setExternalForce(params_.gravity);
            solver->add(pbsphFluid_.get());
            return solver;
        }

        std::unique_ptr<ISPHSolver> PhysicsFluidFactory::buildWCSPH()
        {
            csphFluid_ = std::make_unique<WCSPHFluid>();
            csphFluid_->setPressureCoe(params_.stiffness);
            csphFluid_->setVicosityCoe(params_.viscosity);
            csphFluid_->setEffectLength(params_.effectLength);
            csphFluid_->setDensity(params_.density);

            const float diameter = params_.radius * 2.0f;

            if (!customPositions_.empty()) {
                for (size_t i = 0; i < customPositions_.size(); ++i) {
                    csphFluid_->createParticle(customPositions_[i], params_.radius);
                    if (i < customVelocities_.size())
                        csphFluid_->getParticles().velocities.back() = customVelocities_[i];
                }
            }
            else {
                forEachFluidSeedInBounds(params_.fluidBounds, diameter, [&](const Vector3df& pos) {
                    csphFluid_->createParticle(pos, params_.radius);
                    });
            }

            auto solver = std::make_unique<WCSPHSolver>();
            solver->setEffectLength(params_.effectLength);
            solver->setTimeStep(params_.boundaryTimeStep);
            solver->setBoundary(params_.boundary, params_.boundaryTimeStep);
            solver->setExternalForce(params_.gravity);
            solver->add(csphFluid_.get());
            return solver;
        }

    } // namespace Physics
} // namespace Phantom
