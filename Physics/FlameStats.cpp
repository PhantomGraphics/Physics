#include "pch.h"

#include "FlameStats.h"
#include "FlameFluid.h"
#include "FlameSolidCoupler.h"
#include "FlameSolver.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace Phantom::Math;
using namespace Phantom::Physics;

namespace {

bool isFinite(const Vector3df& v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

// Particles whose reaction rate is below this count as "not burning" for
// burningFraction (a rate of 1e-3/s would take ~15 minutes to consume the fuel).
constexpr float kBurningRateEps = 1.0e-3f;

}

FlameStats Phantom::Physics::computeFlameStats(const FlameFluid& fluid, const FlameSolidCoupler* coupler, const FlameSolver* solver)
{
	FlameStats st;
	st.boundaryEnergyLoss=fluid.boundaryEnergyLoss; st.maxAttemptedWallPenetration=fluid.maxAttemptedWallPenetration;
	st.burnedFuel=fluid.burnedFuelMass; st.outflowFuel=fluid.outflowFuelMass; st.sourceFuel=fluid.sourceFuelMass;
	const auto& gas=fluid.getParticles();
	for(size_t i=0;i<gas.size();++i) st.gasFuel+=gas.fuels[i]*fluid.getDensity()*std::pow(2*gas.radii[i],3.0f);
	if(coupler) {
		st.initialSolidFuel=coupler->initialFuelMass; st.removedSolidMass=coupler->removedMass;
		for(const auto* b:coupler->bodies()) { const auto s=b->stats(); st.bodies.push_back(s);
			st.solidFuel+=s.fuel; st.pendingFuel+=s.pending; st.residueMass+=s.residue;
			st.pyrolyzedMass+=s.pyrolyzed; st.solidHeatExchange+=s.heatExchange;
		}
	}
	st.fuelBalanceError=st.initialSolidFuel+fluid.initialFuelMass+st.sourceFuel-st.solidFuel-st.pendingFuel-st.residueMass-
		st.gasFuel-st.burnedFuel-st.outflowFuel-st.removedSolidMass;
	const auto& soa = fluid.getParticles();
	double sumVy = 0, sumTVy = 0;
	int capped = 0;
	for (size_t i = 0; i < soa.size(); ++i) {
		if (!isFinite(soa.positions[i]) || !isFinite(soa.velocities[i]) || !std::isfinite(soa.temperatures[i])) continue;
		const double mass = fluid.getDensity()*std::pow(2*soa.radii[i],3.0f);
		st.carrierMass += mass; st.totalHeat += mass*soa.temperatures[i];
		st.oxygenMass += mass*soa.oxygens[i];
		st.kineticEnergy += 0.5*mass*getLengthSquared(soa.velocities[i]);
		sumVy += mass*soa.velocities[i].y; sumTVy += mass*soa.temperatures[i]*soa.velocities[i].y;
		st.densityError = std::max(st.densityError, std::abs(soa.densities[i]/fluid.getDensity()-1));
		if (getLength(soa.velocities[i]) >= fluid.getMaxSpeed()*0.999f) ++capped;
		for (const auto& e:fluid.getEmitters()) {
            const auto d=soa.positions[i]-e.center;
            if(e.rate>0 && d.y>=-0.05f && d.y<=std::max(e.pilotHeight,0.12f) && d.x*d.x+d.z*d.z<=e.radius*e.radius) {
                ++st.sourceCarrierCount; break;
            }
        }
        if (solver && solver->getClosedBoundary()) {
			const auto& wall = *solver->getClosedBoundary();
            const auto center=solver->getBoundarySphere()?solver->getBoundarySphere()->getCenter():solver->getBoundaryCylinder()->getCenter();
            const float radius=solver->getBoundarySphere()?solver->getBoundarySphere()->getRadius():solver->getBoundaryCylinder()->getRadius();
			const auto offset = soa.positions[i]-center;
			const float radial = std::sqrt(offset.x*offset.x+offset.z*offset.z)/radius;
			if (radial < 0.3f) { ++st.coreCount; st.coreVelocityY += soa.velocities[i].y; }
			if (radial > 0.65f) { ++st.outerCount; st.outerVelocityY += soa.velocities[i].y; }
			st.maxWallPenetration = std::max(st.maxWallPenetration, soa.radii[i]-wall.getSignedDistance(soa.positions[i]));
		}
	}
	if (st.carrierMass > 0) {
		st.allAvgT = static_cast<float>(st.totalHeat/st.carrierMass);
		st.thermalVelocityCovariance = static_cast<float>(sumTVy/st.carrierMass-st.allAvgT*sumVy/st.carrierMass);
	}
	if (st.coreCount) st.coreVelocityY /= st.coreCount;
	if (st.outerCount) st.outerVelocityY /= st.outerCount;
	if (!soa.empty()) st.speedCapFraction = static_cast<float>(capped)/soa.size();
	st.sourceHeat=fluid.sourceHeat; st.wallHeat=fluid.wallHeat; st.clampHeat=fluid.clampHeat;
	st.heatBalanceError=fluid.initialHeat+fluid.sourceHeat+fluid.reactionHeat-fluid.wallHeat-
		fluid.coolingHeat-fluid.clampHeat-st.totalHeat;
	st.oxygenBalanceError=fluid.initialOxygenMass+fluid.sourceOxygenMass-fluid.consumedOxygen-st.oxygenMass;
	st.count = static_cast<int>(soa.size());
	st.secondaryCount = static_cast<int>(fluid.getSecondaryParticles().size());
	st.histMinT = fluid.getAmbientTemperature();

	if (soa.empty()) {
		st.histMaxT = st.histMinT;
		return st;
	}

	double sumY = 0.0, sumAirY = 0.0, sumSpeed = 0.0;
	double sumT = 0.0, sumFuel = 0.0, sumSoot = 0.0, sumOxygen = 0.0;
	int fuelCount = 0, burning = 0;
	st.maxY = -1.0e30f;
	st.maxT = -1.0e30f;

	std::vector<std::pair<float, float>> fuelTY; // (temperature, y) of non-air particles
	fuelTY.reserve(soa.size());

	for (size_t i = 0; i < soa.size(); ++i) {
		const auto& pos = soa.positions[i];
		const auto& vel = soa.velocities[i];
		const float t = soa.temperatures[i];
		if (!isFinite(pos) || !isFinite(vel) || !std::isfinite(t)) {
			++st.nanCount;
			continue;
		}
		const float speed = getLength(vel);
		sumY += pos.y;
		sumSpeed += speed;
		st.maxY = std::max(st.maxY, pos.y);
		st.maxSpeed = std::max(st.maxSpeed, speed);
		st.maxT = std::max(st.maxT, t);
		sumSoot += soa.soots[i];
		sumOxygen += soa.oxygens[i];

		if (fluid.computeReactionRate(t, soa.fuels[i], soa.oxygens[i]) > kBurningRateEps) {
			++burning;
		}

		if (soa.airs[i]) {
			++st.airCount;
			sumAirY += pos.y;
		} else {
			++fuelCount;
			sumT += t;
			sumFuel += soa.fuels[i];
			fuelTY.emplace_back(t, pos.y);
		}
	}

	const int finite = st.count - st.nanCount;
	if (finite > 0) {
		st.avgY = static_cast<float>(sumY / finite);
		st.avgSpeed = static_cast<float>(sumSpeed / finite);
		st.avgSoot = static_cast<float>(sumSoot / finite);
		st.avgOxygen = static_cast<float>(sumOxygen / finite);
		st.burningFraction = static_cast<float>(burning) / static_cast<float>(finite);
	} else {
		st.maxY = 0.0f;
		st.maxT = 0.0f;
	}
	if (st.airCount > 0) {
		st.airAvgY = static_cast<float>(sumAirY / st.airCount);
	}

	st.histMaxT = std::max(st.maxT, st.histMinT + 1.0f);
	if (fuelCount > 0) {
		st.avgT = static_cast<float>(sumT / fuelCount);
		st.avgFuel = static_cast<float>(sumFuel / fuelCount);

		const float range = st.histMaxT - st.histMinT;
		for (const auto& [t, y] : fuelTY) {
			(void)y;
			int bin = static_cast<int>((t - st.histMinT) / range * FlameStats::kHistogramBins);
			bin = std::clamp(bin, 0, FlameStats::kHistogramBins - 1);
			++st.tempHistogram[bin];
		}

		// Hottest 10% (at least one particle).
		const size_t hotN = std::max<size_t>(1, fuelTY.size() / 10);
		std::nth_element(fuelTY.begin(), fuelTY.begin() + (hotN - 1), fuelTY.end(),
			[](const auto& a, const auto& b) { return a.first > b.first; });
		double sumHotY = 0.0;
		for (size_t k = 0; k < hotN; ++k) {
			sumHotY += fuelTY[k].second;
		}
		st.hotY = static_cast<float>(sumHotY / hotN);
	}
	return st;
}

const char* FlameStats::names()
{
	return "sourceCarrierCount,count,airCount,secondaryCount,nanCount,avgY,maxY,airAvgY,avgSpeed,maxSpeed,"
		"boundaryEnergyLoss,maxAttemptedWallPenetration,"
		"carrierMass,totalHeat,oxygenMass,kineticEnergy,heatBalanceError,oxygenBalanceError,sourceHeat,wallHeat,clampHeat,"
		"allAvgT,coreVelocityY,outerVelocityY,thermalVelocityCovariance,maxWallPenetration,densityError,speedCapFraction,coreCount,outerCount,"
		"avgT,maxT,hotY,avgFuel,avgSoot,avgOxygen,burningFraction,gasFuel,burnedFuel,outflowFuel,sourceFuel,"
		"solidFuel,pendingFuel,residueMass,pyrolyzedMass,solidHeatExchange,initialSolidFuel,removedSolidMass,fuelBalanceError";
}

bool FlameStats::get(const std::string& name, float& out) const
{
#define MASS(n) if(name==#n) { out=static_cast<float>(n); return true; }
	MASS(sourceCarrierCount) MASS(boundaryEnergyLoss) MASS(maxAttemptedWallPenetration)
	MASS(carrierMass) MASS(totalHeat) MASS(oxygenMass) MASS(kineticEnergy) MASS(heatBalanceError) MASS(oxygenBalanceError)
	MASS(sourceHeat) MASS(wallHeat) MASS(clampHeat) MASS(allAvgT) MASS(coreVelocityY) MASS(outerVelocityY)
	MASS(thermalVelocityCovariance) MASS(maxWallPenetration) MASS(densityError) MASS(speedCapFraction) MASS(coreCount) MASS(outerCount)
	MASS(gasFuel) MASS(burnedFuel) MASS(outflowFuel) MASS(sourceFuel) MASS(solidFuel) MASS(pendingFuel)
	MASS(residueMass) MASS(pyrolyzedMass) MASS(solidHeatExchange) MASS(initialSolidFuel) MASS(removedSolidMass) MASS(fuelBalanceError)
#undef MASS
	if (name == "count") { out = static_cast<float>(count); return true; }
	if (name == "airCount") { out = static_cast<float>(airCount); return true; }
	if (name == "secondaryCount") { out = static_cast<float>(secondaryCount); return true; }
	if (name == "nanCount") { out = static_cast<float>(nanCount); return true; }
	if (name == "avgY") { out = avgY; return true; }
	if (name == "maxY") { out = maxY; return true; }
	if (name == "airAvgY") { out = airAvgY; return true; }
	if (name == "avgSpeed") { out = avgSpeed; return true; }
	if (name == "maxSpeed") { out = maxSpeed; return true; }
	if (name == "avgT") { out = avgT; return true; }
	if (name == "maxT") { out = maxT; return true; }
	if (name == "hotY") { out = hotY; return true; }
	if (name == "avgFuel") { out = avgFuel; return true; }
	if (name == "avgSoot") { out = avgSoot; return true; }
	if (name == "avgOxygen") { out = avgOxygen; return true; }
	if (name == "burningFraction") { out = burningFraction; return true; }
	return false;
}

std::string FlameStats::toString() const
{
	char buf[512];
	std::snprintf(buf, sizeof(buf),
		"count=%d;air=%d;secondary=%d;nan=%d;avgY=%.4f;maxY=%.4f;airAvgY=%.4f;avgSpeed=%.4f;maxSpeed=%.4f;"
		"avgT=%.1f;maxT=%.1f;hotY=%.4f;avgFuel=%.4f;avgSoot=%.4f;avgOxygen=%.4f;burning=%.3f;histT=[%.0f,%.0f]",
		count, airCount, secondaryCount, nanCount, avgY, maxY, airAvgY, avgSpeed, maxSpeed,
		avgT, maxT, hotY, avgFuel, avgSoot, avgOxygen, burningFraction, histMinT, histMaxT);
	std::string s(buf);
	for (const char* name : {"sourceCarrierCount", "maxT", "burningFraction", "speedCapFraction", "sourceFuel", "carrierMass", "totalHeat", "allAvgT", "coreVelocityY", "outerVelocityY",
		"thermalVelocityCovariance", "maxWallPenetration", "heatBalanceError", "oxygenBalanceError", "sourceHeat", "wallHeat"}) {
		float value; get(name,value); s += std::string(";")+name+"="+std::to_string(value);
	}
	s += ";h=";
	for (int b = 0; b < kHistogramBins; ++b) {
		if (b > 0) {
			s += '/';
		}
		s += std::to_string(tempHistogram[b]);
	}
	s+=";solidFuel="+std::to_string(solidFuel)+";pendingFuel="+std::to_string(pendingFuel)+
		";residueMass="+std::to_string(residueMass)+";burnedFuel="+std::to_string(burnedFuel)+
		";outflowFuel="+std::to_string(outflowFuel)+";fuelBalanceError="+std::to_string(fuelBalanceError);
	return s;
}
