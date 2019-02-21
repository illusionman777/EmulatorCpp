#include "pch.h"
#include "InternalForces.h"
#include "Support.h"


InternalForces::InternalForces(Copter& copter,
                               CopterDynamicProperties& dynamicProperties) :
	copter_(copter), dynamicProperties_(dynamicProperties)
{
	gyroscopicMoment_ = zeros(NUM_OF_SPACE_DIM);
	const int numOfEngines = copter_.getNumOfEngines();
	enginesGyroMoments_.resize(numOfEngines);
	enginesDriveMoments_.resize(numOfEngines);
}

void InternalForces::compute()
{
	gyroscopicMoment_ = zeros(NUM_OF_SPACE_DIM);
	const auto enginesRotMatrices = dynamicProperties_.getEnginesRotMatrices();
	const auto engInertiaMomentsCur = dynamicProperties_.getEngInertiaMomentsCur();
	const int numOfEngines = copter_.getNumOfEngines();
	for (int i = 0; i < numOfEngines; i++) {
		auto& engine = copter_.engines[i];
		const double driveMomentTmp = engine.getCurrentPower() / engine.maxPower *
			engine.maxDriveMoment;
		double driveMoment = 0.0;
		const double angularVelocityNorm = arma::norm(engine.angularVelocity);
		if (isZero(angularVelocityNorm)) {
			driveMoment = driveMomentTmp;
		}
		else {
			driveMoment = engine.getCurrentPower() / angularVelocityNorm;
			driveMoment = std::min(driveMoment, engine.maxDriveMoment);
		}
		if (engine.rotationDir == "clockwise") {
			driveMoment = -driveMoment;
		}
		enginesDriveMoments_[i] = driveMoment;
		const vec engAngVelCur = enginesRotMatrices[i] * engine.angularVelocity;
		vec gyroMomentTmp = arma::cross(copter_.fuselage.angularVelocity,
			engAngVelCur);
		gyroMomentTmp = engInertiaMomentsCur[i] * gyroMomentTmp;
		enginesGyroMoments_[i] = gyroMomentTmp;
		gyroscopicMoment_ += gyroMomentTmp;
	}
}

vec InternalForces::getGyroscopicMoment() const
{
	return gyroscopicMoment_;
}

std::vector<vec> InternalForces::getEnginesGyroMoments() const
{
	return enginesGyroMoments_;
}

std::vector<double> InternalForces::getEnginesDriveMoments() const
{
	return enginesDriveMoments_;
}
