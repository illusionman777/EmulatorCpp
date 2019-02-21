#include "pch.h"

#include <corecrt_math_defines.h>

#include "ExternalForces.h"
#include "Support.h"


ExternalForces::ExternalForces(Copter& copter,
                               CopterDynamicProperties& dynamicProperties) :
	copter_(copter), dynamicProperties_(dynamicProperties)
{
	const double copterAltitude = copter_.fuselage.position(2);
	airDensity_ = getAirDensity(copterAltitude);
	mainForce_ = zeros(NUM_OF_SPACE_DIM);
	mainMoment_ = zeros(NUM_OF_SPACE_DIM);
	const int numOfEngines = copter_.getNumOfEngines();
	enginesAeroForces_.resize(numOfEngines);
	enginesAeroMoments_.resize(numOfEngines);
}

void ExternalForces::compute()
{
	const double copterAltitude = copter_.fuselage.position(2);
	airDensity_ = getAirDensity(copterAltitude);
	computeForces();
	computeMoments();
}

void ExternalForces::computeForces()
{
	auto gravityForce = vec(NUM_OF_SPACE_DIM);
	gravityForce(0) = 0.0;
	gravityForce(1) = 0.0;
	gravityForce(2) = -copter_.getMass() * getG();
	const vec dragCoef = dynamicProperties_.getFusRotMatrix() * copter_.dragCoef;
	const vec fuselageAirForce = airDensity_ * copter_.aeroSquare / 2 *
		norm(dragCoef % copter_.fuselage.velocity) *
		-copter_.fuselage.velocity;
	vec bladeAirForceSum = zeros(NUM_OF_SPACE_DIM);
	auto bladeAirForceCur = vec(NUM_OF_SPACE_DIM);
	auto enginesRotMatrices = dynamicProperties_.getEnginesRotMatrices();
	const int numOfEngines = copter_.getNumOfEngines();
	for (int i = 0; i < numOfEngines; i++) {
		auto& engine = copter_.engines[i];
		bladeAirForceCur = 2 * M_PI * engine.bladeCoefAlpha * airDensity_ *
			engine.angularVelocity % engine.angularVelocity %
			arma::sign(engine.angularVelocity) *
			engine.bladeDiameter * engine.bladeDiameter *
			engine.bladeDiameter * engine.bladeDiameter;
		if (engine.bladeDir == "clockwise") {
			bladeAirForceCur = -bladeAirForceCur;
		}
		enginesAeroForces_[i] = enginesRotMatrices[i] * bladeAirForceCur;
		bladeAirForceSum += enginesAeroForces_[i];
	}
	mainForce_ = gravityForce + fuselageAirForce + bladeAirForceSum;
}

void ExternalForces::computeMoments()
{
	const vec momentDragCoef = dynamicProperties_.getFusRotMatrix() *
		copter_.momentDragCoef;
	const vec fuselageAirMoment = 8.0 / 27.0 * airDensity_ *
		arma::norm(momentDragCoef % copter_.fuselage.angularVelocity) *
		-copter_.fuselage.angularVelocity *
		copter_.aeroSquare * copter_.aeroSquare * std::sqrt(copter_.aeroSquare);
	vec bladeAirMomentSum = zeros(NUM_OF_SPACE_DIM);
	vec bladeForceMomentSum = zeros(NUM_OF_SPACE_DIM);
	auto vectorFusEngineMcCur = vec(NUM_OF_SPACE_DIM);
	auto enginesRotMatrices = dynamicProperties_.getEnginesRotMatrices();
	const int numOfEngines = copter_.getNumOfEngines();
	for (int i = 0; i < numOfEngines; i++) {
		auto& engine = copter_.engines[i];
		const double bladeAirMomentCur = 2 * M_PI * engine.bladeCoefBeta *
			airDensity_ * engine.bladeDiameter *
			engine.bladeDiameter * engine.bladeDiameter *
			engine.bladeDiameter * engine.bladeDiameter;
		const double angularVelocityRealtive = engine.angularVelocity(2);
		enginesAeroMoments_[i] = bladeAirMomentCur * angularVelocityRealtive *
			angularVelocityRealtive * arma::sign(-angularVelocityRealtive);
		const vec angularVelocityGlobal = enginesRotMatrices[i] *
			engine.angularVelocity;
		bladeAirMomentSum += bladeAirMomentCur * angularVelocityGlobal %
			angularVelocityGlobal % arma::sign(-angularVelocityGlobal);
		vectorFusEngineMcCur = dynamicProperties_.getFusRotMatrix() * 
			copter_.getVectorFusEngineMc(i);
		bladeForceMomentSum += arma::cross(vectorFusEngineMcCur,
		                                   enginesAeroForces_[i]);
	}
	mainMoment_ = fuselageAirMoment + bladeForceMomentSum + bladeAirMomentSum;
}

// gravity const
double ExternalForces::getG()
{
	return 9.8062;
}

// air density at sea level
double ExternalForces::getRho0()
{
	return 1.225;
}

// standart atmosphere formula for air density at altitude H
double ExternalForces::getAirDensity(const double altitude)
{
	if (altitude < 0.0)
		return getRho0();
	return getRho0() * (20000.0 - altitude) / (20000.0 + altitude);
}

vec ExternalForces::getMainForce() const
{
	return mainForce_;
}

vec ExternalForces::getMainMoment() const
{
	return mainMoment_;
}

std::vector<vec> ExternalForces::getEnginesAeroForces() const
{
	return enginesAeroForces_;
}

std::vector<double> ExternalForces::getEnginesAeroMoments() const
{
	return enginesAeroMoments_;
}
