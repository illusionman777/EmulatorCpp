#include "pch.h"
#include "CopterFunction.h"
#include "ExternalForces.h"
#include "InternalForces.h"
#include "OdeMatrices.h"
#include "Support.h"


CopterFunction::CopterFunction(Copter& copter) : copter_(copter)
{
}

// This function create and solve
// ODE system A * X = B where X is a vector of the highest derivatives
mat CopterFunction::compute(const double timeCur, const mat& stateCur) const
{
	copterPosFromMat(stateCur);
	auto dynamicProperties = CopterDynamicProperties(copter_);
	dynamicProperties.computeCurProperties();
	auto exForces = ExternalForces(copter_, dynamicProperties);
	auto inForces = InternalForces(copter_, dynamicProperties);
	exForces.compute();
	inForces.compute();
	const vec mainForce = exForces.getMainForce();
	const vec mainMoment = exForces.getMainMoment();
	copter_.fuselage.acceleration = mainForce / copter_.getMass();

	auto odeMatrices = OdeMatrices(copter_, dynamicProperties);
	odeMatrices.compute(exForces, inForces);
	const mat A = odeMatrices.getCoefMatrix();
	const vec B = odeMatrices.getConstantTermsVector();
	vec X = inv(A) * B;

	for (int i = 0; i < NUM_OF_SPACE_DIM; i++) {
		copter_.fuselage.angularAcceleration[i] = X[i];
	}
	const int numOfEngines = copter_.getNumOfEngines();
	for (int i = 0; i < numOfEngines; i++) {
		auto& engine = copter_.engines[i];
		const vec vectorFusEngineMc = copter_.getVectorFusEngineMc(i);
		const vec relativeVelocity = arma::cross(copter_.fuselage.angularVelocity,
		                                         vectorFusEngineMc);
		engine.velocity = copter_.fuselage.velocity + relativeVelocity;
		engine.acceleration = copter_.fuselage.acceleration +
			arma::cross(copter_.fuselage.angularAcceleration, vectorFusEngineMc) +
			arma::cross(copter_.fuselage.angularVelocity,
			            relativeVelocity);
		engine.angularAcceleration[2] = X[i + NUM_OF_SPACE_DIM];
	}
	mat result = compileOutputMat();
	return result;
}

void CopterFunction::copterPosFromMat(const mat& stateCur) const
{
	for (int i = 0; i < NUM_OF_SPACE_DIM; i++) {
		copter_.fuselage.position[i] = stateCur(0, i);
		copter_.fuselage.velocity[i] = stateCur(2, i);
		copter_.fuselage.angularVelocity[i] = stateCur(3, i);
	}
	copter_.fuselage.rotationQuat = Quaternion(
		stateCur(1, 0),
		stateCur(1, 1),
		stateCur(1, 2),
		stateCur(1, 3));
	copter_.fuselage.rotationQuat.normalize();
	auto angularVelQuat = Quaternion(copter_.fuselage.angularVelocity);
	copter_.fuselage.rotationQuatDiff = angularVelQuat *
		copter_.fuselage.rotationQuat / 2;
	const int matRows = NUM_OF_PHYS_PARAMS * (1 + copter_.getNumOfEngines());
	for (int i = 0; i < matRows - NUM_OF_PHYS_PARAMS; i += NUM_OF_PHYS_PARAMS) {
		auto& engine = copter_.engines[i / NUM_OF_PHYS_PARAMS];
		for (int j = 0; j < NUM_OF_SPACE_DIM; j++) {
			engine.position[j] = stateCur(i + NUM_OF_PHYS_PARAMS, j);
			engine.velocity[j] = stateCur(i + NUM_OF_PHYS_PARAMS + 2, j);
			engine.angularVelocity[j] = stateCur(i + NUM_OF_PHYS_PARAMS + 3, j);
		}
		engine.rotationQuat = Quaternion(
			stateCur(i + NUM_OF_PHYS_PARAMS + 1, 0),
			stateCur(i + NUM_OF_PHYS_PARAMS + 1, 1),
			stateCur(i + NUM_OF_PHYS_PARAMS + 1, 2),
			stateCur(i + NUM_OF_PHYS_PARAMS + 1, 3));
		engine.rotationQuat.normalize();
		angularVelQuat = Quaternion(engine.angularVelocity);
		engine.rotationQuatDiff = angularVelQuat * engine.rotationQuat / 2;
	}
}

//               |Vel.X     Vel.Y     Vel.Z     0      |
//               |dQuat.W   dQuat.X   dQuat.Y   dQuat.Z|
//               |Acel.X    Acel.Y    Acel.Z    0      |
//  mat Output = |AngAcel.X AngAcel.Y AngAcel.Z 0      |
//               |...                                  |
//               |Same 4 parameters for every engine   |
//               |                                     |
mat CopterFunction::compileOutputMat() const
{
	const int matRows = NUM_OF_PHYS_PARAMS * (1 + copter_.getNumOfEngines());
	mat result = mat(matRows, QUAT_LENGTH);
	for (int i = 0; i < NUM_OF_SPACE_DIM; i++) {
		result(0, i) = copter_.fuselage.velocity(i);
		result(2, i) = copter_.fuselage.acceleration(i);
		result(3, i) = copter_.fuselage.angularAcceleration(i);
	}
	for (int i = 0; i < QUAT_LENGTH; i++) {
		result(1, i) = copter_.fuselage.rotationQuatDiff.at(i);
	}
	for (int i = 0; i < matRows - NUM_OF_PHYS_PARAMS; i += NUM_OF_PHYS_PARAMS) {
		auto& engine = copter_.engines[i / NUM_OF_PHYS_PARAMS];
		for (int j = 0; j < NUM_OF_SPACE_DIM; j++) {
			result(i + NUM_OF_PHYS_PARAMS, j) = engine.velocity(j);
			result(i + NUM_OF_PHYS_PARAMS + 2, j) = engine.acceleration(j);
			result(i + NUM_OF_PHYS_PARAMS + 3, j) = engine.angularAcceleration(j);
		}
		for (int j = 0; j < QUAT_LENGTH; j++) {
			result(i + NUM_OF_PHYS_PARAMS + 1, j) = engine.rotationQuatDiff.at(j);
		}
	}
	return result;
}
