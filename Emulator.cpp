#include "pch.h"
#include "Emulator.h"
#include "Support.h"


Emulator::Emulator(Copter& copter, const double dt) : copter_(copter)
{
	function_ = new CopterFunction(copter);
	rungeKutta_ = RungeKuttaMethod(function_, dt);
	startStateInitialized_ = false;
	dt_ = dt;
}

double Emulator::getCurrentTime() const
{
	return rungeKutta_.getCurrentTime();
}

//                       |Pos.X     Pos.Y     Pos.Z     0      |
//                       |Quat.W    Quat.X    Quat.Y    Quat.Z |
//                       |Vel.X     Vel.Y     Vel.Z     0      |
//  mat initConditions = |AngVel.X  AngVel.Y  AngVel.Z  0      |
//                       |...                                  |
//                       |Same 4 parameters for every engine   |
//                       |                                     |
void Emulator::initializeStartState()
{
	const int matRows = NUM_OF_PHYS_PARAMS * (1 + copter_.getNumOfEngines());
	initialConditions_ = mat(matRows, QUAT_LENGTH);
	for (int i = 0; i < 3; i++) {
		initialConditions_(0, i) = copter_.fuselage.position(i);
		initialConditions_(2, i) = copter_.fuselage.velocity(i);
		initialConditions_(3, i) = copter_.fuselage.angularVelocity(i);
	}
	for (int i = 0; i < QUAT_LENGTH; i++) {
		initialConditions_(1, i) = copter_.fuselage.rotationQuat.at(i);
	}
	for (int i = 0; i < matRows - NUM_OF_PHYS_PARAMS; i += NUM_OF_PHYS_PARAMS) {
		auto& engine = copter_.engines[i / NUM_OF_PHYS_PARAMS];
		for (int j = 0; j < NUM_OF_SPACE_DIM; j++) {
			initialConditions_(i + NUM_OF_PHYS_PARAMS, j) = engine.position(j);
			initialConditions_(i + NUM_OF_PHYS_PARAMS + 2, j) = engine.velocity(j);
			initialConditions_(i + NUM_OF_PHYS_PARAMS + 3, j) = engine.angularVelocity(j);
		}
		for (int j = 0; j < QUAT_LENGTH; j++) {
			initialConditions_(i + NUM_OF_PHYS_PARAMS + 1, j) = engine.rotationQuat.at(j);
		}
	}
	startStateInitialized_ = true;
}

std::vector<std::vector<double>> Emulator::computeNextState(
	const std::vector<int>& currentPwm, const double endTime)
{
	if (!startStateInitialized_)
		this->initializeStartState();
	const int numOfEngines = copter_.getNumOfEngines();
	for (int i = 0; i < numOfEngines; i++) {
		copter_.engines[i].setCurrentPwm(currentPwm[i]);
	}
	double currentTime = rungeKutta_.getCurrentTime();
	while (currentTime < endTime) {
		const mat inputListNext = rungeKutta_.computeNextState(initialConditions_);
		initialConditions_ = inputListNext;
		currentTime = rungeKutta_.getCurrentTime();
	}
	function_->copterPosFromMat(initialConditions_);

	mat endState = initialConditions_;
	initialConditions_ = rungeKutta_.computeNextState(initialConditions_);
	for (int i = 0; i < NUM_OF_SPACE_DIM; i++) {
		copter_.fuselage.acceleration[i] = (initialConditions_(2, i) -
			endState(2, i)) / dt_;
		copter_.fuselage.angularAcceleration[i] = (initialConditions_(3, i) -
			endState(3, i)) / dt_;
	}

	auto result = compileOutputVector();
	return result;
}

//                 |Position           |
//                 |Quaternion         |
//                 |Velocity           |
// vector Output = |AngularVelocity    |
//                 |Acceleration       |
//                 |AngularAcceleration|
//                 |...                |
//                 |Engine parameters  |
//
// engine parameters are currentPWM, currentPower, Quat.W, Quat.Z, angVel.Z
std::vector<std::vector<double>> Emulator::compileOutputVector() const
{
	const int numOfEngines = copter_.getNumOfEngines();
	const int numOfVectors = getSizeOfOutParams() + numOfEngines;
	std::vector<std::vector<double>> result;
	result.resize(numOfVectors);
	for (int i = 0; i < getSizeOfOutParams(); i++) {
		if (i != 1) {
			result[i].resize(NUM_OF_SPACE_DIM);
		}
		else {
			result[i].resize(QUAT_LENGTH);
		}
	}
	for (int i = 0; i < NUM_OF_SPACE_DIM; i++) {
		result[0][i] = copter_.fuselage.position(i);
		result[2][i] = copter_.fuselage.velocity(i);
		result[3][i] = copter_.fuselage.angularVelocity(i);
		result[4][i] = copter_.fuselage.acceleration(i);
		result[5][i] = copter_.fuselage.angularAcceleration(i);
	}
	for (int i = 0; i < QUAT_LENGTH; i++) {
		result[1][i] = copter_.fuselage.rotationQuat.at(i);
	}
	for (int i = getSizeOfOutParams(); i < numOfVectors; i++) {
		result[i].resize(getSizeOfEngineParams());
		auto& engine = copter_.engines[i - getSizeOfOutParams()];
		result[i][0] = static_cast<double>(engine.getCurrentPwm());
		result[i][1] = engine.getCurrentPower();
		result[i][2] = engine.rotationQuat.at(0);
		result[i][3] = engine.rotationQuat.at(3);
		result[i][4] = engine.angularVelocity(2);
		if (engine.bladeDir == "clockwise") {
			result[i][4] *= -1;
		}
	}
	return result;
}

int Emulator::getSizeOfOutParams() const
{
	return 6;
}

int Emulator::getSizeOfEngineParams() const
{
	return 5;
}

Emulator::~Emulator()
{
	delete function_;
}
