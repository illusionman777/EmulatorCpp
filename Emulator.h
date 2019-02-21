#pragma once
#include "RungeKuttaMethod.h"
#include "Copter.h"
#include "CopterFunction.h"


class Emulator
{
public:
	Emulator(Copter& copter, double dt);
	std::vector<std::vector<double>> computeNextState(
		const std::vector<int>& currentPwm, double endTime);
	void initializeStartState();
	double getCurrentTime() const;
	int getSizeOfOutParams() const;
	int getSizeOfEngineParams() const;

	~Emulator();
private:
	Copter& copter_;
	CopterFunction* function_;
	RungeKuttaMethod rungeKutta_;
	double dt_;
	mat initialConditions_;
	bool startStateInitialized_;
	std::vector<std::vector<double>> compileOutputVector() const;
};
