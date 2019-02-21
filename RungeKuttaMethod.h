#pragma once
#include "PhysicalFunction.h"


class RungeKuttaMethod
{
public:
	RungeKuttaMethod();
	RungeKuttaMethod(PhysicalFunction* function, double dt);
	mat computeNextState(const mat& stateCurrent);
	double getCurrentTime() const;
private:
	PhysicalFunction* function_;
	double dt_;
	double timeCur_;
};

