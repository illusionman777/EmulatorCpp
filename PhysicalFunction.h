#pragma once
#include <armadillo>

using namespace arma;

class PhysicalFunction
{
public:
	PhysicalFunction();
	virtual mat compute(double timeCur, const mat& inputValues) const =0;
	virtual ~PhysicalFunction();
};

