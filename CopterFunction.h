#pragma once
#include "PhysicalFunction.h"
#include "Copter.h"

class CopterFunction :
	public PhysicalFunction
{
public:
	CopterFunction(Copter& copter);
	mat compute(double timeCur, const mat& stateCur) const override;	
	void copterPosFromMat(const mat& stateCur) const;
private:
	Copter& copter_;
	mat compileOutputMat() const;
};

