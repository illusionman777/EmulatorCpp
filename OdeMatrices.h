#pragma once
#include "CopterDynamicProperties.h"
#include "ExternalForces.h"
#include "InternalForces.h"

class OdeMatrices
{
public:
	explicit OdeMatrices(Copter& copter,
	                     CopterDynamicProperties& dynamicProperties);
	mat getCoefMatrix() const;
	vec getConstantTermsVector() const;
	void compute(const ExternalForces& exForces, const InternalForces& inForces);
private:
	Copter& copter_;
	CopterDynamicProperties& dynamicProperties_;
	mat coefMatrix_;
	vec constantTermsVector_;
	int matrixDimension_;
	void computeMatrix();
	void computeVector(const ExternalForces& exForces,
	                   const InternalForces& inForces);
};
