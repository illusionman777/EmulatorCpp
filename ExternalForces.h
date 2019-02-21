#pragma once
#include "CopterDynamicProperties.h"

class ExternalForces
{
public:
	ExternalForces(Copter& copter, CopterDynamicProperties& dynamicProperties);
	vec getMainForce() const;
	vec getMainMoment() const;
	std::vector<vec> getEnginesAeroForces() const;
	std::vector<double> getEnginesAeroMoments() const;
	void compute();
	double getG();
	double getRho0();
	double getAirDensity(double altitude);
private:
	Copter& copter_;
	CopterDynamicProperties& dynamicProperties_;
	double airDensity_;
	vec mainForce_;
	vec mainMoment_;
	std::vector<vec> enginesAeroForces_;
	std::vector<double> enginesAeroMoments_;
	void computeForces();
	void computeMoments();
};
