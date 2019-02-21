#pragma once
#include "CopterDynamicProperties.h"


class InternalForces
{
public:
	InternalForces(Copter& copter, CopterDynamicProperties& dynamicProperties);
	void compute();
	vec getGyroscopicMoment() const;
	std::vector<vec> getEnginesGyroMoments() const;
	std::vector<double> getEnginesDriveMoments() const;
private:
	Copter& copter_;
	CopterDynamicProperties& dynamicProperties_;
	vec gyroscopicMoment_;
	std::vector<vec> enginesGyroMoments_;
	std::vector<double> enginesDriveMoments_;
};
