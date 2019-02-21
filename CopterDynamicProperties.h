#pragma once
#include "Copter.h"


class CopterDynamicProperties
{
public:
	explicit CopterDynamicProperties(Copter& copter);

	mat getFusRotMatrix() const;
	mat getFusInertiaMomentCur() const;
	mat getCopterInertiaMomentCur() const;
	std::vector<mat> getEnginesRotMatrices() const;
	std::vector<mat> getEnginesRotMatricesT() const;
	std::vector<mat> getEngInertiaMomentsCur() const;
	std::vector<mat> getEngInertiaMomentsCurRelative() const;

	void computeCurProperties();
private:
	const Copter& copter_;

	mat fusRotMatrix_;
	mat fusInertiaMomentCur_;
	mat copterInertiaMomentCur_;
	std::vector<mat> enginesRotMatrices_;
	std::vector<mat> enginesRotMatricesT_;
	std::vector<mat> engInertiaMomentsCur_;
	std::vector<mat> engInertiaMomentsCurRelative_;
};

