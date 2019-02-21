#pragma once
#include <string>
#include "PhysicalObject.h"


class Engine :
	public PhysicalObject
{
public:
	Engine();
	double maxPower;
	double maxDriveMoment;
	int maxPwm;
	double bladeDiameter;
	double bladeCoefAlpha;
	double bladeCoefBeta;
	std::string rotationDir;
	std::string bladeDir;
	int getCurrentPwm() const;
	void setCurrentPwm(int value);
	double getCurrentPower() const;
private:
	int currentPwm_;
	double currentPower_;
};
