#include "pch.h"
#include "Engine.h"


Engine::Engine()
{
	maxPower = 1.0;
	maxDriveMoment = 0.0;
	maxPwm = 1;
	bladeDiameter = 0.0;
	bladeCoefAlpha = 0.8;
	bladeCoefBeta = 1.0;
	rotationDir = "counterclockwise";
	bladeDir = "counterclockwise";
	currentPwm_ = 0;
	currentPower_ = 0.0;
}

int Engine::getCurrentPwm() const
{
	return currentPwm_;
}

void Engine::setCurrentPwm(const int value)
{
	currentPwm_ = value;
	currentPower_ = currentPwm_ * currentPwm_ * maxPower /
		(maxPwm * maxPwm);
}

double Engine::getCurrentPower() const
{
	return currentPower_;
}
