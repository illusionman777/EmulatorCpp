#pragma once
#include "Quaternion.h"

using namespace arma;


class PhysicalObject
{
public:
	PhysicalObject();
	double mass;
	mat inertiaMoment;
	vec position;
	Quaternion rotationQuat;
	vec velocity;
	Quaternion rotationQuatDiff;
	vec angularVelocity;
	vec acceleration;
	vec angularAcceleration;
};
