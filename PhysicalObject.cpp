#include "pch.h"
#include "PhysicalObject.h"
#include "Support.h"


PhysicalObject::PhysicalObject()
{
	mass = 0;
	inertiaMoment = mat(NUM_OF_SPACE_DIM, NUM_OF_SPACE_DIM);
	inertiaMoment << 0.0 << 0.0 << 0.0 << endr
		          << 0.0 << 0.0 << 0.0 << endr
	              << 0.0 << 0.0 << 0.0 << endr;
	position = vec(NUM_OF_SPACE_DIM);
	position << 0.0 << 0.0 << 0.0;
	rotationQuat = Quaternion();
	velocity = vec(NUM_OF_SPACE_DIM);
	velocity << 0.0 << 0.0 << 0.0;
	rotationQuatDiff = Quaternion(0.0, 0.0, 0.0, 0.0);
	angularVelocity = vec(NUM_OF_SPACE_DIM);
	angularVelocity << 0.0 << 0.0 << 0.0;
	acceleration = vec(NUM_OF_SPACE_DIM);
	acceleration << 0.0 << 0.0 << 0.0;
	angularAcceleration = vec(NUM_OF_SPACE_DIM);
	angularAcceleration << 0.0 << 0.0 << 0.0;
}
