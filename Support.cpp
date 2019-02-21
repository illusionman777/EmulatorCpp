#include "pch.h"
#include <cmath>
#include "Support.h"


// Number of space dimensions (3d)
const int NUM_OF_SPACE_DIM = 3;
// Number of physical parameters for one physical object at ODE
const int NUM_OF_PHYS_PARAMS = 4;
// Quaternion length
const int QUAT_LENGTH = 4;
bool isZero(const double value)
{
	const double EPS = 1e-12;
	return abs(value) < EPS;
}
