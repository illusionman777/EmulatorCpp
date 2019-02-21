#include "pch.h"
#include "Copter.h"
#include "Support.h"


Copter::Copter()
{
	fuselage = PhysicalObject();
	aeroSquare = 0.0;
	dragCoef = vec(NUM_OF_SPACE_DIM);
	dragCoef << 0.0 << 0.0 << 0.0;
	momentDragCoef = vec(NUM_OF_SPACE_DIM);
	momentDragCoef << 0.0 << 0.0 << 0.0;
	numOfEngines_ = 4;
	defineEngines();
}

Copter::Copter(const int numOfEngines)
{
	fuselage = PhysicalObject();
	aeroSquare = 0.0;
	dragCoef = vec(NUM_OF_SPACE_DIM);
	dragCoef << 0.0 << 0.0 << 0.0;
	momentDragCoef = vec(NUM_OF_SPACE_DIM);
	momentDragCoef << 0.0 << 0.0 << 0.0;
	numOfEngines_ = numOfEngines;
	defineEngines();
}


void Copter::defineEngines()
{
	engines.resize(numOfEngines_);
	vectorFusEngineMc_.resize(numOfEngines_);
	distanceFusEngineMc_.resize(numOfEngines_);
	heightFusEngineMc_.resize(numOfEngines_);
	fusEngineQuaternions.resize(numOfEngines_);
	angleFusEngine_.resize(numOfEngines_);
	for (int i = 0; i < numOfEngines_; i++) {
		engines[i] = Engine();
		distanceFusEngineMc_[i] = 0.0;
		heightFusEngineMc_[i] = 0.0;
		angleFusEngine_[i] = 0.0;
		fusEngineQuaternions[i] = Quaternion();
		vectorFusEngineMc_[i] = vec(NUM_OF_SPACE_DIM);
		vectorFusEngineMc_[i] << 0.0 << 0.0 << 0.0;
	}
}

double Copter::getMass()
{
	double result = fuselage.mass;
	for (const auto& engine : engines) {
		result += engine.mass;
	}
	return result;
}

void Copter::setNumOfEngines(const int value)
{
	numOfEngines_ = value;
	defineEngines();
}

int Copter::getNumOfEngines() const
{
	return numOfEngines_;
}

void Copter::setVectorXyPlane()
{
	for (int i = 0; i < numOfEngines_; i++) {
		vectorFusEngineMc_[i].at(0) = distanceFusEngineMc_[i] *
			cos(angleFusEngine_[i]);
		vectorFusEngineMc_[i].at(1) = distanceFusEngineMc_[i] *
			sin(angleFusEngine_[i]);
	}
}

void Copter::setVectorXyPlane(const int engineNo)
{
	vectorFusEngineMc_[engineNo].at(0) = distanceFusEngineMc_[engineNo] *
		cos(angleFusEngine_[engineNo]);
	vectorFusEngineMc_[engineNo].at(1) = distanceFusEngineMc_[engineNo] *
		sin(angleFusEngine_[engineNo]);
}

void Copter::setDistanceFusEngineMc(const std::vector<double>& value)
{
	distanceFusEngineMc_ = value;
	this->setVectorXyPlane();
}

void Copter::setDistanceFusEngineMc(const double distance,
                                    const int engineNo)
{
	distanceFusEngineMc_[engineNo] = distance;
	this->setVectorXyPlane(engineNo);
}

std::vector<double> Copter::getDistanceFusEngineMc() const
{
	return distanceFusEngineMc_;
}

double Copter::getDistanceFusEngineMc(const int engineNo) const
{
	return distanceFusEngineMc_[engineNo];
}

void Copter::setHeightFusEngineMc(const std::vector<double>& value)
{
	heightFusEngineMc_ = value;
	for (int i = 0; i < numOfEngines_; i++) {
		vectorFusEngineMc_[i].at(2) = heightFusEngineMc_[i];
	}
}

void Copter::setHeightFusEngineMc(const double height, const int engineNo)
{
	heightFusEngineMc_[engineNo] = height;
	vectorFusEngineMc_[engineNo].at(2) = heightFusEngineMc_[engineNo];
}

std::vector<double> Copter::getHeightFusEngineMc() const
{
	return heightFusEngineMc_;
}

double Copter::getHeightFusEngineMc(const int engineNo) const
{
	return heightFusEngineMc_[engineNo];
}

void Copter::setAngleFusEngine(const std::vector<double>& value)
{
	angleFusEngine_ = value;
	this->setVectorXyPlane();
}

void Copter::setAngleFusEngine(const double angle, const int engineNo)
{
	angleFusEngine_[engineNo] = angle;
	this->setVectorXyPlane(engineNo);
}

std::vector<double> Copter::getAngleFusEngine() const
{
	return angleFusEngine_;
}

double Copter::getAngleFusEngine(const int engineNo) const
{
	return angleFusEngine_[engineNo];
}

std::vector<vec> Copter::getVectorFusEngineMc() const
{
	return vectorFusEngineMc_;
}

vec Copter::getVectorFusEngineMc(const int engineNo) const
{
	return vectorFusEngineMc_[engineNo];
}
