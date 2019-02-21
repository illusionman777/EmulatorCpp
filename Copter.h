#pragma once

#include <vector>
#include "Engine.h"


class Copter
{
public:
	Copter();
	explicit Copter(int numOfEngines);
	PhysicalObject fuselage;
	double aeroSquare;
	vec dragCoef;
	vec momentDragCoef;
	std::vector<Engine> engines;
	std::vector<Quaternion> fusEngineQuaternions;
	double getMass();

	int getNumOfEngines() const;
	void setNumOfEngines(int value);

	std::vector<double> getDistanceFusEngineMc() const;
	double getDistanceFusEngineMc(int engineNo) const;
	void setDistanceFusEngineMc(double distance, int engineNo);
	void setDistanceFusEngineMc(const std::vector<double>& value);

	std::vector<double> getHeightFusEngineMc() const;
	double getHeightFusEngineMc(int engineNo) const;
	void setHeightFusEngineMc(double height, int engineNo);
	void setHeightFusEngineMc(const std::vector<double>& value);

	std::vector<double> getAngleFusEngine() const;
	double getAngleFusEngine(int engineNo) const;
	void setAngleFusEngine(double angle, int engineNo);
	void setAngleFusEngine(const std::vector<double>& value);

	std::vector<vec> getVectorFusEngineMc() const;
	vec getVectorFusEngineMc(int engineNo) const;
private:
	int numOfEngines_;
	std::vector<double> distanceFusEngineMc_;
	std::vector<double> heightFusEngineMc_;
	std::vector<double> angleFusEngine_;
	std::vector<vec> vectorFusEngineMc_;

	void defineEngines();
	void setVectorXyPlane();
	void setVectorXyPlane(int engineNo);
};
