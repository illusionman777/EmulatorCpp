#include "pch.h"

#include <corecrt_math_defines.h>
#include <iostream>

#include "Quaternion.h"
#include "Emulator.h"


int main()
{
	Copter copter = Copter(4);
	copter.fuselage.mass = 1.0;
	copter.fuselage.inertiaMoment << 0.1 << 0.0 << 0.0 << endr
		                          << 0.0 << 0.1 << 0.0 << endr
	                              << 0.0 << 0.0 << 0.2 << endr;
	copter.aeroSquare = 1.0;
	copter.dragCoef << 0.01 << 0.01 << 0.2;
	copter.momentDragCoef << 0.1 << 0.1 << 0.1;
	copter.engines[1].rotationDir = "clockwise";
	copter.engines[1].bladeDir = "clockwise";
	copter.engines[3].rotationDir = "clockwise";
	copter.engines[3].bladeDir = "clockwise";
	const int numOfEngines = copter.getNumOfEngines();
	for (int i = 0; i < numOfEngines; i++) {
		copter.setDistanceFusEngineMc(0.25, i);
		copter.setHeightFusEngineMc(0.002, i);
		double angle = 2.0 * M_PI / numOfEngines;
		angle *= 1.0 / 2.0 + i;
		copter.setAngleFusEngine(angle, i);
		auto& engine = copter.engines[i];
		engine.mass = 0.1;
		engine.inertiaMoment << 0.01 << 0.0 << 0.0 << endr
		                     << 0.0 << 0.01 << 0.0 << endr
		                     << 0.0 << 0.0 << 0.02 << endr;
		engine.maxPower = 110.0;
		engine.maxDriveMoment = 5.0;
		engine.maxPwm = 1024;
		engine.bladeDiameter = 0.2;
		engine.angularVelocity[2] = 7036.6 / 60.0 / 2.0 / M_PI;
		if (engine.rotationDir == "clockwise") {
			engine.angularVelocity *= -1;
		}
	}
	copter.fuselage.position << 0.0 << 0.0 << 10.0;
	const double dt = 0.001;
	double endTime = 0.0;
	auto emulatorTest = Emulator(copter, dt);
	std::vector<std::vector<double>> result;
	std::vector<int> pwm;
	pwm.resize(numOfEngines);
	for (int i = 0; i < numOfEngines; i++) {
		pwm[i] = 391;
	}
	std::clock_t start, finish;
	start = std::clock();
	for (int i = 0; i < 400; i++) {
		endTime += 0.02;
		result = emulatorTest.computeNextState(pwm, endTime);
	}
	finish = std::clock();
	const double duration = (finish - start) / static_cast<double>(CLOCKS_PER_SEC);

	cout << emulatorTest.getCurrentTime() << " " << endTime << endl;
	cout << "Duration: " << duration << endl;
	cout << "Position:" << endl;
	for (auto coordinate : result[0]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Angular position:" << endl;
	for (auto coordinate : result[1]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Velocity:" << endl;
	for (auto coordinate : result[2]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Angular velocity:" << endl;
	for (auto coordinate : result[3]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Acceleration:" << endl;
	for (auto coordinate : result[4]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Angular acceleration:" << endl;
	for (auto coordinate : result[5]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Engine 0 props:" << endl;
	for (auto coordinate : result[6]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Engine 1 props:" << endl;
	for (auto coordinate : result[7]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Engine 2 props:" << endl;
	for (auto coordinate : result[8]) {
		cout << coordinate << " ";
	}
	cout << endl;
	cout << "Engine 3 props:" << endl;
	for (auto coordinate : result[9]) {
		cout << coordinate << " ";
	}
	cout << endl;

	return 0;
}
