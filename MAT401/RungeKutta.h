#pragma once
#ifndef RUNGEKUTTA_H
#define RUNGEKUTTA_H

#include "Helper/Vector3.h"
#include "Cone.h"
#include <vector>
class RungeKutta
{
public:
	RungeKutta();
	static std::vector<Vector3<double>> SolveForRK4Cone(const int steps, const float stepIncrements, const Cone cone, const Vector3<double> initForce);
	static void OutputResults(const int steps, const float stepIncrements, const std::vector<Vector3<double>> angularVelocity);
private:
	static Vector3<double> RK4Cone(const double delta, const Vector3<float> gamma, const Vector3<double> vel);
};

#endif // !RUNGEKUTTA_H
