#pragma once

#ifndef EULER_H
#define EULER_H

#include "Helper/Vector3.h"
#include <vector>
class Euler
{
public:
	static std::vector<std::pair<Vector3<double>, Vector3<double>>> SemiImplicitCone(const int steps, const float stepIncrements, const Vector3<double> initVelocity, const Vector3<double> gravity);
	static void OutputResults(const int steps, const float stepIncrements, const std::vector<std::pair<Vector3<double>, Vector3<double>>> positionVelocityPair);
};

#endif // !EULER_H
