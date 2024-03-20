#pragma once
#include "Helper/Vector3.h"
#include <vector>

class GeneralMotion
{
public:
	static std::vector<Vector3<double>> SolveForCone(const int steps, const float stepIncrements, const Vector3<double> initPosition, const std::vector<Vector3<double>> rk4, const std::vector<Vector3<double>> sieVelocity, const std::vector<Vector3<double>> siePosition);
	static void OutputResults(const int steps, const float stepIncrements, const std::vector<Vector3<double>> position);
private:

	struct RotationMatrix {
		double mat[3][3];
	};
	static RotationMatrix SolveRotationMatrix(const double alpha, const double beta, const double gamma, const double theta);
	static Vector3<double> MultiplyRotationMatrix(const Vector3<double> position, const RotationMatrix rotation);
	static Vector3<double> DeterminantMatrix(const RotationMatrix matrix);
	static void TestFunction();
};

