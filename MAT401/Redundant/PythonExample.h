#pragma once
#include "Helper/TimeMeasure.h"
#include "Helper/VectorPositions.h"
#include <vector>

class PythonExample
{
public:
	void PythonToCPPExample();
private:
	static std::vector<double> yc;

	void CaptureEuler(TimeMeasure& time, std::vector<Vector2<double>>& eulerSteps, std::vector<double>& errorEuler, std::vector<double>& timeEuler, double x0, double y0, double xe, double dx);
	void CaptureRK2(TimeMeasure& time, std::vector<Vector2<double>>& rk2Steps, std::vector<double>& errorRK2, std::vector<double>& timeRK2, double x0, double y0, double xe, double dx);
	void CaptureRK4(TimeMeasure& time, std::vector<Vector2<double>>& rk4Steps, std::vector<double>& errorRK4, std::vector<double>& timeRK4, double x0, double y0, double xe, double dx);

	static std::vector<Vector2<double>> Solve(double xStart, double yStart, double xEnd, double xDelta);
	static std::vector<Vector2<double>> SolveForRK2(double xStart, double yStart, double xEnd, double xDelta);
	static std::vector<Vector2<double>> SolveForRK4(double xStart, double yStart, double xEnd, double xDelta);

	static float ExactFunction(double x)
	{
		return pow(x, 5);
	}

	static float DifferentialBy1Function(double x)
	{
		return (5.0 * pow(x, 4));
	}

	static std::vector<double> ExactFunction(std::vector<double> x) {
		std::vector<double> result;
		for (int i = 0; i < x.size(); i++)
		{
			result.push_back(ExactFunction(x[i]));

		}
		return result;
	}

	static std::vector<double> DifferentialBy1Function(std::vector<double> x) {
		std::vector<double> result;
		for (int i = 0; i < x.size(); i++)
		{
			result.push_back(DifferentialBy1Function(x[i]));

		}
		return result;
	}

};

