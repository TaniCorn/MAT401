#include "PythonExample.h"
#include <iostream>
#include <string>
void PythonExample::PythonToCPPExample()
{
	std::cout << "Hello World!\n";
	double y0 = 0.0;
	double x0 = 0.0;
	double xe = 20.0;
	double dx = 10.0;

	std::vector<Vector2<double>> EulerPlot;
	EulerPlot = Solve(x0, y0, xe, dx);
	std::vector<Vector2<double>> RK2Plot;
	RK2Plot = SolveForRK2(x0, y0, xe, dx);
	std::vector<Vector2<double>> RK4Plot;
	RK4Plot = SolveForRK4(x0, y0, xe, dx);

	const double step = 20;
	dx = 10;
	std::vector<double> dxHistory;
	TimeMeasure time;
	std::vector<Vector2<double>> eulerSteps;
	std::vector<Vector2<double>> rk2Steps;

	std::vector<double> timeEuler;
	std::vector<double> timeRK2;
	std::vector<double> timeRK4;

	std::vector<double> errorEuler;
	std::vector<double> errorRK2;
	std::vector<double> errorRK4;

	std::vector<double> x;
	for (int i = 0; i < RK4Plot.size(); i++)
	{
		x.push_back(RK4Plot[i].x);
	}

	yc = ExactFunction(x);
	for (int i = 0; i < step; i++)
	{
		dxHistory.push_back(dx);
		CaptureEuler(time, eulerSteps, errorEuler, timeEuler, x0, y0, xe, dx);
		CaptureRK2(time, eulerSteps, errorEuler, timeEuler, x0, y0, xe, dx);
		CaptureRK4(time, eulerSteps, errorEuler, timeEuler, x0, y0, xe, dx);
		dx = dx / 2.0;
	}

	yc = ExactFunction(x);
}

void PythonExample::CaptureEuler(TimeMeasure& time, std::vector<Vector2<double>>& eulerSteps, std::vector<double>& errorEuler, std::vector<double>& timeEuler, double x0, double y0, double xe, double dx)
{
    time.CaptureStart();
    eulerSteps = Solve(x0, y0, xe, dx);
    time.CaptureEnd();
    double tm = time.GetTimeInNanoseconds();
    std::cout << "Time for Euler dx " + std::to_string(dx) + " is " + std::to_string(tm) << std::endl;
    timeEuler.push_back(tm);
    errorEuler.push_back(abs(eulerSteps.back().y - yc.back() / yc.back() * 100.0));
}

void PythonExample::CaptureRK2(TimeMeasure& time, std::vector<Vector2<double>>& rk2Steps, std::vector<double>& errorRK2, std::vector<double>& timeRK2, double x0, double y0, double xe, double dx)
{
	time.CaptureStart();
	rk2Steps = SolveForRK2(x0, y0, xe, dx);
	time.CaptureEnd();
	double tm = time.GetTimeInNanoseconds();
	std::cout << "Time for RK2 dx " + std::to_string(dx) + " is " + std::to_string(tm) << std::endl;
	timeRK2.push_back(tm);
	errorRK2.push_back(abs(rk2Steps.back().y - yc.back() / yc.back() * 100.0));
}

void PythonExample::CaptureRK4(TimeMeasure& time, std::vector<Vector2<double>>& rk4Steps, std::vector<double>& errorRK4, std::vector<double>& timeRK4, double x0, double y0, double xe, double dx)
{
	time.CaptureStart();
	rk4Steps = SolveForRK4(x0, y0, xe, dx);
	time.CaptureEnd();
	double tm = time.GetTimeInNanoseconds();
	std::cout << "Time for RK4 dx " + std::to_string(dx) + " is " + std::to_string(tm) << std::endl;
	timeRK4.push_back(tm);
	errorRK4.push_back(abs(rk4Steps.back().y - yc.back() / yc.back() * 100.0));
}

std::vector<Vector2<double>> PythonExample::Solve(double xStart, double yStart, double xEnd, double xDelta)
{
	//Fill x with values from xStart to xEnd by stepping with xDelta
	std::vector<double> x;
	for (int i = 0; i < xEnd; i++)
	{
		x.push_back(xStart + xDelta * i);
	}
	//Y solution of xStep
	std::vector<double> y;
	y.push_back(yStart);
	for (int i = 1; i < x.size(); i++)
	{
		y.push_back(y[i - 1] + xDelta * DifferentialBy1Function(x[i - 1]));
	}

	//Collate results into the return format
	std::vector<Vector2<double>> Result;
	for (int i = 0; i < y.size(); i++)
	{
		Result.push_back(Vector2<double>(x[i], y[i]));
	}
	return Result;
}

std::vector<Vector2<double>> PythonExample::SolveForRK2(double xStart, double yStart, double xEnd, double xDelta)
{
	//Fill x with values from xStart to xEnd by stepping with xDelta
	std::vector<double> x;
	for (int i = 0; i < xEnd; i++)
	{
		x.push_back(xStart + xDelta * i);
	}
	//Y solution of xStep
	std::vector<double> y;
	y.push_back(yStart);
	for (int i = 1; i < x.size(); i++)
	{
		/*		double k1 = xDelta * DifferentialBy1Function(x[i - 1], y[i - 1]);
				double k2 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta / 2.0, y[i - 1] + k1 / 2.0);*/
		double k1 = xDelta * DifferentialBy1Function(x[i - 1]);
		double k2 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta / 2.0);
		y.push_back(y[i - 1] + k2);
	}

	//Collate results into the return format
	std::vector<Vector2<double>> Result;
	for (int i = 0; i < y.size(); i++)
	{
		Result.push_back(Vector2<double>(x[i], y[i]));
	}
	return Result;
}

std::vector<Vector2<double>> PythonExample::SolveForRK4(double xStart, double yStart, double xEnd, double xDelta)
{
	//Fill x with values from xStart to xEnd by stepping with xDelta
	std::vector<double> x;
	for (int i = 0; i < xEnd; i++)
	{
		x.push_back(xStart + xDelta * i);
	}
	//Y solution of xStep
	std::vector<double> y;
	y.push_back(yStart);
	for (int i = 1; i < x.size(); i++)
	{
		//double k1 = xDelta * DifferentialBy1Function(x[i - 1], y[i - 1]);
		//double k2 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta / 2.0, y[i - 1] + k1 / 2.0);
		//double k3 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta / 2.0, y[i - 1] + k2 / 2.0);
		//double k4 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta, y[i - 1] + k3);
		double k1 = xDelta * DifferentialBy1Function(x[i - 1]);
		double k2 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta / 2.0);
		double k3 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta / 2.0);
		double k4 = xDelta * DifferentialBy1Function(x[i - 1] + xDelta);
		y.push_back(y[i - 1] + 1.0 / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4));
	}

	//Collate results into the return format
	std::vector<Vector2<double>> Result;
	for (int i = 0; i < y.size(); i++)
	{
		Result.push_back(Vector2<double>(x[i], y[i]));
	}
	return Result;
}
