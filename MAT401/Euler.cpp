#include "Euler.h"
#include <iostream>
#include <fstream>

std::vector<std::pair<Vector3<double>, Vector3<double>>> Euler::SemiImplicitCone(const int steps, const float stepIncrements, const Vector3<double> initVelocity, const Vector3<double> gravity)
{
	//Calculating Velocity and Position trajectory
	const int stepsToRun = steps / stepIncrements;

	//Init velocity at position pair vectors
	std::vector < std::pair<Vector3<double>, Vector3<double>>> velocityPositionPair;
	Vector3<double> velocity = initVelocity;
	Vector3<double> position = Vector3<double>(0,0,0);
	std::pair<Vector3<double>, Vector3<double>> currentPair;
	currentPair.first = velocity;
	currentPair.second = position;
	velocityPositionPair.push_back(currentPair);

	//Vn+1 = Vn - GT
	//Xn+1 = Xn + Vn+1T
	for (int i = 0; i < stepsToRun; i++)
	{
		//Page 95 of the notes, 10.1.1
		velocity = velocity + (gravity * stepIncrements);
		position = position + (velocity * stepIncrements); 
		currentPair.first = velocity;
		currentPair.second = position;
		velocityPositionPair.push_back(currentPair);
	}

	return velocityPositionPair;
}

void Euler::OutputResults(const int steps, const float stepIncrements, const std::vector<std::pair<Vector3<double>, Vector3<double>>> velocityPositionPair)
{
	const int stepsToRun = steps / stepIncrements;
	//File output
	std::ofstream file;
	file.open("SemiEulerCone.csv");
	file << "Vx" << "," << "Vy" << "," << "Vz" << "," << "Dx" << "Dy" << "Dz" << "\n";

	for (int i = 0; i < stepsToRun; i++)
	{
		file << stepIncrements * i << "," << velocityPositionPair[i].first.x << "," << velocityPositionPair[i].first.y << "," << velocityPositionPair[i].first.z << "," << stepIncrements * i << "," << velocityPositionPair[i].second.x << "," << velocityPositionPair[i].second.y << "," << velocityPositionPair[i].second.z << "\n";
	}
	file.close();
}

