#include "RungeKutta.h"
#include <math.h>
#include <iostream>
#include <fstream>

RungeKutta::RungeKutta()
{
}

std::vector<Vector3<double>> RungeKutta::SolveForRK4Cone(const int steps, const float stepIncrements, const Cone cone, const Vector3<double> initForce)
{
	const int stepsToRun = steps / stepIncrements;
	const float radius = cone.radius;
	const float height = cone.height;
	const float mass = cone.mass;

	Vector3<double> v3;
	std::vector<Vector3<double>> vel;
	vel.push_back(initForce);


	//Inertia tensor from assessment page for our cone is
	// I = (3/20) * Mass * diag[r^2 +(1/4)h^2, r^2 +(1/4)h^2, 2r^2]
	//Principle moments of intertia, I1,I2,I3
	float outsideI = (3.0f / 20.0f) * mass;
	float I1 = outsideI * (pow(radius, 2) + (pow(height, 2)) / 4.0f);
	float I2 = outsideI * (pow(radius, 2) + (pow(height, 2)) / 4.0f);
	float I3 = outsideI * (2.0f * pow(radius, 2));

	//Gamma values from assessment page
	Vector3<float> gamma;
	gamma.x = (I3 - I2) / I1;
	gamma.y = (I1 - I3) / I2;
	gamma.z = (I2 - I1) / I3;

	//Eulers equation becomes 
	//wx = - gamma1 * av.y * av.z
	//wy = - gamma2 * av.x * av.z
	//wz = - gamma3 * av.x * av.y
	//Stepping through aka differentiating will give us our equations for the loop
	for (int i = 0; i < stepsToRun; i++)
	{
		Vector3<double> v = vel[i];
		
		vel.push_back(RK4Cone(stepIncrements, gamma, v));
	}

	return vel;
}

void RungeKutta::OutputResults(const int steps, const float stepIncrements, const std::vector<Vector3<double>> angularVelocity)
{
	const int stepsToRun = steps / stepIncrements;
	//Output file
	std::ofstream file;
	file.open("RK4Cone.csv");
	for (int i = 0; i < stepsToRun; i++)
	{
		file <<stepIncrements * i << "," << angularVelocity[i].x << "," << angularVelocity[i].y << "," << angularVelocity[i].z << "\n";
	}
	file.close();
}

Vector3<double> RungeKutta::RK4Cone(const double delta, const Vector3<float> gamma, const Vector3<double> v)
{
	//Where v is the angular velocity
	//Calculate k1 values
	double kx1 = -delta * gamma.x * v.y * v.z;
	double ky1 = -delta * gamma.y * v.x * v.z;
	double kz1 = -delta * gamma.z * v.x * v.y;

	//We're multiplying instead of dividing, like the example does, because it's computationally faster
	//Calculate k2 values - Page 97 of MAT401 lecture notes
	double kx2 = -delta * gamma.x * (v.y + (ky1 * 0.5)) * (v.z + (kz1 * 0.5));
	double ky2 = -delta * gamma.y * (v.x + (kx1 * 0.5)) * (v.z + (kz1 * 0.5));
	double kz2 = -delta * gamma.z * (v.x + (kx1 * 0.5)) * (v.y + (ky1 * 0.5));

	//Calculate k3 values
	double kx3 = -delta * gamma.x * (v.y + (ky2 * 0.5)) * (v.z + (kz2 * 0.5));
	double ky3 = -delta * gamma.y * (v.x + (kx2 * 0.5)) * (v.z + (kz2 * 0.5));
	double kz3 = -delta * gamma.z * (v.x + (kx2 * 0.5)) * (v.y + (ky2 * 0.5));

	//Calculate k4 values
	double kx4 = -delta * gamma.x * (v.y + ky3) * (v.z + kz3);
	double ky4 = -delta * gamma.y * (v.x + kx3) * (v.z + kz3);
	double kz4 = -delta * gamma.z * (v.x + kx3) * (v.y + ky3);

	//Calculate final k values
	double kx = v.x + ((kx1 + (2.0f * kx2) + (2.0f * kx3) + kx4) / 6.0f);
	double ky = v.y + ((ky1 + (2.0f * ky2) + (2.0f * ky3) + ky4) / 6.0f);
	double kz = v.z + ((kz1 + (2.0f * kz2) + (2.0f * kz3) + kz4) / 6.0f);
	return Vector3<double>(kx, ky, kz);
}
