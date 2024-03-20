#include "GeneralMotion.h"
#include <iostream>
#include <fstream>


std::vector<Vector3<double>> GeneralMotion::SolveForCone(const int steps, const float stepIncrements, const Vector3<double> initPosition, const std::vector<Vector3<double>> rk4, const std::vector<Vector3<double>> sieVelocity, const std::vector<Vector3<double>> siePosition)
{
	const int stepsToRun = steps / stepIncrements;

	std::vector<Vector3<double>> position;
	position.push_back(initPosition);
	Vector3<double> pos = initPosition;
	float theta = 0.0f;

	//R = Rcm, 0 + TVcm + AR'0
	// R is the new position vector OP'
	// Rcm, 0 = Position of center of mass, relative to world, is the position of the center of mass, OG, 
	// TVcm = t time, Vcm velocity of center of mass, relative to itself, it is the displacement of the center of mass over time GG'
	// AR'0 = A = rotation matrix multiplied by |angular velocity|, R'0 = initial value of r', new position vector relative to G'  G'P'
	//R' = AR'0
	for (int i = 0; i < stepsToRun-1; i++)
	{
		Vector3<double> angularVelocity = rk4[i];
		double avMag = angularVelocity.Magnitude();
		theta = avMag * stepIncrements;
		Vector3<double> avNorm = angularVelocity.Normalize();

		RotationMatrix rotation = SolveRotationMatrix(avNorm.x, avNorm.y, avNorm.z, theta);
		pos = MultiplyRotationMatrix(pos, rotation);
		
		Vector3<double> newPosition = siePosition[i] + pos;
		position.push_back(newPosition);
	}

	return position;
}

void GeneralMotion::OutputResults(const int steps, const float stepIncrements, const std::vector<Vector3<double>> position)
{
	const int stepsToRun = steps / stepIncrements;
	//File output
	std::ofstream file;
	file.open("GeneralMotion.csv");
	file << "time,x,y,z" << "\n";

	for (int i = 0; i < stepsToRun; i++)
	{
		file << stepIncrements * i << "," << position[i].x << "," << position[i].y << "," << position[i].z << "," << "\n";
	}
	file.close();
}

GeneralMotion::RotationMatrix GeneralMotion::SolveRotationMatrix(const double alpha, const double beta, const double gamma, const double theta)
{
	//Rotation by theta about an axis alpha,beta,gamma(unit vector) going through the center of mass
	//Using the generalmotion of rigidbodies from page 87 of the notes
	RotationMatrix rotationMatrix;

	double ccos = cos(theta);
	double csin = sin(theta);
		//First row
	rotationMatrix.mat[0][0] = alpha * alpha * (1 - ccos) + ccos;
	rotationMatrix.mat[0][1] = alpha * beta * (1 - ccos) - (gamma * csin);
	rotationMatrix.mat[0][2] = alpha * gamma * (1 - ccos) + (beta * csin);

	//Second row
	rotationMatrix.mat[1][0] = alpha * beta * (1 - ccos) + (gamma * csin);
	rotationMatrix.mat[1][1] = beta*beta * (1 - ccos) + ccos;
	rotationMatrix.mat[1][2] = beta * gamma * (1 - ccos) - (alpha * csin);

	//Third row
	rotationMatrix.mat[2][0] = alpha * gamma * (1 - ccos) - (beta * csin);
	rotationMatrix.mat[2][1] = beta * gamma * (1 - ccos) + (alpha * csin);
	rotationMatrix.mat[2][2] = gamma*gamma * (1 - ccos) + ccos;
	return rotationMatrix;
}

Vector3<double> GeneralMotion::MultiplyRotationMatrix(const Vector3<double> position, const RotationMatrix rotation)
{
	Vector3<double> newPosition;
	newPosition.x = (rotation.mat[0][0] * position.x) + (rotation.mat[0][1] * position.y) + (rotation.mat[0][2] * position.z);
	newPosition.y = (rotation.mat[1][0] * position.x) + (rotation.mat[1][1] * position.y) + (rotation.mat[1][2] * position.z);
	newPosition.z = (rotation.mat[2][0] * position.x) + (rotation.mat[2][1] * position.y) + (rotation.mat[2][2] * position.z);
	return newPosition;
}


void GeneralMotion::TestFunction()
{

	////////USING example 9.1.2 in notes page 90
	////Solve matrix with normalised angular velocty xyz, and magnitude angular velocity * time
	RotationMatrix rotation2 = SolveRotationMatrix(2.0f / 3.0f, -2.0f / 3.0f, 1.0f / 3.0f, (3*3.1415f) / 2.0f);
	//multiply rotaiton matrix with position vector t0
	Vector3<double> GDashPDash2 = MultiplyRotationMatrix(Vector3<double>(4,1,0), rotation2);
	//Figure out the velocity of the body
	//V = Vcm + W + /\R'0
	//  = Ucm + AcmT + W + /\R'0
	//Ucm = last velocity step
	//AcmT = F*T/m = gravity * step
	// /\R'0 = r'
	// W * /\R'0 = |ijk, WxWyWz, r'xr'yr'z|
	Vector3<double> mid = (Vector3<double>(-1, 4, 2) * (3.1415f / 5));
	Vector3<double> startandMid = Vector3<double>(0, 3, 1) + mid;
	RotationMatrix ma;
	ma.mat[1][0] = 1;
	ma.mat[1][1] = -1;
	ma.mat[1][2] = 1.0/2.0;
	ma.mat[2][0] = 15.0/9.0;
	ma.mat[2][1] = -24.0/9.0;
	ma.mat[2][2] = -24.0/9.0;
	Vector3<double> detMat = DeterminantMatrix(ma);
	Vector3<double> newVel2 = startandMid + detMat;

	//Figure out the position of the body with 
	//R = Rcm + r'
	//Rcm = UT + 1/2AT^2
	//  = UcmT + 1/2AcmT^2 + r'
	Vector3<double> rcm1;
	Vector3<double> rcm2;
	Vector3<double> rcm;
	Vector3<double> r;
	rcm1 = Vector3<double>(0,3,1) * 3.1415f;
	rcm2 = Vector3<double>(-1, 4, 2) * ((1.0f / 2.0f) * (3.1415f * 3.1415f) * (1.0f / 5.0f));
	rcm = rcm1 + rcm2;
	r = rcm + GDashPDash2;
}
Vector3<double> GeneralMotion::DeterminantMatrix(RotationMatrix matrix)
{
	Vector3<double> det;
	det.x = (matrix.mat[1][1] * matrix.mat[2][2]) - (matrix.mat[2][1] * matrix.mat[1][2]);
	det.y = -((matrix.mat[1][0] * matrix.mat[2][2]) - (matrix.mat[2][0] * matrix.mat[1][2]));
	det.z = (matrix.mat[1][0] * matrix.mat[2][1]) - (matrix.mat[2][0] * matrix.mat[1][1]);
	return det;

}
