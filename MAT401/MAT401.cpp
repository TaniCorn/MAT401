
#include "Euler.h"
#include "RungeKutta.h"
#include "GeneralMotion.h"
#include <iostream>
#include <string>
int main()
{

    //Task 1,2,3,4,5
    //All functions output their own CSV file inside the WorkingDirectory
    //Results with graphs are located in "SolutionDirectory/Results"

    const Cone cone(4,1,10);
    const int steps = 20;
    const float stepSize = 0.01f;

    //Tasks 1 and 2
    const Vector3<double> initForce(3, 1, 2);
    const std::vector<Vector3<double>> angularVelocity = RungeKutta::SolveForRK4Cone(steps ,stepSize, cone, initForce);
    //Judging on page 104 of notes, a similar question is asked and produces similar results, I can deduce this is correct

    //Tasks 3 and 4
    const Vector3<double> initVelocity(0, 0, 200);
    const Vector3<double> gravity(0, 0, -9.8);
    const std::vector<std::pair<Vector3<double>, Vector3<double>>> velocityCMPositionCM = Euler::SemiImplicitCone(steps, stepSize, initVelocity, gravity);
    //Unpairing position and velocity vectors
    std::vector<Vector3<double>> velocityCM, positionCM;
    for (int i = 0; i < velocityCMPositionCM.size(); i++)
    {
        velocityCM.push_back(velocityCMPositionCM[i].first);
        positionCM.push_back(velocityCMPositionCM[i].second);
    }

    //Task 5
    const Vector3<double> initPosition = Vector3<double>(0.0, ((3.0 / 4.0) * cone.radius), 0.0);
    const std::vector<Vector3<double>> conePosition = GeneralMotion::SolveForCone(steps, stepSize, initPosition, angularVelocity,velocityCM, positionCM);

    //Output results
    RungeKutta::OutputResults(steps, stepSize, angularVelocity);
    Euler::OutputResults(steps, stepSize, velocityCMPositionCM);
    GeneralMotion::OutputResults(steps, stepSize, conePosition);


}