#include <iostream>
#include <DeltaKinematics.h>

using namespace std;

DeltaKinematics<double>::DeltaGeometricDim test_robot_dim = {660,90,200,530,70,-5,55}; 
DeltaKinematics<double> test_robot(test_robot_dim);

typedef DeltaKinematics<double>::DeltaVector DV;

int main()
{
    DV my_vector = {0,0,-500,0,0,0};
    my_vector.Print();
    test_robot.CalculateIpk(&my_vector, 1);
    my_vector.Print();
    my_vector.z = 0.0;
    test_robot.CalculateFpk(&my_vector, 1);
    my_vector.Print();
    return 0;
}