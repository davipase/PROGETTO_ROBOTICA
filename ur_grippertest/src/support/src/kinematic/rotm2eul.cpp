#include <iostream>
#include "std_msgs/Float64.h"
#include "parameters.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h> 
using namespace Eigen;
using namespace std_msgs;

//USING CONVENTION ZYX
//conversion rotation matrix -> euler angles
Matrix<double,3,1> rotm2eul(Matrix<double,3,3> rotm){

    Matrix<double,3,1> eul;
    
    eul(0,0) = atan2(rotm(1,0), rotm(0,0)); // theta_z
    eul(1,0) = asin(-rotm(2,0));             // theta_y
    eul(2,0) = atan2(rotm(2,1), rotm(2,2)); // theta_x
    return eul;
}