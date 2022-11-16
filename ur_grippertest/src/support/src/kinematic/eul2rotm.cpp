#include <iostream>
#include "std_msgs/Float64.h"
#include "parameters.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h> 
using namespace Eigen;
using namespace std_msgs;

//USING CONVENTION ZYX
//convertion euler angles -> rotation matrix
Matrix<double,3,3> eul2rotm(Matrix<double,3,1> eul){
    double cr,sr,cp,sp,cy,sy;
    cr=cos(eul(0,0)); cp=cos(eul(1,0)); cy=cos(eul(2,0));
    sr=sin(eul(0,0)); sp=sin(eul(1,0)); sy=sin(eul(2,0));

    Matrix<double,3,3> rotm;

    rotm(0,0) = cy*cp;
    rotm(0,1) = cy*sp*sr - sy*cr;
    rotm(0,2) = cy*sp*cr + sy*sr;

    rotm(1,0) = sy*cp;
    rotm(1,1) = cy*cr + sp*sy*sr;
    rotm(1,2) = sp*sy*cr - sr*cy;  

    rotm(2,0) = -sp;
    rotm(2,1) = cp*sr;
    rotm(2,2) = cp*cr;

    return rotm;
}