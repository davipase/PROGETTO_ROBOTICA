#include <iostream>
#include "parameters.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
using namespace std_msgs;
using namespace Eigen;
using namespace std;


//direct kinematic: given joints configuration, find position of ee
eepos direct_kinematic(Matrix<double,6,1> Th){
    Matrix<double,4,4> T10m,T21m,T32m,T43m,T54m,T65m,T06;
    eepos p;
    T10m = T10f(Th(0,0));
    T21m = T21f(Th(1,0));
    T32m = T32f(Th(2,0));
    T43m = T43f(Th(3,0));
    T54m = T54f(Th(4,0));
    T65m = T65f(Th(5,0));
    T06 = T10m*T21m*T32m*T43m*T54m*T65m;
    p.ee_pos = T06.block<3,1>(0,3);
    p.ee_rotm = T06.block<3,3>(0,0);
    return p;
}