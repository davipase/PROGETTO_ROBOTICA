#ifndef __PARAMETERS__
#define __PARAMETERS__
#include <std_msgs/Float64.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <math.h>
#include <complex>
#define pi 3.14159265
using namespace Eigen;
using namespace std_msgs;


double A[6] = {0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000};
double D[6] = {.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823};

double alpha[6] = { pi/2, 0, 0, pi/2, -pi/2, 0 };


struct eepos{
    Matrix<double,3,1> ee_pos;
    Matrix<double,3,3> ee_rotm;
    Matrix<double,3,1> ee_eul;
};

Matrix<double,4,4> T10f(double th1){
    Matrix<double,4,4> m;
    m<< cos(th1), -sin(th1), 0, 0,
        sin(th1), cos(th1), 0, 0,
        0, 0, 1, D[0],
        0, 0, 0, 1;
    return m;
}

Matrix<double,4,4> T21f(double th2){
    Matrix<double,4,4> m;
    m<< cos(th2), -sin(th2), 0, 0,
        0, 0, -1, 0,
        sin(th2), cos(th2), 0, 0,
        0, 0, 0, 1;
    return m;
}

Matrix<double,4,4> T32f(double th3){
    Matrix<double,4,4> m;
    m<< cos(th3), -sin(th3), 0, A[1],
        sin(th3), cos(th3), 0, 0,
        0, 0, 1, D[2],
        0, 0, 0, 1;
    return m;
}

Matrix<double,4,4> T43f(double th4){
    Matrix<double,4,4> m;
    m<< cos(th4), -sin(th4), 0, A[2],
        sin(th4), cos(th4), 0, 0,
        0, 0, 1, D[3],
        0, 0, 0, 1;
    return m;
}

Matrix<double,4,4> T54f(double th5){
    Matrix<double,4,4> m;
    m<< cos(th5), -sin(th5), 0, 0,
        0, 0, -1, -D[4],
        sin(th5), cos(th5), 0, 0,
        0, 0, 0, 1;
    return m;
}

Matrix<double,4,4> T65f(double th6){
    Matrix<double,4,4> m;
    m<< cos(th6), -sin(th6), 0, 0,
        0, 0, 1, D[5],
        -sin(th6), -cos(th6), 0, 0,
        0, 0, 0, 1;
    return m;
}

#endif
