#ifndef __DM__
#define __DM__

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include "support/messaggio.h"
#include "support/msg_position.h"
#include "../kinematic/p2pmotionplan.cpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h> 
#include "../kinematic/parameters.h"
#include "decision_maker_implementation_3.cpp"
#include "control_msgs/JointControllerState.h"
#include <thread>
#define hz2 100
using namespace std;
using namespace std_msgs;


void callBack(const support::msg_position& msg);

float calcolaT(Matrix<double,3,1>xEs,Matrix<double,3,1> xEf);

void correctxEf();

void correctphiEf();

#endif
