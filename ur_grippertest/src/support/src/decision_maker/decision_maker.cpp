#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h> 
#include "decision_maker.h"
#include <thread>
using namespace std;
using namespace std_msgs;



void callBack(const support::msg_position& msg);
float calcolaT(Matrix<double,3,1>xEs,Matrix<double,3,1> xEf);
void correctxEf();
void correctphiEf();

int main(int argc, char* argv[]){
    //sbscription and initialising
    ros::init(argc, argv, "decision_maker");
    ros::NodeHandle nh;
    argc1=argc;
    argv1=argv;

    for(int j=0;j<11;j++) sorted_legos[j]=0;

    // ros::Publisher pub = nh.advertise<std_msgs::Float64>("/gripper_position_controller/command",0);
    pubgr= new ros::Publisher;
    *pubgr=nh.advertise<std_msgs::Float64>("gripper_channel",1);
    ros::Subscriber sub_pos = nh.subscribe("/legos_position",100,callBack);

    //X E Y SONO ESATTAMENTE OPPOSTE A QUELLO CHE DOVREBBERO ESSERE
    //initial pose of the robot
    xEs<<-0.487, 0.1092, 0.514;
    xEs(2,0)-=0.082;
    phiEs<<-pi/2, 0, -pi/2;
    phiEs(0,0)+=(pi/2);
    phiEs(1,0)+=(pi);

    ROS_INFO("Decision_maker::node ready\n");
    // cout<<"Sono pronto, aspetto\n";
    ros::spin();
}







