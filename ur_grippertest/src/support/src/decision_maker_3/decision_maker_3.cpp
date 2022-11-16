#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h> 
#include "decision_maker_3.h"
#include <thread>
using namespace std;
using namespace std_msgs;



void callBack(const support::msg_position& msg);
float calcolaT(Matrix<double,3,1>xEs,Matrix<double,3,1> xEf);
void correctxEf();
void correctphiEf();

int main(int argc, char* argv[]){
    ros::init(argc, argv, "decision_maker_3");
    ros::NodeHandle nh;
    argc1=argc;
    argv1=argv;

    for(int j=0;j<11;j++) sorted_legos[j]=0;

    // ros::Subscriber sub = nh.subscribe("/gripper_position_controller/state",0,callBack);
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/gripper_position_controller/command",0);
    ros::Publisher pub2 = nh.advertise<std_msgs::Float64>("/start_detection_3",1);
    pubgr= new ros::Publisher;
    *pubgr = nh.advertise<std_msgs::Float64>("gripper_channel",1);
    ros::Subscriber sub_pos = nh.subscribe("/legos_position_3",100,callBack);
    // ros::Subscriber sub_init = nh.subscribe("/set_pose",1,initialize);


    // double t;
    //X E Y SONO ESATTAMENTE OPPOSTE A QUELLO CHE DOVREBBERO ESSERE
    xEs<<-0.487, 0.1092, 0.514;
    xEs(2,0)-=0.082;
    phiEs<<-pi/2, 0, -pi/2;
    phiEs(0,0)+=(pi/2);
    phiEs(1,0)+=(pi);
    cout<<"Sono pronto, aspetto\n";
    ros::spin();
}







