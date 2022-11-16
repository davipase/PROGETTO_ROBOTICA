#include "ros/ros.h"
#include <iostream>
#include <ctime>
#include <cstdio>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "move.h"
#include "implementation.cpp"
#include "support/messaggio.h"
using namespace std;



int main(int argc, char* argv[]){

    //initializing ros and the class istance
    ros::init(argc, argv, "position_actuator");
    nh=new ros::NodeHandle();
    jst.initSubs();

    //subscribing to position channel to receive orders
    ros::Subscriber sub = nh->subscribe("position_channel",1000,callBack);
    ros::Subscriber subgr = nh->subscribe("gripper_channel",1000,move_grp);

    //waiting for messages
    cout<<"Sono pronto, aspetto\n";
    ros::spin();

}

void callBack(const support::messaggio& msg);
void move(const support::messaggio& msg);
void move_grp(const std_msgs::Float64& msg);
