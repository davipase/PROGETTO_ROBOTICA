#include "ros/ros.h"
#include <iostream>
#include <thread>
#include <ctime>
#include <cstdio>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "move.h"
#include "support/messaggio.h"

using namespace std;
ros::NodeHandle* nh=NULL;

joints_state::joints_state(){
    shoulder_pan_state.data=-pi;
    shoulder_lift_state.data=-pi/2;
    elbow_state.data=-pi/2;
    wrist1_state.data=-pi/2;
    wrist2_state.data=pi/2;
    wrist3_state.data=0.0;
    gripper_state.data=0.0;
}

//initializing subscriber
void joints_state::initSubs(){
    pub_sh_pan = nh->advertise<std_msgs::Float64>("/shoulder_pan_joint_position_controller/command",1000);
    pub_sh_lft = nh->advertise<std_msgs::Float64>("/shoulder_lift_joint_position_controller/command",1000);
    pub_elb = nh->advertise<std_msgs::Float64>("/elbow_joint_position_controller/command",1000);
    pub_wr1 = nh->advertise<std_msgs::Float64>("/wrist_1_joint_position_controller/command",1000);
    pub_wr2 = nh->advertise<std_msgs::Float64>("/wrist_2_joint_position_controller/command",1000);
    pub_wr3 = nh->advertise<std_msgs::Float64>("/wrist_3_joint_position_controller/command",1000);
    pub_grp = nh->advertise<std_msgs::Float64>("/gripper_position_controller/command",1000);
    
}


void joints_state::set_pan_state(std_msgs::Float64 x){shoulder_pan_state.data=x.data;}
void joints_state::set_lift_state(std_msgs::Float64 x){shoulder_lift_state.data=x.data;}
void joints_state::set_elbow_state(std_msgs::Float64 x){elbow_state.data=x.data;}
void joints_state::set_wrist1_state(std_msgs::Float64 x){wrist1_state.data=x.data;}
void joints_state::set_wrist2_state(std_msgs::Float64 x){wrist2_state.data=x.data;}
void joints_state::set_wrist3_state(std_msgs::Float64 x){wrist3_state.data=x.data;}
void joints_state::set_gripper_state(std_msgs::Float64 x){gripper_state.data=x.data;}


std_msgs::Float64 joints_state::get_pan_state(){return shoulder_pan_state;}
std_msgs::Float64 joints_state::get_lift_state(){return shoulder_lift_state;}
std_msgs::Float64 joints_state::get_elbow_state(){return elbow_state;}
std_msgs::Float64 joints_state::get_wrist1_state(){return wrist1_state;}
std_msgs::Float64 joints_state::get_wrist2_state(){return wrist2_state;}
std_msgs::Float64 joints_state::get_wrist3_state(){return wrist3_state;}
std_msgs::Float64 joints_state::get_gripper_state(){return gripper_state;}

//set new position, and publish joint values
void move(const support::messaggio& msg){
    jst.set_pan_state(msg.sh_pan_pos);
    jst.set_lift_state(msg.sh_lift_pos);
    jst.set_elbow_state(msg.elb_pos);
    jst.set_wrist1_state(msg.wr1_pos);
    jst.set_wrist2_state(msg.wr2_pos);
    jst.set_wrist3_state(msg.wr3_pos);
    

    jst.pub_sh_pan.publish(jst.get_pan_state());
    jst.pub_sh_lft.publish(jst.get_lift_state());
    jst.pub_elb.publish(jst.get_elbow_state());
    jst.pub_wr1.publish(jst.get_wrist1_state());
    jst.pub_wr2.publish(jst.get_wrist2_state());
    jst.pub_wr3.publish(jst.get_wrist3_state());
}


void callBack(const support::messaggio& msg){
    move(msg);
}

//publish gripper value
void move_grp(const std_msgs::Float64& msg){
    clock_t start=clock();
    double duration=2;
    while((double)((clock()-start)/CLOCKS_PER_SEC) < duration){
        jst.pub_grp.publish(msg);
    }
}
