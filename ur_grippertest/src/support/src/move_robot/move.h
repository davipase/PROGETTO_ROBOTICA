#ifndef __MOVE__
#define __MOVE__

#include "ros/ros.h"
#include <iostream>
#include <thread>
#include <ctime>
#include <cstdio>
#include "std_msgs/Float64.h"
#include "control_msgs/JointControllerState.h"
#include "support/messaggio.h"
#define pi 3.14159265

class joints_state{
    private:

        std_msgs::Float64 shoulder_pan_state;
        std_msgs::Float64 shoulder_lift_state;
        std_msgs::Float64 elbow_state;
        std_msgs::Float64 wrist1_state;
        std_msgs::Float64 wrist2_state;
        std_msgs::Float64 wrist3_state;
        std_msgs::Float64 gripper_state;

    public:

        ros::Publisher pub_sh_pan;
        ros::Publisher pub_sh_lft;
        ros::Publisher pub_elb;
        ros::Publisher pub_wr1;
        ros::Publisher pub_wr2;
        ros::Publisher pub_wr3;
        ros::Publisher pub_grp;

        joints_state();
        void initSubs();

        void set_pan_state(std_msgs::Float64 x);
        void set_lift_state(std_msgs::Float64 x);
        void set_elbow_state(std_msgs::Float64 x);
        void set_wrist1_state(std_msgs::Float64 x);
        void set_wrist2_state(std_msgs::Float64 x);
        void set_wrist3_state(std_msgs::Float64 x);
        void set_gripper_state(std_msgs::Float64 x);

        std_msgs::Float64 get_pan_state();
        std_msgs::Float64 get_lift_state();
        std_msgs::Float64 get_elbow_state();
        std_msgs::Float64 get_wrist1_state();
        std_msgs::Float64 get_wrist2_state();
        std_msgs::Float64 get_wrist3_state();
        std_msgs::Float64 get_gripper_state();
};

joints_state jst;

void callBack(const support::messaggio& msg);
void move(const support::messaggio& msg);
void move_grp(const std_msgs::Float64& msg);

#endif
