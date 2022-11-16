#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include "support/msg_position.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <iterator>
#include <map>
#include <fstream>
#include <math.h>
#include "start_detect.h"
#include "start_detect_implementation_3.cpp"
#define pi 3.14159265
#define img_width 640
#define img_height 480
#define cupx 1.847520
#define cupy 1.3254834
#define pitch 0.8
using namespace std;
using namespace std_msgs;
using namespace Eigen;

int num=0;
bool is_needed=false;

ros::NodeHandle* nh=NULL;

bool initiated=false;

void callBack(const sensor_msgs::PointCloud2& msg);
void callBack_up(const sensor_msgs::PointCloud2& msg);
void detect(const Float64& msg);


int main(int argc, char* argv[]){

    cout<<"Inizio\n";
    ros::init(argc, argv, "start_detect");
    nh=new ros::NodeHandle();
    //initializin subscriber and publishers
    pub_position = nh->advertise<support::msg_position>("/legos_position",100);
    pub_python = nh-> advertise<std_msgs::Float64>("/start_python",10);
    pub_position_3 = nh->advertise<support::msg_position>("/legos_position_3",100);

    ros::Subscriber sub = nh->subscribe("/camera/depth/points",1,callBack);
    // ros::Subscriber subup = nh->subscribe("/camera_up/depth/points",1,callBack_up);
    ros::Subscriber sub2 = nh->subscribe("/start_detection",0,detect);
    ros::Subscriber sub3 = nh->subscribe("/start_detection_3",0,detect_3);
    
    // cout<<"mi sono iscritto, aspetto\n";

    std_msgs::Float64 msg;
    msg.data=1;

    // ROS_INFO("Start detect ready\n");
    ROS_INFO("Start_detect::node started\n");
    ros::spin();
    }

// LISTA CLASS1:
// 0: 122 - due
// 1: 222arch - due fill
// 2: 222 - due per due
// 3: 121 - due small
// 4: 142 - quattro
// 5: 141 - quattro small
// 6: 132 - tre
// 7: 112 - uno
// 8: 122slope - uno chamfer
// 9: 132arch - uno fill
// 10: 122doublearch - uno twin