#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <std_msgs/Float64.h>
using namespace std;


int main(int argc, char* argv[]){

    ros::init(argc,argv,"start_assignment1");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::Float64>("/start_detection",0);

    std_msgs::Float64 msg;
    msg.data=1;


    srand(time(NULL));

    //generating random class
    int c=rand()%11;
    cout<<"c:"<<c<<endl;

    //generating random coordinates
    double x=rand()%5000+3000;
    x=-x/10000;
    cout<<"x:"<<x<<endl;

    double y=rand()%6800-3400;
    y=-y/10000;
    cout<<"y:"<<y<<endl;
    char command[200];

    double Y=rand()%62830-31415;
    Y=Y/10000;
    cout<<"Y:"<<Y<<endl;

    //set right command to spawn model
    switch(c){
        case 0:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/112/112.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 112",x,y,Y);
            break;
        }
        case 1:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/121/121.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 121",x,y,Y);
            break;
        }
        case 2:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/122/122.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 122",x,y,Y);
            break;
        }
        case 3:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/122doublearch/122doublearch.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 122doublearch",x,y,Y);
            break;
        }
        case 4:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/122slope/122slope.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 122slope",x,y,Y);
            break;
        }
        case 5:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/132/132.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 132",x,y,Y);
            break;
        }
        case 6:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/132arch/132arch.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 132arch",x,y,Y);
            break;
        }
        case 7:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/141/141.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 141",x,y,Y);
            break;
        }
        case 8:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/142/142.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 142",x,y,Y);
            break;
        }
        case 9:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/222/222.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 222",x,y,Y);
            break;
        }
        case 10:{
            sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/222arch/222arch.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 222arch",x,y,Y);
            break;
        }
    }

    system(command);
    pub.publish(msg);
    
    cout<<"pubblicato\n";
    return 0;

}