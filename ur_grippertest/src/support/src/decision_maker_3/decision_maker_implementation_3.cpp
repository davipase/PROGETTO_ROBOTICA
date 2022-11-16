#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h> 
#include <thread>
#include "decision_maker_3.h"
#include <csignal>
using namespace std;
using namespace std_msgs;

int argc1;
char** argv1=NULL;

Matrix<double,3,1> xEs,phiEs,xEf,phiEf;
int sorted_legos[11];
ros::Publisher* pubgr=NULL;
eepos p;
bool initialized=false;


float calcolaT(Matrix<double,3,1>xEs,Matrix<double,3,1> xEf){
    float l=sqrt(pow(xEs(0,0)-xEf(0,0),2)+pow(xEs(1,0)-xEf(1,0),2)+pow(xEs(2,0)-xEf(2,0),2));
    return (l/0.2+2);
}


void correctxEf(){
    xEf(0,0)=-xEf(0,0);
    xEf(1,0)=-xEf(1,0);
}

void correctphiEf(){
    
}

void callBack(const support::msg_position& msg){
    float x=msg.x.data;
    float y=msg.y.data;
    int c=msg.classification.data;
    ROS_INFO("Ricevuto posizione lego: x=%f, y=%f, class=%d",x,y,c);

    
    Float64 gr_pos;
    gr_pos.data=0;
    float t;
    //-----STEP 1: from starting position to above the lego-----
    xEf(0,0)=msg.x.data;
    xEf(1,0)=msg.y.data;
    xEf(2,0)=0.3;
    // cout<<"^^xEf prima:"<<xEf<<endl;
    xEf(0,0)=-xEf(0,0);
    xEf(1,0)=-xEf(1,0);
    // cout<<"^^xEf dopo:"<<xEf<<endl;
    phiEf(0,0)=1.57075;
    phiEf(1,0)=0;
    phiEf(2,0)=msg.Y.data;

    float dx,dy;
    switch(c){
        // case 0:{dx=0.205*sin(phiEf(2,0)); dy=0.205*cos(phiEf(2,0)); break;}
        // case 1:{dx=0.205*sin(phiEf(2,0)); dy=0.205*cos(phiEf(2,0)); break;}
        // case 2:{dx=0.205*sin(phiEf(2,0)); dy=0.205*cos(phiEf(2,0)); break;}
        // case 3:{dx=0.205*sin(phiEf(2,0)); dy=0.205*cos(phiEf(2,0)); break;}
        case 4:{dx=(0.0235+0.08)*sin(phiEf(2,0)); dy=(0.0235+0.08)*cos(phiEf(2,0)); break;}
        case 5:{dx=(0.0235+0.08)*sin(phiEf(2,0)); dy=(0.0235+0.08)*cos(phiEf(2,0)); break;}
        case 6:{dx=(0.022+0.08)*sin(phiEf(2,0)); dy=(0.022+0.08)*cos(phiEf(2,0)); break;}
        case 7:{dx=(0.019+0.08)*sin(phiEf(2,0)); dy=(0.019+0.08)*cos(phiEf(2,0)); break;}
        // case 8:{dx=0.205*sin(phiEf(2,0)); dy=0.205*cos(phiEf(2,0)); break;}
        case 9:{dx=(0.022+0.08)*sin(phiEf(2,0)); dy=(0.022+0.08)*cos(phiEf(2,0)); break;}
        // case 10:{dx=0.205*sin(phiEf(2,0)); dy=0.205*cos(phiEf(2,0)); break;}
        default:{dx=0.0205*sin(phiEf(2,0)); dy=0.0205*cos(phiEf(2,0)); break;}
    }
    xEf(0,0)-=dx;
    xEf(1,0)-=dy;
    // phiEf(0,0)=-1.57075;
    // phiEf(0,0)+=(pi/2);
    phiEf(1,0)+=(pi);
    t=calcolaT(xEs,xEf);
    p=p2pmotionplan(xEs,phiEs,xEf,phiEf,t,argc1,argv1);
    pubgr->publish(gr_pos);
    xEs=p.ee_pos;
    phiEs=p.ee_eul;

    //-----STEP 2: lower grip and pick up lego-----
    xEf(2,0)=0.1;//right height to pick objects ??????????????????????????
    if(c==1 || c==2) gr_pos.data=0.25;
    else gr_pos.data=0.52;
    t=4;//right time so lowering is not too fast
    p=p2pmotionplan(xEs,phiEs,xEf,phiEf,t,argc1,argv1);
    pubgr->publish(gr_pos);
    xEs=p.ee_pos;
    phiEs=p.ee_eul;

    //-----STEP 3: go back up-----
    xEf(2,0)=0.2;
    t=3;
    p=p2pmotionplan(xEs,phiEs,xEf,phiEf,t,argc1,argv1);
    xEs=p.ee_pos;
    phiEs=p.ee_eul;

    //-----STEP 4: rotate-----
    // switch(c){
    //     case 0:{xEf(0,0)=0.4;  xEf(1,0)= 0;      break;}
    //     case 1:{xEf(0,0)=0.7;  xEf(1,0)= 0.25;   break;}
    //     case 2:{xEf(0,0)=0.7;  xEf(1,0)=-0.25;  break;}
    //     case 3:{xEf(0,0)=0.4;  xEf(1,0)=-0.25;  break;}
    //     case 4:{xEf(0,0)=0.55; xEf(1,0)= 0.375; break;}
    //     case 5:{xEf(0,0)=0.55; xEf(1,0)= 0.125;  break;}
    //     case 6:{xEf(0,0)=0.55; xEf(1,0)=-0.375; break;}
    //     case 7:{xEf(0,0)=0.4;  xEf(1,0)=-0.5;   break;}
    //     case 8:{xEf(0,0)=0.4;  xEf(1,0)= 0.5;    break;}
    //     case 9:{xEf(0,0)=0.55; xEf(1,0)=-0.125; break;}
    //     case 10:{xEf(0,0)=0.4; xEf(1,0)= 0.25;   break;}
    // }
    // xEf(0,0)=-xEf(0,0);  xEf(1,0)=-xEf(1,0);
    // phiEf(2,0)=0;
    // t=calcolaT(xEs,xEf);
    phiEf(1,0)=pi;
    t=2;
    p=p2pmotionplan(xEs,phiEs,xEf,phiEf,t,argc1,argv1);
    xEs=p.ee_pos;
    phiEs=p.ee_eul;

    //-----STEP 5: lower gripper and release lego-----
    xEf(2,0)=0.03;
    // sorted_legos[c]++;
    gr_pos.data=0;
    // phiEf(2,0)=0;
    t=5;  //right time so lowering is not too fast
    p=p2pmotionplan(xEs,phiEs,xEf,phiEf,t,argc1,argv1);
    pubgr->publish(gr_pos);
    xEs=p.ee_pos;
    phiEs=p.ee_eul;

    // //-----STEP 6: move back up ready to go to another object-----
    xEf(2,0)=0.4;
    // t=3;
    p=p2pmotionplan(xEs,phiEs,xEf,phiEf,t,argc1,argv1);
    xEs=p.ee_pos;
    phiEs=p.ee_eul;

}
