#include <iostream>
#include "parameters.h"
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include "direct_kinematic.cpp"
#include "inverse_kinematic.cpp"
#include "support/messaggio.h"
#include <ctime>
#include <cstdio>
#define hz 1000
#define step 0.001
using namespace std_msgs;
using namespace Eigen;
using namespace std;

//Questo pubblichera direttamente su ros
typedef Matrix<double,3,1> vector3d;
eepos pos;
float gr_last=0;

//create a motion plan between 2 points using inverse kinematic
eepos p2pmotionplan(vector3d xEs, vector3d phiEs, vector3d xEf, vector3d phiEf, double T,int argc, char* argv[]/*, double gr_pos*/){
    ros::init(argc,argv, "motionplanner");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<support::messaggio>("position_channel",1000);
    support::messaggio msg;

    Matrix<double,1,6> qEs,qEf;

    //i +pi/2 e +pi è perche gazebo considerava come posizione 0,0,0
    //una posizione diversa da quella che consideravamo noi

    
    qEs=(Matrix<double,1,6>)inverse_kinematic(xEs,phiEs);
    qEf=(Matrix<double,1,6>)inverse_kinematic(xEf,phiEf);
    //gazezbo values goes from -2pi to 2pi
    //we normalize it to -pi pi
    for(int j=0;j<6;j++){
             if(qEs(0,j)>=3.1415) qEs(0,j)-=6.283;
             else if(qEs(0,j) <=-3.1415) qEs(0,j)+=6.283;

             if(qEf(0,j)>=3.1415) qEf(0,j)-=6.283;
             else if(qEf(0,j) <=-3.1415) qEf(0,j)+=6.283;
         }
    // cout<<"qEs:\n"<<qEs<<"\nqEf\n"<<qEf<<endl;
    Matrix<double,6,4> Aa;
    Matrix<double,4,4> M;
    M << 1, 0, 0, 0,
         0, 1, 0, 0,
         1, T, T*T, T*T*T,
         0, 1, 2*T, 3*T*T;
    Matrix<double,4,1> a;
    Matrix<double,4,1> b;

    //q=joint configuration
    //N.B.: originally M would have been
    // 1  minT  minT^2  mint^3  = qs(t)
    // 0   1   2*minT   3*minT  = qs'(t)
    // 1  maxT  maxT^2  maxT^3  = qf(t)
    // 0   1   2*maxT   3*maxT  = qf'(t)
    //however we always use minT=0 and maxT=T

    //b = [qEs, qEs', qEf, qEf'] for every joint
    //we found a = coefficient of the cubic function to be multiplied
    //with t to get joint configuration

    for(int i=0;i<6;i++){
        //b(0,0)<<qEs(0,i),0,qEf(0,i),0;
        b(0,0)=qEs(0,i);
        b(1,0)=0;
        b(2,0)=qEf(0,i);
        b(3,0)=0;
        a=M.inverse()*b;
        Aa(i,0)=a(0,0);
        Aa(i,1)=a(1,0);
        Aa(i,2)=a(2,0);
        Aa(i,3)=a(3,0);
    }

    //Aa: matrix that has as rows the coefficients for every joint
    // cout<<"Aa\n"<<Aa<<endl;

    Matrix<double,6,1> th;
    //6 è dimensione di qEs

    ros::Rate rate(hz);
    double t=0;

    T=T;
    for(t=0;t<T;t+=step){
        for(int i=0;i<6;i++){
            th(i,0)=Aa(i,0)+Aa(i,1)*t+Aa(i,2)*t*t+Aa(i,3)*t*t*t;
        }
        // cout<<th(0,0)<<" "<<th(1,0)<<" "<<th(2,0)<<" "<<th(3,0)<<" "<<th(4,0)<<" "<<th(5,0)<<" \n";
        
        msg.sh_pan_pos.data=th(0,0);
        msg.sh_lift_pos.data=th(1,0);
        msg.elb_pos.data=th(2,0);
        msg.wr1_pos.data=th(3,0);
        msg.wr2_pos.data=th(4,0);
        msg.wr3_pos.data=th(5,0);
        pub.publish(msg);

        t+=step;
        rate.sleep();
    }

    pos.ee_pos=xEf;
    pos.ee_eul=phiEf;
    cout<<-pos.ee_pos(0,0)<<" "<<-pos.ee_pos(1,0)<<" "<<pos.ee_pos(2,0)<<" "<<pos.ee_eul(0,0)<<" "<<pos.ee_eul(1,0)<<" "<<pos.ee_eul(2,0)<<" "<<endl;;
    return pos;
}