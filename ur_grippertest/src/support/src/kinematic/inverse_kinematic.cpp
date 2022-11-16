#include <iostream>
#include "parameters.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <math.h>
#include <complex>
#include "eul2rotm.cpp"

using namespace std_msgs;
using namespace Eigen;
using namespace std;


//inverse kinematic: given position of ee, find configuration of joints
Matrix<double,1,6> inverse_kinematic(Matrix<double,3,1> p60, Matrix<double,3,1> phi){
    Matrix<double,3,3> R60 = eul2rotm(phi);
    Matrix<double,4,4> T60;
    T60 << R60, p60,0,0,0,1;
    // cout<<"T60:\n"<<T60<<endl;

    //find th1
    Matrix<double,4,1> mat;
    mat <<0,0,-D[5],1;
    // cout<<"mat:\n"<<mat<<endl;

    // mat<<0,0,-D[6],1;
    Matrix<double,4,1> p50 = T60*mat;
    // cout<<"p50:\n"<<p50<<endl;
    double th1_1, th1_2;

    th1_1 = ((complex<double>)atan2(p50(1,0), p50(0,0)) + (complex<double>)acos(D[3]/(complex<double>)hypot(p50(1,0), p50(0,0)))).real()+(3.14159265/2);
    th1_2 = ((complex<double>)atan2(p50(1,0), p50(0,0)) - (complex<double>)acos(D[3]/(complex<double>)hypot(p50(1,0), p50(0,0)))).real()+(3.14159265/2);

    //find th5
    double th5_1, th5_2, th5_3, th5_4;
    th5_1 = +((complex<double>)acos((p60(0,0)*(complex<double>)sin(th1_1) - p60(1,0)*(complex<double>)cos(th1_1)-D[3]) / D[5])).real();
    th5_2 = -((complex<double>)acos((p60(0,0)*(complex<double>)sin(th1_1) - p60(1,0)*(complex<double>)cos(th1_1)-D[3]) / D[5])).real();
    th5_3 = +((complex<double>)acos((p60(0,0)*(complex<double>)sin(th1_2) - p60(1,0)*(complex<double>)cos(th1_2)-D[3]) / D[5])).real();
    th5_4 = -((complex<double>)acos((p60(0,0)*(complex<double>)sin(th1_2) - p60(1,0)*(complex<double>)cos(th1_2)-D[3]) / D[5])).real();
    
    Matrix<double,4,4> T06 = T60.inverse();
    Matrix<double,3,1> Xhat;
    Xhat << T06.block<3,1>(0,0); //blocco di dimensioni 3x1 a partire dalla casella 0,0
    Matrix<double,3,1> Yhat;
    Yhat << T06.block<3,1>(0,1);
    // cout<<"IK--Xhat: \n"<<Xhat<<endl;
    // cout<<"IK--Yhat: \n"<<Yhat<<endl;


    //find th6
    double th6_1 = ((complex<double>)atan2(((-Xhat(1,0)*sin(th1_1)+Yhat(1,0)*cos(th1_1)))/sin(th5_1), ((Xhat(0,0)*sin(th1_1)-Yhat(0,0)*cos(th1_1)))/sin(th5_1))).real();
    // cout<<"IK--th6_1: "<<th6_1<<endl;
    double th6_2 = ((complex<double>)atan2(((-Xhat(1,0)*sin(th1_1)+Yhat(1,0)*cos(th1_1))/sin(th5_2)), ((Xhat(0,0)*sin(th1_1)-Yhat(0,0)*cos(th1_1))/sin(th5_2)))).real();
    // cout<<"IK--th6_2: "<<th6_2<<endl;
    double th6_3 = ((complex<double>)atan2(((-Xhat(1,0)*sin(th1_2)+Yhat(1,0)*cos(th1_2))/sin(th5_3)), ((Xhat(0,0)*sin(th1_2)-Yhat(0,0)*cos(th1_2))/sin(th5_3)))).real();
    // cout<<"IK--th6_3: "<<th6_3<<endl;
    double th6_4 = ((complex<double>)atan2(((-Xhat(1,0)*sin(th1_2)+Yhat(1,0)*cos(th1_2))/sin(th5_4)), ((Xhat(0,0)*sin(th1_2)-Yhat(0,0)*cos(th1_2))/sin(th5_4)))).real();
    // cout<<"IK--th6_4: "<<th6_4<<endl;

    Matrix<double,4,4> T41m = (T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
    // cout<<"IK--T41m: "<<T41m<<endl;
    Matrix<double,3,1> p41_1 = T41m.block<3,1>(0,3);
    // cout<<"p41_1: "<<p41_1<<endl;
    double p41xz_1 = ((complex<double>)hypot(p41_1(0,0), p41_1(2,0))).real();

    T41m = (T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
    Matrix<double,3,1> p41_2 = T41m.block<3,1>(0,3);
    // cout<<"p41_2: "<<p41_2<<endl;
    double p41xz_2 = ((complex<double>)hypot(p41_2(0,0), p41_2(2,0))).real();

    T41m = (T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
    Matrix<double,3,1> p41_3 = T41m.block<3,1>(0,3);
    // cout<<"p41_3: "<<p41_3<<endl;
    double p41xz_3 = ((complex<double>)hypot(p41_3(0,0), p41_3(2,0))).real();

    T41m = (T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
    Matrix<double,3,1> p41_4 = T41m.block<3,1>(0,3);
    // cout<<"p41_4: "<<p41_4<<endl;
    double p41xz_4 = ((complex<double>)hypot(p41_4(0,0), p41_4(2,0))).real();
    // cout<<"IK--p41xz_1: "<<p41xz_1<<endl;
    // cout<<"IK--p41xz_2: "<<p41xz_2<<endl;
    // cout<<"IK--p41xz_3: "<<p41xz_3<<endl;
    // cout<<"IK--p41xz_4: "<<p41xz_4<<endl;

    //8 possible values for th3
    double th3_1,th3_2,th3_3,th3_4,th3_5,th3_6,th3_7,th3_8;
    th3_1 = (acos((complex<double>)((p41xz_1*p41xz_1 - A[1]*A[1] - A[2]*A[2]) / (2*A[1]*A[2])))).real();
    // cout<<"IK--th3_1: \n"<<th3_1<<endl;
    th3_2 = (acos((complex<double>)((p41xz_2*p41xz_2 - A[1]*A[1] - A[2]*A[2]) / (2*A[1]*A[2])))).real();
    th3_3 = (acos((complex<double>)((p41xz_3*p41xz_3 - A[1]*A[1] - A[2]*A[2]) / (2*A[1]*A[2])))).real();
    th3_4 = (acos((complex<double>)((p41xz_4*p41xz_4 - A[1]*A[1] - A[2]*A[2]) / (2*A[1]*A[2])))).real();
    th3_5 = -th3_1;
    th3_6 = -th3_2;
    th3_7 = -th3_3;
    th3_8 = -th3_4;

    //8 possible values for th2
    double th2_1,th2_2,th2_3,th2_4,th2_5,th2_6,th2_7,th2_8;
    th2_1 = (atan2(-p41_1(2,0), -p41_1(0,0))-asin((-A[2]*sin(th3_1))/p41xz_1));
    // cout<<"th2_1: "<<th2_1<<endl;
    th2_2 = ((complex<double>)atan2(-p41_2(2,0), -p41_2(0,0))-asin((-A[2]*sin(th3_2))/p41xz_2)).real();
    th2_3 = ((complex<double>)atan2(-p41_3(2,0), -p41_3(0,0))-asin((-A[2]*sin(th3_3))/p41xz_3)).real();
    th2_4 = ((complex<double>)atan2(-p41_4(2,0), -p41_4(0,0))-asin((-A[2]*sin(th3_4))/p41xz_4)).real();
 
    th2_5 = ((complex<double>)atan2(-p41_1(2,0), -p41_1(0,0))-asin((A[2]*sin(th3_1))/p41xz_1)).real();
    th2_6 = ((complex<double>)atan2(-p41_2(2,0), -p41_2(0,0))-asin((A[2]*sin(th3_2))/p41xz_2)).real();
    th2_7 = ((complex<double>)atan2(-p41_3(2,0), -p41_3(0,0))-asin((A[2]*sin(th3_3))/p41xz_3)).real();
    th2_8 = ((complex<double>)atan2(-p41_4(2,0), -p41_4(0,0))-asin((A[2]*sin(th3_4))/p41xz_4)).real();

    //find 8 values of th4
    double th4[8];
    Matrix<double,4,4> T43m = (T32f(th3_1)).inverse()*(T21f(th2_1)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
    Matrix<double,3,1> Xhat43 = T43m.block<3,1>(0,0);
   th4[0] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_2)).inverse()*(T21f(th2_2)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[1] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_3)).inverse()*(T21f(th2_3)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[2] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_4)).inverse()*(T21f(th2_4)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[3] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_5)).inverse()*(T21f(th2_5)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_1)).inverse()*(T54f(th5_1)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[4] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_6)).inverse()*(T21f(th2_6)).inverse()*(T10f(th1_1)).inverse()*T60*(T65f(th6_2)).inverse()*(T54f(th5_2)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[5] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_7)).inverse()*(T21f(th2_7)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_3)).inverse()*(T54f(th5_3)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[6] = atan2(Xhat43(1,0), Xhat43(0,0));

    T43m = (T32f(th3_8)).inverse()*(T21f(th2_8)).inverse()*(T10f(th1_2)).inverse()*T60*(T65f(th6_4)).inverse()*(T54f(th5_4)).inverse();
    Xhat43 = T43m.block<3,1>(0,0);
    th4[7] = atan2(Xhat43(1,0), Xhat43(0,0));

    Matrix<double,8,6> Th;
    Th<<th1_1, th2_1, th3_1, th4[0], th5_1, th6_1,
        th1_1, th2_2, th3_2, th4[1], th5_2, th6_2,
        th1_2, th2_3, th3_3, th4[2], th5_3, th6_3,
        th1_2, th2_4, th3_4, th4[3], th5_4, th6_4,
        th1_1, th2_5, th3_5, th4[4], th5_1, th6_1,
        th1_1, th2_6, th3_6, th4[5], th5_2, th6_2,
        th1_2, th2_7, th3_7, th4[6], th5_3, th6_3,
        th1_2, th2_8, th3_8, th4[7], th5_4, th6_4;

    // cout << "Th:\n "<<Th<<endl;
    //line 8 gives a better configuration when the ee is in a horizontal position and close to the ground
    //the only angle that can rotate the ee downward is the roll, since we are using he convention XYZ
    // cout<<"is right?: "<<p60(2,0)<<" "<<phi(0,0)<<endl;
    // cout<< (p60(2,0)<=0.2 && phi(0,0)>=-0.2 && phi(0,0)>=-0.2)<<endl;
    for(int i=7;i>=0;i--){
        if (th4[i]<0){
            cout<<"returing line "<<i<<endl;
            return Th.block<1,6>(i,0);
        }
        
    }
    return Th.block<1,6>(6,0);
}
