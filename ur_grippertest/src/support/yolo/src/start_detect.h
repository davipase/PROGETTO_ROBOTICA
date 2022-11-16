#ifndef __START_DETECT__
#define __START_DETECT__

#include <iostream>
#include <ros/ros.h>
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
#define pi 3.14159265
#define img_width 640
#define img_height 480
#define cupx 1.847520
#define cupy 1.3254834
#define pitch 0.8
using namespace std;
using namespace std_msgs;
using namespace Eigen;


class brick{
    private:
        float u;
        float v;
        float x;
        float y;
        float z;
        float R;
        float P;
        float Y;
        float width;
        float height;
        int classification;
        float confidence;
        // int on_the_side;
        //0 = is correct or upside down
        //1 = is on the side, width is the unknown side
        //2 = is on the side, height is the unknown side

    public:
        brick();
        brick(float u, float v, float Y,float width, float height);
        void reset_up(float u, float v, float Y,float width, float height);
        void reset_front(float u, float v, float width, float height, int classification, float confidence);
        int noti();

        float getu();
        float getv();
        float getx();
        float gety();
        float getz();
        float getR();
        float getP();
        float getY();
        float getWidth();
        float getHeight();
        float getConf();
        int getClass();

        void setx(float x);
        void sety(float y);
        void setSide(int i);

        void operator=(const brick& br);

    friend ostream& operator<<(ostream& os, const brick& br);
    friend bool operator==(const brick& br1, const brick& br2);
}br,br_up;

ostream& operator<<(ostream& os, const brick& br);
bool operator==(const brick& br1, const brick& br2);

void callBack(const sensor_msgs::PointCloud2& msg);
// void callBack_up(const sensor_msgs::PointCloud2& msg);
void detect(const Float64& msg);


#endif
