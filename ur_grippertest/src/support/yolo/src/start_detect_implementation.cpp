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
#include "start_detect.h"

#define camera_angle_x 0.5235987755983
#define camera_angle_y 0.3926990816987
#define camera_heigth 1.6

sensor_msgs::PointCloud ptc;
// sensor_msgs::PointCloud ptc_up;

map<int,brick> lego;

ros::Publisher pub_position;
ros::Publisher pub_python;


brick::brick()
{
    x=0; y=0; z=0;
    R=0; P=0; Y=0;
    width=0; height=0;
    confidence=0;
    classification=0;
}


brick::brick(float u, float v, float Y,float width, float height)
{
    this->u=u/img_width;
    this->v=v/img_height;

    this->Y=Y;
    this->Y=this->Y*pi/180;
    if(this->height > this->width) this->Y+=pi/2;

    this->width=width;
    this->height=height;

    x=-((this->v-0.5)*(camera_heigth*tan(camera_angle_y))*2)-0.6;
    y=-((this->u-0.5)*(camera_heigth*tan(camera_angle_x))*2);

    confidence=0;
    classification=0;
}

//reset a piece using information from up camera
void brick::reset_up(float u, float v, float Y,float width, float height)
{
    this->u=u/img_width;
    this->v=v/img_height;

    this->Y=Y;
    this->Y=this->Y*pi/180;

    this->width=width;
    this->height=height;
    if(this->height > this->width) this->Y+=pi/2;

    

    x=-((this->v-0.5)*(camera_heigth*tan(camera_angle_y))*2)-0.6;
    y=-((this->u-0.5)*(camera_heigth*tan(camera_angle_x))*2);
    
    confidence=0;
    classification=0;
}

//reset a block using information from front camera
void brick::reset_front(float u, float v, float width, float height, int classification, float confidence)
{
    this->u=u;
    this->v=v;
    this->width=width;
    this->height=height;
    this->classification=classification;
    this->confidence=confidence;
}

//used for point 3, not finished
int brick::noti()
{
    int o=width/0.03;
    int p=height/0.03;
    double r1=width-o*0.03;
    double r2=height-p*0.03;
    if(r1>0.002 || r1<0.028) return 1; //r1=width the unknown side
    if(r2>0.002 || r2<0.028) return 2; //r2=height the unknown side
    return 0;
}

ostream& operator<<(ostream& os, const brick& br)
{
    // cout<<" u:"<<br_up.u<<" v:"<<br_up.v<<" x:"<<br_up.x.data<<" y:"<<br_up.y.data<<" Y:"<<br_up.Y.data<<endl;
    os <<"u:"<< br.u <<" v:"<< br.v <<" x:"<<br.x<<" y:"<<br.y<<" Y:"<<br.Y<<" classe:"<<br.classification;
    return os;
}


bool operator==(const brick& br1, const brick& br2)
{
    float l=sqrt(pow((br1.x-br2.x),2)+pow((br1.y-br2.y),2));
    cout<<"l:"<<l<<endl;
    return (l<=0.05);
}


void brick::operator=(const brick& br)
{
    this->classification=br.classification;
    this->confidence=br.confidence;
}


float brick::getu(){ return this->u; }
float brick::getv(){ return this->v; }
float brick::getx(){ return this->x; }
float brick::gety(){ return this->y; }
float brick::getz(){ return this->z; }
float brick::getR(){ return this->R; }
float brick::getP(){ return this->P; }
float brick::getY(){ return this->Y; }
float brick::getWidth(){ return this->width; }
float brick::getHeight(){ return this->height; }
float brick::getConf(){ return this->confidence; }
int brick::getClass(){ return this->classification; }

void brick::setx(float x){ this->x=x; }
void brick::sety(float y){ this->y=y; }
// void brick::setSide(int i){ this->on_the_side=i; }



//callbacks used to update point cloud of camera
void callBack(const sensor_msgs::PointCloud2& msg)
{
    sensor_msgs::convertPointCloud2ToPointCloud(msg,ptc);
}


// void callBack_up(const sensor_msgs::PointCloud2& msg)
// {
//     sensor_msgs::convertPointCloud2ToPointCloud(msg,ptc_up);
// }


void detect(const Float64& msg)
{
    ROS_INFO("Start_detect::message received\n");

    //deleting old experiences and taking photo up
    system("rm -rf $(rospack find support)/yolo/yolov5/runs/detect/exp");
    system("rm -rf $(rospack find support)/img_up.txt");
    // cout<<"Sono in detect\n";
    system("rosservice call image_saver/save");
    // cout<<"foto davanti scattata\n";

    ROS_INFO("Start_detect::front image taken\n");
    
    lego.clear();
    int j=0;
    // cout<<"\n\n----CAMERA SOPRA----\n";
    Float64 ms;
    ms.data=1;

    //starting up image analyzer
    pub_python.publish(ms);
    ROS_INFO("Start_detect::analyzing up image...\n");
    //start detection program

    ROS_INFO("Start_detect::analyzing front image...\n");

    system("python3 $(rospack find support)/yolo/yolov5/detect.py --weights $(rospack find support)/yolo/yolov5/best.pt --img 640 --conf 0.45 --source $(rospack find support)/yolo/img.jpg --save-txt --save-conf");
    cout<<"analizzata foto davanti\n";

    ROS_INFO("Start_detect::front image analyzed\n");

    float u,v,x,y,z,R,P,Y,width,height,classification,confidence;
    // brick br,br_up;

    ifstream infile;
    
    infile.open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/img_up.txt");

    ROS_INFO("Start_detect::saving up image results...\n");
    //reading results and inserting them in the map
    infile>>u;
    infile>>v;
    infile>>z;
    infile>>R;
    infile>>P;
    infile>>Y;
    infile>>width;
    infile>>height;    
    // int noti;
    
    while(infile.good()){

        br_up.reset_up(u,v,Y,width,height);
        // noti=br_up.noti();
        // br_up.setSide(noti);
        // if(noti==0){ //if noti==0: all side are known: correct or upside down
        lego.insert(pair<int,brick> (j,br_up));
        j++;
        // cout<<"width:"<<width<<" height:"<<height<<endl;

        infile>>u;
        infile>>v;
        infile>>z;
        infile>>R;
        infile>>P;
        infile>>Y;
        infile>>width;
        infile>>height;
        // }
        //TODO
    }
    infile.close();

    ROS_INFO("Start_detect::up image results saved\n");

    
    j=0;
    
    infile.open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/yolo/yolov5/runs/detect/exp/labels/img.txt");

    map<int,brick>::iterator it;

    //reading results from front camera and merging them with data from up camera
    //merging is based on proximity, values get with point cloud
    ROS_INFO("Start_detect::reading front image results...\n");

    infile>>classification;
    infile>>u;
    infile>>v;
    infile>>width;
    infile>>height;
    infile>>confidence;
    // cout<<"----CAMERA DAVANTI----\n";

    while(infile.good()){
        // cout<<"sono dentro\n";

        br.reset_front(u,v,width,height,classification,confidence);

        int l=((int)(br.getv()*img_height-1)*img_width) + (int)(br.getu()*img_width);

        br.setx(-1.3+(ptc.points[l].z*cos(pitch) - ptc.points[l].y*sin(pitch)));
        br.sety(-ptc.points[l].x);

        int cl = br.getClass();
        if(cl==2 || cl==1)  br.setx(br.getx()+0.02);
        cout<<"br::"<<br;
        
        //operator == checks if the distance between 2 pieces if lower than a treshold
        //operator << prints the important attributes of the class
        //operator = assign to the first operand the classification and confidence of the second
        for(it=lego.begin();it!=lego.end();it++){
            cout<<"\nunisco\n";
            cout<<"it->second::"<<it->second<<endl;
            if(it->second == br){
                if(br.getConf()>it->second.getConf()){
                    cout<<"trovato\n";
                    it->second=br; 
                    
                    break;
                }
                else{
                    cout<<endl;
                    break;
                }
            } 
        }
        infile>>classification;
        infile>>u;
        infile>>v;
        infile>>width;
        infile>>height;
        infile>>confidence;
    }
    infile.close();

    ROS_INFO("Start_detect::front image results saved\n");
    ROS_INFO("Start_detect::merging results...\n");

    //sending positions of pieces to decision maker
    support::msg_position m;
    for(it=lego.begin();it!=lego.end();it++){
        m.x.data=it->second.getx();
        m.y.data=it->second.gety();
        m.z.data=it->second.getz();
        m.R.data=it->second.getR();
        m.P.data=it->second.getP();
        m.Y.data=it->second.getY();
        m.classification.data=it->second.getClass();
        // cout<<"Sto inviando: x:"<<m.x.data<<" y:"<<m.y.data<<" z:"<<m.z.data<<endl<<" Y:"<<m.Y.data<<" class:"<<m.classification.data<<endl;
        pub_position.publish(m);
        // cout<<m.classification.data<<"messaggio inviato\n";
    }
    // cout<<"finito\n";   
    ROS_INFO("Start_detect::Finished\n");
}