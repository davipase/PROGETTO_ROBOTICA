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
#include "start_detect_implementation.cpp"
#include <csignal>

map<int,brick> lego3_1,lego3_2;

void rotate();

ros::Publisher pub_position_3;
float u,v,x,y,z,R,P,Y,width,height,classification,confidence;
// brick br,br_up;
map<int,brick>::iterator it,it2;
support::msg_position m;

void signal_handler(int signal);


void detect_3(const Float64& msg){
    system("rm -r $(rospack find support)/yolo/yolov5/runs/detect/exp");
    system("rm -r $(rospack find support)/img_up.txt");
    //TODO: metti i comadi qua dentro e quiet

    cout<<"Sono in detect\n";
    system("rosservice call image_saver/save");
    cout<<"foto davanti scattata\n";
    
    lego3_1.clear();
    int j=0;
    cout<<"\n\n----CAMERA SOPRA----\n";
    Float64 ms;
    ms.data=1;
    pub_python.publish(ms);

    system("python3 $(rospack find support)/yolo/yolov5/detect.py --weights $(rospack find support)/yolo/yolov5/best.pt --img 640 --conf 0.45 --source $(rospack find support)/yolo/img.jpg --save-txt --save-conf");
    cout<<"analizzata foto davanti\n";

    
    

    ifstream infile;
    infile.open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/img_up.txt");
   
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
        cout<<"Y:"<<Y<<endl;
        cout<<"br_up::"<<br_up<<endl;
        // noti=br_up.noti();
        // br_up.setSide(noti);
        // if(noti==0){ //if noti==0: all side are known: correct or upside down
        lego3_1.insert(pair<int,brick> (j,br_up));
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
    
    j=0;
    
    infile.open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/yolo/yolov5/runs/detect/exp/labels/img.txt");



    infile>>classification;
    infile>>u;
    infile>>v;
    infile>>width;
    infile>>height;
    infile>>confidence;
    cout<<"----CAMERA DAVANTI----\n";

    while(infile.good()){
        cout<<"sono dentro\n";
        // Matrix<double,3,1> p;


        br.reset_front(u,v,width,height,classification,confidence);

        int l=((int)(br.getv()*img_height-1)*img_width) + (int)(br.getu()*img_width);
        // cout<<"l:"<<l<<endl;

        br.setx(-1.3+(ptc.points[l].z*cos(pitch) - ptc.points[l].y*sin(pitch)));
        br.sety(-ptc.points[l].x);

        int cl = br.getClass();
        if(cl==2 || cl==1)  br.setx(br.getx()+0.02);
        cout<<"br::"<<br;
        
        for(it=lego3_1.begin();it!=lego3_1.end();it++){
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
    
    for(it=lego3_1.begin();it!=lego3_1.end();it++){
        m.x.data=it->second.getx();
        m.y.data=it->second.gety();
        m.z.data=it->second.getz();
        m.R.data=it->second.getR();
        m.P.data=it->second.getP();
        m.Y.data=it->second.getY();
        m.classification.data=it->second.getClass();
        cout<<"Sto inviando: x:"<<m.x.data<<" y:"<<m.y.data<<" z:"<<m.z.data<<endl<<" Y:"<<m.Y.data<<" class:"<<m.classification.data<<endl;
        pub_position_3.publish(m);
        // cout<<m.classification.data<<"messaggio inviato\n";
    }
    cout<<"finito\n";   


}


void signal_handler(int signal){
    int j;
    if(signal==16){
        ifstream infile;
        infile.open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/img_up.txt");
    
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
            lego3_2.insert(pair<int,brick> (j,br_up));
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
        
        j=0;
        
        infile.open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/yolo/yolov5/runs/detect/exp/labels/img.txt");

        map<int,brick>::iterator it;

        infile>>classification;
        infile>>u;
        infile>>v;
        infile>>width;
        infile>>height;
        infile>>confidence;
        cout<<"----CAMERA DAVANTI----\n";

        while(infile.good()){
            cout<<"sono dentro\n";
            // Matrix<double,3,1> p;


            br.reset_front(u,v,width,height,classification,confidence);

            int l=((int)(br.getv()*img_height-1)*img_width) + (int)(br.getu()*img_width);
            // cout<<"l:"<<l<<endl;

            br.setx(-1.3+(ptc.points[l].z*cos(pitch) - ptc.points[l].y*sin(pitch)));
            br.sety(-ptc.points[l].x);

            int cl = br.getClass();
            if(cl==2 || cl==1)  br.setx(br.getx()+0.02);
            cout<<"br::"<<br;
            
            for(it=lego3_2.begin();it!=lego3_2.end();it++){
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

        
        for(it=lego3_1.begin();it!=lego3_1.end();it++){
            for(it2=lego3_2.begin();it2!=lego3_2.end();it2++){
                if(it2->second.getConf() > it->second.getConf()){
                    m.x.data=it2->second.getx();
                    m.y.data=it2->second.gety();
                    m.z.data=it2->second.getz();
                    m.R.data=it2->second.getR();
                    m.P.data=it2->second.getP();
                    m.Y.data=it2->second.getY();
                    m.classification.data=it2->second.getClass();
                    pub_position_3.publish(m);
                    pub_position.publish(m);
                } 
                else{
                    m.x.data=it->second.getx();
                    m.y.data=it->second.gety();
                    m.z.data=it->second.getz();
                    m.R.data=it->second.getR();
                    m.P.data=it->second.getP();
                    m.Y.data=it->second.getY();
                    m.classification.data=it->second.getClass();
                    pub_position.publish(m);
                }
            }
        }
        // support::msg_position m;
        // for(it=lego3_2.begin();it!=lego3_2.end();it++){
            // m.x.data=it->second.getx();
            // m.y.data=it->second.gety();
            // m.z.data=it->second.getz();
            // m.R.data=it->second.getR();
            // m.P.data=it->second.getP();
            // m.Y.data=it->second.getY();
            // m.classification.data=it->second.getClass();
        //     cout<<"Sto inviando: x:"<<m.x.data<<" y:"<<m.y.data<<" z:"<<m.z.data<<endl<<" Y:"<<m.Y.data<<" class:"<<m.classification.data<<endl;
        //     pub_position_3.publish(m);
        //     // cout<<m.classification.data<<"messaggio inviato\n";
        // }

    }
}