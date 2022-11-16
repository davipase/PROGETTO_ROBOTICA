#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <std_msgs/Float64.h>
#include <iterator>
#include <map>
using namespace std;

struct xyY{
    double x;
    double y;
    double Y;
}k;


int main(int argc, char* argv[]){

    ros::init(argc,argv,"start_assignment2");
    ros::NodeHandle nh;
    ros::Publisher pub=nh.advertise<std_msgs::Float64>("/start_detection",1);

    std_msgs::Float64 msg;
    msg.data=1;


    int num,j,i,c;
    cout<<"Inserire il numero di pezzi da generare (minimo 1, massimo 6) ==> ";
    cin>>num;

    srand(time(NULL));

    map<int,xyY> pezzi;

    //generating num unique pieces (unicity is enforced by the non-duplicate property of maps)
    while(pezzi.size()!=num){
        c=(int)rand()%11;
        pezzi.insert(pair<int,xyY> (c,k));
    }

    bool collision=false;

    //generating non-collident coordinates
    map<int,xyY>::iterator it,it2;
    for(it=pezzi.begin();it!=pezzi.end();it++){
        do{
            collision=false;
            k.x=rand()%5000+3000;
            k.x=-k.x/10000;
            cout<<"x:"<<k.x<<endl;

            k.y=rand()%6800-3400;
            k.y=-k.y/10000;
            cout<<"y:"<<k.y<<endl;

            k.Y=rand()%62830-31415;
            k.Y=k.Y/10000;
            cout<<"Y:"<<k.Y<<endl;

            for(it2=pezzi.begin();it2!=it;it2++){
                float dx=it2->second.x-k.x;
                float dy=it2->second.y-k.y;
                double d=sqrt(dx*dx + dy*dy);
                if(d<=0.15) collision=true;
            }
        }while(collision);
        it->second=k;
    }



    //set correct command for every piece
    for(it=pezzi.begin();it!=pezzi.end();it++){
        cout<<it->first<<" "<<it->second.x<<" "<<it->second.y<<endl;
        char command[200];
        
        switch(it->first){
            case 0:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/112/112.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 112",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 1:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/121/121.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 121",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 2:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/122/122.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 122",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 3:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/122doublearch/122doublearch.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 122doublearch",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 4:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/122slope/122slope.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 122slope",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 5:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/132/132.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 132",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 6:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/132arch/132arch.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 132arch",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 7:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/141/141.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 141",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 8:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/142/142.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 142",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 9:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/222/222.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 222",it->second.x,it->second.y,it->second.Y);
                break;
            }
            case 10:{
                sprintf(command,"rosrun gazebo_ros spawn_model -file $(rospack find support)/legos/legos_sdf/222arch/222arch.sdf -sdf -x %f -y %f -z 0.965 -Y %lf -model 222arch",it->second.x,it->second.y,it->second.Y);
                break;
            }
        }
        cout<<"!!"<<command<<endl;

        system(command);
    }

    
    pub.publish(msg);
    
    cout<<"pubblicato\n";
    return 0;

}