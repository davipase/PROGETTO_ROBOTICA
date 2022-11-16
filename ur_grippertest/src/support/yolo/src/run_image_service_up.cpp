#include <iostream>
using namespace std;


int main(){
    system("rosrun image_view image_saver image:=/camera_up/color/image_raw _save_all_image:=false _filename_format:=$(rospack find support)/yolo/img_up.jpg __name:=image_saver_up");
}