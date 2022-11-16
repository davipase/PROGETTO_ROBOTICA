#!/usr/bin/env python3
import __init__
from cv_lib.src.object_detection3 import ObjectDetector
from cv_lib.src.camera_listener3 import CameraListener
from cv_ros.msg import ObjectPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Float64
import tf2_ros, rospy, torch, cv2
import numpy as np
from support.msg import msg_position

# publish topics list
topic_rs_status = '/RS_status'
topic_cv_status = '/CV_status'
topic_cv_data = '/CV_data'
topic_cv_obj = '/CV_object'
topic_lego_position = '/position_beta'


class PosePublisher():

    def __init__(self):


        # Init publisher
        self.pub = rospy.Publisher(topic_cv_data, ObjectPose, queue_size=10)
        self.pub_state = rospy.Publisher(topic_cv_status, String, queue_size=10)
        self.pub_obj = rospy.Publisher(topic_cv_obj, String, queue_size=10)
        self.pub_legos = rospy.Publisher(topic_lego_position, msg_position, queue_size=100)

        

        # Msg, buffer for camera to world transform, status
        self.msg = ObjectPose()
        self.cv_status = String()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.rate = rospy.Rate(10)


    def publish_status(self):
        # publish status
        self.pub_state.publish(self.cv_status)

    def publish_obj_detected(self, data):
        # publish number and type of detected object(s)
        self.pub_obj.publish(data)

    def set_status(self, str='STANDSTILL'):
        # set status
        self.cv_status.data = str


def run(data):
    # setup configuration
    publisher = PosePublisher()

    # start camera listener and detector
    camera = CameraListener()
    detector = ObjectDetector()
    publisher.set_status()

    # counters
    # i, j = 1,1
    # n = 1

    # utils
    # rs_status = String('STANDSTILL') # to refine

    print('Ready')

    # while not rospy.is_shutdown():
        # # Wait for Robotic station status
    # try:
    #     rs_status = rospy.wait_for_message(topic_rs_status, String, timeout=1)
    # except(rospy.exceptions.ROSException):
            # If we were already in the STANDSTILL setting, we stay in standstill
            # Othwerwise, we do nothing
        # if rs_status.data == 'STANDSTILL':
        #     pass

    # if rs_status.data == 'STANDSTILL':   
            # Take picture
    camera.get_frames()

        #detect object
    detector.set_picture(camera.bgr_image)
    found_object = detector.find_object(threshold=9000, verbose=False)

    msg=msg_position()
    f=open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/img_up.txt","w")
    # f.close()
    # f=open("/mnt/c/Users/davip/terzo_anno/Robotics/workspaces/PROGETTO_ROBOTICA/ur_grippertest/src/support/img_up.txt","a")
    for obj in found_object:
        # msg.x.data=obj[0][0]
        # msg.y.data=obj[0][1]
        # msg.z.data=0.965
        # msg.R.data=0
        # msg.P.data=0
        # msg.Y.data=obj[1]
        # msg.classification.data=0
        if (obj[0][0]<460 and obj[0][0]>175) and (obj[0][1]<380 and obj[0][1]>100):
            L=[str(obj[0][0])+" "+str(obj[0][1])+" "+str(0.95)+" "+str(0)+" "+str(0)+" "+str(-obj[1])+" "+str(obj[2])+" "+str(obj[3])+"\n"]
            print(L)
            f.writelines(L)
    f.close()
            

        #ti know when it finished publishig
        # msg.x.data=-2
        # publisher.pub_legos.publish(msg)

            # Find positions and orientations
            #detector.get_pos(camera, verbose=True)
            #detector.get_plane_orientation(camera, plot=True)

            #If we found any object
            #if found_object:
            #    # Publish state and number of flower detected
            #    publisher.set_status('SUCCESS')
            #    msg = str(poses['nb_detected']) + ' flower(s) detected'
            #    publisher.publish_obj_detected(msg)

    if not found_object:
        # publisher.set_status('RETRY')
        publisher.publish_obj_detected('None') 
    publisher.publish_status()
        # publisher.rate.sleep()


if __name__ == '__main__':
    #init node
    rospy.loginfo("init_node")
    rospy.init_node("Vision")
    rospy.Subscriber("/start_python",Float64,run)
    rospy.spin()
    # test()