import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# subscribed topics list
topic_camera_rgb = '/camera_up/color/image_raw'
topic_camera_depth = '/camera_up/depth/image_raw'
topic_camera_info = '/camera_up/depth/camera_info'


class CameraListener:

    def __init__(self):
        """
        Initializes camera
        """
        # frames
        self.bgr_image = None
        self.depth_frame = None
        self.bridge = CvBridge()
        self.error_mode = False
        # camera intrinsics
        self.intrinsics = 0

    def get_frames(self):
        """
        Gets depth and bgr frames
        """
        img = rospy.wait_for_message(topic_camera_rgb, Image)
        img_depth = rospy.wait_for_message(topic_camera_depth, Image)
        try:
            # convert to CV BGR image
            self.bgr_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")  # BGR format
            self.depth_frame = self.bridge.imgmsg_to_cv2(img_depth, img_depth.encoding)
        except CvBridgeError as e:
            self.error_mode = True
            print(e)
        except ValueError as e:
            self.error_mode = True
            print(e)

    def get_distance(self, cx, cy):
        """
        :param cx, cy: point **pixel** coordinates ( (cx, cy) = (u,v) = (y,x) array indexing
        :return: corresponding distance
        """
        z = self.depth_frame[cy, cx]
        return z  # y,x convention in opencv

    def image_2_camera(self, pixels, depth):
        """
        Converts pixel to camera coordinates:
        -> takes possible distortion into account only if point-wise (tuple)
        -> otherwise performs matrix multiplication without distortion (faster than pointwise).
        See test function in main_ros.py

        :param pixels: tuple (u,v) / array [u,v] OR ndarray of dim (N,2)
        :param depth: (from self.depth_frame[v,u]) OR ndarray of dim (N,)

        :return: 3d corresponding point [x,y,z] OR ndarray of dim (N, 3) in camera coordinates
        """
        # matrix multiplication
        if isinstance(pixels, np.ndarray):
            # we need two ndarrays
            assert(isinstance(depth, np.ndarray)), 'Depth must be an ndarray'
            # inverse projection matrix
            K = self.get_matrix()
            K_inv = np.linalg.inv(K)
            # Homogeneous representation (u,v,1) -> K_inv  @ -> (x', y', 1) -> * z -> (x, y, z)
            homogeneous_pixels = np.append(pixels, np.ones((len(pixels),1)), axis=1) # -> (N, 3)
            # matrix multiplication
            points_2d_homogeneous = homogeneous_pixels @ K_inv.T
            # followed by a point wise multiplication
            # I need depth of dimension (N,3) -> so add along axis 1 3 times (N,)
            points_3d = np.tile(depth, (3,1)).T * points_2d_homogeneous

            return points_3d


    def get_matrix(self):
        """
        :return: intrinsics matrix
        """
        if self.intrinsics:
            return np.array([[self.intrinsics.fx, 0, self.intrinsics.ppx],
                         [0, self.intrinsics.fy, self.intrinsics.ppy],
                         [0, 0, 1]])
        else:
            self.get_info()
            return np.array([[self.intrinsics.fx, 0, self.intrinsics.ppx],
                             [0, self.intrinsics.fy, self.intrinsics.ppy],
                              [0, 0, 1]])

