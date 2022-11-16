from skspatial.objects import Points, Plane
import cv2
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

"""
Object detection by HSV thresholding
"""

class ObjectDetector():

    def __init__(self):
        """Args: input image (BGR) from which we perform object detection"""
        
        self.objs = [] #list of object rotation 
        self.masks = [] #list of object wise masks
        self.frame = None
        self.detected_obj = [] #list of pixels positions of objects
        self.planes = [] #list of corresponding 3D planes
        self.coo = [] #objects (world) camera coordinates
        
    def reset(self):
        self.frame = None
        self.objs, self.masks, self.detected_obj, self.planes, self.coo = [], [], [], []

    def set_picture(self, img):
        self.frame = img
    
        
    
    def find_object(self, threshold = 2000, verbose = False): #OK
        """Stores object in image (pixels) coordinates and returns if found any
            Args: threshold for max area detection
        """
        #read image and copy
        if verbose: print("Finding object...")
        output = self.frame.copy()

        # convert to gray
        img_grey = cv2.cvtColor(output,cv2.COLOR_BGR2GRAY)
        #cv2.imshow("grey", img_grey)
        #cv2.waitKey()  
              
        # convert to binary - threshold -already invers
        ret,thresh_img = cv2.threshold(img_grey, 100, 255, cv2.THRESH_BINARY)
        #cv2.imshow("thres:img_grey",thresh_img)
        #cv2.waitKey() 
            
        # find contours
        contours_list , hierarchy = cv2.findContours(thresh_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for c in contours_list:
            x, y, w, h = cv2.boundingRect(c)
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            objs=[]
            if rect[1][0]<60:
                if rect[1][1]<60:
                    cv2.drawContours(output,[box],0,(0,0,255),1)
                    self.detected_obj.append([rect[0]])
                    center = rect[0]
                    rotation = rect[2]
                    l=center,rotation,rect[1][0],rect[1][1]
                    self.objs.append(l)
                    # print(center, rotation)        
        # cv2.imshow("bounding_box",output)
        # cv2.waitKey()

        if verbose:
            # Draw contours
            cv2.drawContours(output, contours_list, -1, (255, 0, 0), 2) # image, contours, contourIdx, color, thickness
            # cv2.imshow('object',output)
            # cv2.waitKey(0)
            cv2.destroyAllWindows()
            print("Done")
        return self.objs #check if objects centroid were found
    
        
        
