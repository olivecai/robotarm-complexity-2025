#!/usr/bin/python3

'''
June 23 2025

Connect to camera
PUBLISH to topic '/cameras/camidx' the capture as a Ros Image every 0.1 second.
SUBSCRIBE to the topic 'visual_servoing', CALLBACK release camera and end capture.

'''

import sys
import rospy
import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image # ROS Image message 
from std_msgs.msg import String

# create a camera node that publishes a ros message image every second.

class CameraNode:
    '''
    Each CAMERA in use should correspond to a CAMERA NODE. 
    Initialize a CameraNode object with argument of the path or index of the camera.
    '''
    def __init__(self, cam_id):
        '''
        Publish OpenCV video capture 
        '''
        cam_id = str(cam_id)
        rospy.loginfo("Initializing camera_node"+str(cam_id))
        rospy.init_node("camera_node", anonymous=True) #anonymous since there may be multiple cameras
        self.vidcap = cv2.VideoCapture(cam_id)
        if not self.vidcap.isOpened():
            rospy.logerr("Error: Could not open video source.")
            exit()

        self.cam_id=cam_id #this is our identifier for our camera        
        self.bridge = cv_bridge.CvBridge()

        self.cap_pub = rospy.Publisher(f"/cameras/cam{self.cam_id}", Image, queue_size=10)
        self.id_pub = rospy.Publisher("/cam_id", String,

        #timer to repeatedly publish a capture from vidcap
        rospy.Timer(rospy.Duration(0.1), self.callback)
        
    def callback(self):
        '''
        Callback: publish a ROS Image Message from OpenCV,
        '''
        self.




