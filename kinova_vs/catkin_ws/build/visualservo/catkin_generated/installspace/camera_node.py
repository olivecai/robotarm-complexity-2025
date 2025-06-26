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
from std_msgs.msg import Int32

# create a camera node that publishes a ros message image every second.

class CameraNode:
    '''
    Each CAMERA in use should correspond to a CAMERA NODE. 
    Initialize a CameraNode object with argument as index of the camera.
    '''
    def __init__(self, cam_id):
        '''
        Publish OpenCV video capture 
        '''
        rospy.loginfo("Initializing camera_node"+str(cam_id))
        rospy.init_node("camera_node", anonymous=True) #anonymous since there may be multiple cameras
        self.vidcap = cv2.VideoCapture(cam_id)
        if not self.vidcap.isOpened():
            rospy.logerr("Error: Could not open video source.")
            exit()

        self.cam_id=cam_id #this is our identifier for our camera        
        self.bridge = cv_bridge.CvBridge()

        self.cap_pub = rospy.Publisher(f"/cameras/cam{self.cam_id}", Image, queue_size=10)
        self.id_pub = rospy.Publisher("/cam_id", Int32, queue_size=1)
        self.stop_capture = False



        #timer to repeatedly publish a capture from vidcap
        rospy.Timer(rospy.Duration(0.1), self.publish_capture_callback)
        
    def publish_capture_callback(self, event):
        '''
        Callback: publish a ROS Image Message from OpenCV,
        '''
        if self.stop_capture:
            return 
        
        print(f"publishing camera {self.cam_id} video capture...")
        self.id_pub.publish(Int32(data=self.cam_id))

        ret, frame = self.vidcap.read()
        if not ret:
            rospy.logwarn(f"Couldn't get image frame for camera {self.cam_id}")
            return
        
        #convert the BGR frame to a grayscale image to send through the ros message.
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rosImage = self.bridge.cv2_to_imgmsg(cvim=frame, encoding="mono8") #cv2_to_imgmsg returns a full Image message already
        rosImage.header.frame_id = f"{self.cam_id}"

        self.cap_pub.publish(rosImage)


if __name__ == '__main__':
    camera1 = CameraNode(0)
    rospy.spin()



