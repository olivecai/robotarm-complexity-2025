#!/usr/bin/python3

'''
June 23 2025

Connect to camera
PUBLISH to topic '/cameras/camid' the capture as a Ros Image every 0.1 second.
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
    def __init__(self):
        '''
        Publish OpenCV video capture 
        '''
        rospy.init_node("camera_node", anonymous=True) #anonymous since there may be multiple cameras
        
        cam_param = rospy.search_param("cam_id")
        self.cam_id=rospy.get_param(cam_param, None) #this is our identifier for our camera        
        
        if self.cam_id is None:
            rospy.logwarn("Must pass camera index as _cam_id:=<cam_id>")
            exit()
        
        rospy.delete_param(cam_param) # delete param so its needed for future runs.
        rospy.loginfo(f"Initialized Camera on topic /cameras/cam{self.cam_id}")
        
    
        self.vidcap = cv2.VideoCapture(self.cam_id)
        if not self.vidcap.isOpened():
            rospy.logerr("Error: Could not open video source.")
            exit()

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


def main(args):
    rospy.sleep(5) 
    rospy.loginfo("Starting a CameraNode ...")
    node = CameraNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Camera...")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



