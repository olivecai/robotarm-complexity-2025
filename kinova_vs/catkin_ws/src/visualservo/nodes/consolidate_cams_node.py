#!/usr/bin/python3

'''
June 26 2025

Consider this program a helper for the visual servoing node.
Consolidate our tracked points into one system of linear equations (dont include damping) to simply publish to our visual servoing node.
'''
import sys
import rospy
import cv2
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import Image # ROS Image message 

from visualservo.msg import image_point

class ConsolidateTrackersNode:
    def __init__(self):
        rospy.init_node("consolidate_trackers_node", anonymous=False) #there should ONLY BE ONE
        
        cam_param = rospy.search_param("cams_list")
        self.cams_list=rospy.get_param(cam_param, None) #this is our identifier for our camera        
        
        if self.cams_list is None:
            rospy.logwarn("Must pass camera index as _cams_list:=<cams_list>")
            exit()
        
        rospy.delete_param(cam_param) # delete param so its needed for future runs.
        rospy.loginfo(f"Initialized Tracker on topic /cameras/consolidate_trackers{self.cams_list}")
        