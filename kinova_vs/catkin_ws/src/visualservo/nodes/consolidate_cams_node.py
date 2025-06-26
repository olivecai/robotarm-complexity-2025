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

import message_filters
from sensor_msgs.msg import Image # ROS Image message 

from visualservo.msg import image_point

class ConsolidateTrackersNode:
    def __init__(self):
        rospy.init_node("consolidate_trackers_node", anonymous=False) #there should ONLY BE ONE
        
        cam_param = rospy.search_param("cams_list")
        self.cams_id_list=rospy.get_param(cam_param, None) #this is our identifier for our camera        
        
        if self.cams_list is None:
            rospy.logwarn("Must pass camera index as _cams_list:=<cams_list>")
            exit()
        
        rospy.delete_param(cam_param) # delete param so its needed for future runs.
        rospy.loginfo(f"Initialized Tracker on topic /cameras/consolidate_trackers{self.cams_list}")
        
        msg_filter_subs = [] #put the desired and current points in one subscriber 

        for i in range(len(self.cams_id_list)):
            cam_id = self.cams_id_list[i]
            cam_i_des_pub = message_filters.Subscriber(f"/cameras/cam{cam_id}/des_pnt", image_point, queue_size=10)
            cam_i_des_cur = message_filters.Subscriber(f"/cameras/cam{cam_id}/curr_pnt", image_point, queue_size=10)
            msg_filter_subs.append(cam_i_des_cur)
            msg_filter_subs.append(cam_i_des_pub)

        ts = message_filters.ApproximateTimeSynchronizer(msg_filter_subs, 10)

        ts.registerCallback(self.callback)

    def callback(what to put here?)

            