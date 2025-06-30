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

from visualservo.msg import image_point, points_array

class ConsolidateTrackersNode:
    def __init__(self):
        rospy.init_node("consolidate_trackers_node", anonymous=False) #there should ONLY BE ONE
        
        cam_param = rospy.search_param("cams_list")
        self.cams_id_list=rospy.get_param(cam_param, None) #this is our identifier for our camera        
        
        if self.cams_id_list is None:
            rospy.logwarn("Must pass camera index as _cams_list:=\"[cam1_idx, cam2_idx, ...]\"")
            exit()
        
        rospy.delete_param(cam_param) # delete param so its needed for future runs.
        rospy.loginfo(f"Initialized Tracker on topic /cameras/consolidate_trackers{self.cams_id_list}")
        
        msg_filter_subs = [] #put the desired and current points in one subscriber 

        for i in range(len(self.cams_id_list)):
            cam_id = self.cams_id_list[i]
            cam_i_des_sub = message_filters.Subscriber(f"/cameras/cam{cam_id}/des_pnt", image_point, queue_size=10)
            cam_i_cur_sub = message_filters.Subscriber(f"/cameras/cam{cam_id}/curr_pnt", image_point, queue_size=10)
            msg_filter_subs.append(cam_i_des_sub)
            msg_filter_subs.append(cam_i_cur_sub)

        ts = message_filters.ApproximateTimeSynchronizer(msg_filter_subs, 10, 0.1, allow_headerless=True) 
        # so for one iteration with two cameras, the list of nodes should look like
        # [cam1_des_pub, cam1_cur_pub, cam2_des_pub, cam2_cur_pub]

        ts.registerCallback(self.callback)

        self.error_pub = rospy.Publisher("/visualservoing/errors", points_array, queue_size=10)
        self.cur_pos_pub = rospy.Publisher("/visualservoing/current_position", points_array, queue_size=10)

      
    def callback(self, *msgs): #for n cameras, we get 2n arguments
        # we need two things: the ERROR (des-cur) and the CURRENT 
        '''
        Receives 2n msgs: (des1, cur1, des2, cur2, ...)
        Computes errors for each camera: (des - cur)
        '''
        errors = []
        cur_cart_pos = []

        for i in range(0, len(msgs), 2):
            des_msg = msgs[i]
            cur_msg = msgs[i+1]

            err_x = des_msg.x - cur_msg.x #get the x error for one camera
            err_y = des_msg.y - cur_msg.y #get the y error for one camera
            errors.append(err_x)
            errors.append(err_y) # errors = [x_err for camera1, y_err for camera1, x_err for camera2, y_err for camera2, ...]
            
            cur_cart_pos.append(cur_msg.x)
            cur_cart_pos.append(cur_msg.y)

        rospy.loginfo(f"errors: {errors}")
        rospy.loginfo(f"current cartesian position: {cur_cart_pos}")

        self.error_pub.publish(errors)
        self.cur_pos_pub.publish(cur_cart_pos)

def main(args):
    rospy.sleep(5) 
    rospy.loginfo("Starting TrackingNode ...")
    node = ConsolidateTrackersNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down TrackingNode...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)