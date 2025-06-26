#!/usr/bin/python3

'''
June 25 2025

For each camera, have a tracker with subscribers initialized on that topic.

'''
import sys
import rospy
import cv2
import cv_bridge
import numpy as np
import matplotlib as plt

from sensor_msgs.msg import Image # ROS Image message 
from std_msgs.msg import Int32

from camera_node.msg import image_point

class TrackingNode:
    '''
    Currently only made for point to point, with constant position of cameras
    '''
    def __init__(self, cam_id):
        rospy.init_node('tracking_node', anonymous=True)
        self.bridge = cv_bridge.CvBridge()
        self.cam_id = cam_id
        rospy.Subscriber(f"/cameras/cam{self.cam_id}", Image, self.callback)

        self.lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        self.first_frame_recv = 0

        self.old_points = None
        self.new_points = None
        self.old_gray = None
        self.new_gray = None

        self.des_pub = rospy.Publisher(f"/cameras/cam{self.cam_id}/des_pnt", image_point, queue_size=10)
        self.curr_pub = rospy.Publisher(f"/cameras/cam{self.cam_id}/curr_pnt", image_point, queue_size=10)

    def callback(self, imgmsg):
        print("TrackingNode id:", self.cam_id, "--- initialized and listening")

        rospy.loginfo(str(self.cam_id) + " received image data")
        cv_image = self.bridge.imgmsg_to_cv2(imgmsg.data, "bgr8")
        
        if self.first_frame_recv == 0: #beginning of the visual servoing
            rgb_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            plt.imshow(rgb_frame)
            plt.title("Click END EFFECTOR, then click DESIRED POINT, then CLOSE WINDOW")
            plt.axis('on')
            points = plt.ginput(2)
            plt.close()

            print("OLD POINTS:", points)
            self.old_points = np.array(points)

            self.old_gray = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2GRAY)

            self.first_frame_recv = 1 #the first frame has been taken in

            #now we should exit this loop without publishing anything yet.
            rospy.loginfo("Visual servoing goals set.")

        else:
            #now we should publish the error, the tracked points, etc, and return
            self.new_gray = cv2.cvtColor(imgmsg.data, cv2.COLOR_BGR2GRAY)

            new_points, _, _ = cv2.calcOpticalFlowPyrLK(self.old_gray, self.new_gray, self.old_points, None, **self.lk_params)
            self.new_points = np.array(new_points)
            
            for i, (new, old) in enumerate(zip(self.new_points, self.old_points)):
                a,b = new.ravel()
                c,d = old.ravel()
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color=(0, 255, 0), thickness=10)
                frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)
            
            img = cv2.add(frame, mask)

            cv2.imshow('frame', img)

            # Now update the previous frame and previous points
            self.old_gray = self.new_gray.copy()
            self.old_points = self.new_points.copy()

            #now PUBLISH the desired point and the end effector point
            cur = points[0]
            des = points[1]

            des_pnt = image_point()
            des_pnt.x = des[0]
            des_pnt.y = des[1]

            cur_pnt = image_point()
            cur_pnt.x = cur[0]
            cur_pnt.y = cur[1]

            self.des_pub.publish(des_pnt)
            self.curr_pub.publish(cur_pnt)


                
            


        

        
        