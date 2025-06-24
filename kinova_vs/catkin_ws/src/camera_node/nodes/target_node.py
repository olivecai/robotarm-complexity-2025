#!/usr/bin/python3

'''
Select a number of constraints to fulfill. For now, just choose one point-to-point constraint.

Get initial image and open in OpenCV

convert image from BGR to RGB
Use matplotlib ginput to click the END_EFFECTOR, and another to click the DESIRED_POSITION

get the ERROR. That is all we will do for now.
'''
import sys
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt

import time

print("Initializing camera node with Lucas-Kanade Optical Flow...")

# Initialize ROS node
rospy.init_node("camera_node", anonymous=True) #create a camera node
cam_idx = str(0)  #hardcode the camera idx

# Open video capture
cap = cv2.VideoCapture(cam_idx)
if not cap.isOpened():
    print("Error: Could not open video source.")
    exit()

ret, old_frame = cap.read() #obtain the opencv frame

#convert that frame from BGR to RGB
rgb_frame = cv2.cvtColor(old_frame, cv2.COLOR_BGR2RGB)

plt.imshow(rgb_frame)
plt.title("Click END EFFECTOR, then click DESIRED POINT, then CLOSE WINDOW")
plt.axis('on')
points = plt.ginput(2)
plt.close()

for i in range(len(points)):
    points[i]=np.array(points[i])
    print("Point",i,":", points[i])
    print("x:", points[i][0])
    print("y:", points[i][1])

error = points[1] - points[0]
print("ERROR:", error)

#now that we have the error, we should like to update our video input and error using the LK TRACKING. 
mask=np.zeros_like(old_frame)

#rename points to be points0, so that we have a sense of points0 and points1 during the tracking loop.
points0=np.array(points).astype(np.float32)
old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)

# Parameters for lucas kanade optical flow
lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

starttime=time.monotonic()
endtime = starttime+30
while time.monotonic() < endtime:
    ret, frame = cap.read()
    if not ret:
        print("No frames grabbed...")
        break
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #the trackers rely on changes in intensity, so it works better when grayscale

    #now calculate the optical flow of our points.
    points1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, points0, None, **lk_params)

    for i, (new, old) in enumerate(zip(points1, points0)):
        a,b = new.ravel()
        c,d = old.ravel()
        mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color=(0, 255, 0), thickness=10)
        frame = cv2.circle(frame, (int(a), int(b)), 5, (0, 255, 0), -1)
    img = cv2.add(frame, mask)

    cv2.imshow('frame', img)
    k = cv2.waitKey(30) & 0xff
    if k == 27:
        break
    # Now update the previous frame and previous points
    old_gray = frame_gray.copy()
    points0 = points1.copy()
    time.sleep(0.1)

cap.release()
cv2.destroyAllWindows