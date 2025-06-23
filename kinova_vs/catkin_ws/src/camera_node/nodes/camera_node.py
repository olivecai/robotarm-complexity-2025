#!/usr/bin/python3

'''
June 23 2025

Connect to camera and use cv_bridge to convert ros image to OpenCV image.
This node's responsibilities: get photo input.



print("Hello")
import sys
import rospy
import cv2
import cv_bridge
import numpy as np

rospy.init_node("camera_node", anonymous=True)
cam_param = rospy.search_param("cam_idx")
cam_idx =0
                
if cam_idx is None:
    print("Must pass camera index as _cam_idx:=<cam_idx>")
else:
    print("Camera Parameter:" ,cam_param)
    print("\n")
    print("Camera Index:", cam_idx)

#print(rospy.get_param_names())
print("\nGetting video feed...")
cap=cv2.VideoCapture(cam_idx)
if not cap.isOpened():
    print("Error: Could not open video source.")
    exit()

while True:
    # Read a frame from the video source
    ret, frame = cap.read()

    # Break the loop if the frame was not retrieved
    if not ret:
        print("End of video or error reading frame.")
        break

    # Display the frame
    cv2.imshow("Video", frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows
'''

#!/usr/bin/python3
import sys
import rospy
import cv2
import numpy as np

print("Initializing camera node with Lucas-Kanade Optical Flow...")

# Initialize ROS node
rospy.init_node("camera_node", anonymous=True)
cam_idx = 0  # Using direct index for camera

# Open video capture
cap = cv2.VideoCapture(cam_idx)
if not cap.isOpened():
    print("Error: Could not open video source.")
    exit()

# Parameters for ShiTomasi corner detection
feature_params = dict(maxCorners=100,
                      qualityLevel=0.3,
                      minDistance=7,
                      blockSize=7)

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Random colors for drawing
color = np.random.randint(0, 255, (100, 3))

# Read the first frame and find corners to track
ret, old_frame = cap.read()
if not ret:
    print("Failed to read initial frame.")
    cap.release()
    sys.exit(1)

old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

# Create mask for drawing
mask = np.zeros_like(old_frame)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        print("End of video or error reading frame.")
        break

    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Calculate Optical Flow
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    # Select good points
    if p1 is not None and st is not None:
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # Draw the tracks
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)), color[i].tolist(), 2)
            frame = cv2.circle(frame, (int(a), int(b)), 5, color[i].tolist(), -1)

        img = cv2.add(frame, mask)
    else:
        img = frame  # fallback

    cv2.imshow('Lucas-Kanade Optical Flow', img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break

    # Update previous frame and points
    old_gray = frame_gray.copy()
    if p1 is not None and st is not None:
        p0 = good_new.reshape(-1, 1, 2)

cap.release()
cv2.destroyAllWindows()
