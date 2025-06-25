notes.md

# setting up visual servoing experiments on the kinova:

## setting up docker:

install docker, make Dockerfile and docker-compose.yml file

run `$ docker compose up` to start up the container

then open up a new terminal and run `$ docker exec -it <CONTAINER_NAME> bash`, where `-it` indicates INTERACTIVE TERMINAL

add the `--build--` flag when you make changes to the Dockerfile.

to give docker access to the screen, run `$ xhost +`

## setting up catkin package:

get inside dev container and call `$ catkin_make` in the catkin_ws.

`$ catkin_create_pkg` <package_name> <dependencies> to create a package

once inside the package, you can create messages, services, actions --> remember to edit the make file and the .xml file. 

hint : most likely need build_depend and exec_depend, lines 40 and 46, to be uncommented in the package.
hint : add the names of your messages, services, etc in the make file, and add the python install thing too 
hint : add the python shebang line at the top of your programs.

## connect to the kinova

use rospy, kortex_bringup to run basic commands on the kinova

kinova's ip address is: 192.168.1.10

set local static wired connection: 192.168.1.11

and ping 192.168.1.10 to see if kinova is responding.

can also search in browser for 192.168.1.10 --> control kinova!

## using openCV to get video capture and using optical flow LK tracking

`$ ls /dev` to view curent cameras: usually video0 is laptop, and even numbers are external cameras you want to find.

to play video: `$ ffplay /dev/video0`

overview of the openCV code on using cv2.calcOpticalFlowPyrLK():
1. set up capture using cv2.videoCapture(idx or path)
2. set up feature_params and lk_params
3. tak first frame and use cv.goodFeaturesToTrack to get the initial points
4. create a mask image for drawing purposes - same size as the read capture. get this from ret, old_frame = cap.read()
5. while true: get capture frame, calc optical flow, select good points, draw the tracks, update previous frame and previous points
6. destroyAllWindows

cole's process for getting visual servoing set up with ros:

*why convert opencv images to ros images?*

rqt lets you display image feeds in ros, but rqt NEEDS the ros image messages. 

*how to select points for vs?*

matplotlib ginput: ask for two clicks: click start and end on the interface, track from there.

tkinter
