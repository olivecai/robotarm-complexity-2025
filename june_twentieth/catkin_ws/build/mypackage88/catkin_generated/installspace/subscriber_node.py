#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mypackage88.msg import Position

def callback(data):
    rospy.loginfo("%s X: %f Y: %f", data.message, data.x, data.y) #data is string object, get its 'data'

def listener():
    rospy.init_node("subscriber_node", anonymous=True)
    rospy.Subscriber('talking_topic', Position, callback) #topic name must match, type of message, callback function
    rospy.spin() #run the node continuously until we shut it down

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass