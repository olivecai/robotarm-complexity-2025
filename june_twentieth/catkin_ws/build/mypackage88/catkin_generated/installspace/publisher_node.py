#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from mypackage88.msg import Position

def talk_to_me():
    # create publisher object, takes three arguments: the name of the topic, the type of message we are going to publish, the queue size
    pub = rospy.Publisher('talking_topic', Position, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True) #name of node, anon node? if create two nodes, both unique names
    rate = rospy.Rate(1) #set the rate in Hertz: tell ros how long to sleep
    rospy.loginfo("Publisher Node Started... Publishing messages...")
    while not rospy.is_shutdown():
        msg=Position()
        msg.message = "My Position is: " #populated data 
        msg.x = 2.0
        msg.y = 1.5
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterupptException:
        pass