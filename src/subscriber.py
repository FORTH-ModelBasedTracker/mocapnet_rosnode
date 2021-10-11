#!/usr/bin/env python

#remove or add the library/libraries for ROS
import rospy, time, math, cv2, sys

#remove or add the message type
from std_msgs.msg import Float32MultiArray

#define function/functions to provide the required functionality
def mnet_new_pose_callback(msg):
    #make_something_here_with msg.data
    rospy.loginfo("I heard %s", msg.data)

if __name__=='__main__':
    #Add here the name of the ROS. In ROS, names are unique named.
    rospy.init_node('MocapNET ROS Python Template')
    #subscribe to a topic using rospy.Subscriber class "std_msgs/Float32MultiArray"
    sub=rospy.Subscriber('/mocapnet_rosnode/bvhFrame', Float32MultiArray , mnet_new_pose_callback)
    rospy.spin()
