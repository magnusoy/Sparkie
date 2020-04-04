#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs import Joy


def callback0(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

def callback1(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

"""
def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.range)
    sonar2 = data.range


def callback3(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.range)
    sonar3 = data.range
"""

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/t265/odom/sample", Odometry, callback0)
    rospy.Subscriber("joy", Joy, callback1)
    #rospy.Subscriber("/controller/gui", Range, callback2)
    #rospy.Subscriber("/rviz/goal/out", Range, callback3)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s %s %s %s", sonar0,sonar1,sonar2,sonar3)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()