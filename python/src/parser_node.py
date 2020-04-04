#!/usr/bin/env python
import rospy
import math as m

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

def xbox_2_json(data):
    buttons = data.buttons
    axes = data.axes
    msg = {"0": axes[0], "1": axes[1], "2": axes[2], "3": axes[3], "4": axes[4], "5": axes[5],
    "6": buttons[0], "7": buttons[1], "8": buttons[2], "9": buttons[3], "10": buttons[4],
    "11": buttons[5], "12": buttons[6], "13": buttons[7], "14": buttons[8], "15": buttons[9],
    "16": buttons[10]}
    return msg

def odom_2_json(data):
    position = data.pose.pose.position
    orientation = data.pose.pose.orientation
    w = orientation.w
    x = -orientation.x
    y = orientation.y
    z = -orientation.z
    pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
    roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
    yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
    msg = {"x": position.x, "y": position.y, "z": position.z, "pitch": pitch, "roll": roll, "yaw": yaw}
    return msg

def callback0(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    msg = odom_2_json(data)
    #print(msg)

def callback1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    msg = xbox_2_json(data)
    #print(msg)


"""
def callback2(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


def callback3(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
"""

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/odom/sample", Odometry, callback0)
    rospy.Subscriber("/joy", Joy, callback1)
    #rospy.Subscriber("/controller/gui", Range, callback2)
    #rospy.Subscriber("/rviz/goal/out", Range, callback3)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s %s %s %s", sonar0,sonar1,sonar2,sonar3)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()