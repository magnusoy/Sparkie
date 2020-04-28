#! /usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, UInt8
import os


def result_callback(data):
    print("Position reached!")
    command = 'python image_saver_action.py'
    os.system(command)

def main():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber('in_position', String, result_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

