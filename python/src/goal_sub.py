#! /usr/bin/python


import rospy
from std_msgs.msg import String, UInt8
import os


def result_callback(msg):
    print("Goal reached!")
    command = 'python image_saver_action.py'
    os.system(command)

def main():
    rospy.init_node('goal_listener', anonymous=True)
    rospy.Subscriber('goal_reached', UInt8, result_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

