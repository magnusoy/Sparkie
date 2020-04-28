#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
import rospy
import sys
from std_msgs.msg import UInt8
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *


rospy.init_node('movebase_client_py', anonymous=True)
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
result_publisher = rospy.Publisher('goal_reached', UInt8, queue_size=1)
move_base.wait_for_server()
print('Connected to server')

num_goal_reached = sys.argv[1]

if num_goal_reached == '0':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.415
    goal.target_pose.pose.position.y = -0.031
    goal.target_pose.pose.orientation.z = -0.38
    goal.target_pose.pose.orientation.w = 0.999
    move_base.send_goal(goal)
    print("Sending 0 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '1':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 1.666
    goal.target_pose.pose.position.y = -0.555
    goal.target_pose.pose.orientation.z = -0.081
    goal.target_pose.pose.orientation.w = 0.997
    move_base.send_goal(goal)
    print("Sending 1 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '2':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.566
    goal.target_pose.pose.position.y = -0.502
    goal.target_pose.pose.orientation.z = 0.016
    goal.target_pose.pose.orientation.w = 1.000
    move_base.send_goal(goal)
    print("Sending 2 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '3':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 3.526
    goal.target_pose.pose.position.y = -0.865
    goal.target_pose.pose.orientation.z = 0.54
    goal.target_pose.pose.orientation.w = 0.999
    move_base.send_goal(goal)
    print("Sending 3 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '4':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 5.066
    goal.target_pose.pose.position.y = 0.128
    goal.target_pose.pose.orientation.z = 0.165
    goal.target_pose.pose.orientation.w = 0.986
    move_base.send_goal(goal)
    print("Sending 4 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '5':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 7.100
    goal.target_pose.pose.position.y = -0.227
    goal.target_pose.pose.orientation.z = -0.378
    goal.target_pose.pose.orientation.w = 0.926
    move_base.send_goal(goal)
    print("Sending 5 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '6':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 7.457
    goal.target_pose.pose.position.y = -1.349
    goal.target_pose.pose.orientation.z = -0.698
    goal.target_pose.pose.orientation.w = 0.716
    move_base.send_goal(goal)
    print("Sending 6 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)
elif num_goal_reached == '7':
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 7.457
    goal.target_pose.pose.position.y = -2.321
    goal.target_pose.pose.orientation.z = -0.698
    goal.target_pose.pose.orientation.w = 0.716
    move_base.send_goal(goal)
    print("Sending 7 goal")
    wait = move_base.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!")
        result_publisher.publish(1)