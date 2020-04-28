#!/usr/bin/env python
import roslib
import rospy

from std_msgs.msg import UInt8
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *


rospy.init_node('movebase_client_py')
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
result_publisher = rospy.Publisher('goal_reached', UInt8, queue_size=1)
move_base.wait_for_server()


goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 0.86
#goal.target_pose.pose.position.y = 0.5
goal.target_pose.pose.orientation.w = 1.0


"""
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 1.31
goal.target_pose.pose.position.y = -2.25
goal.target_pose.pose.orientation.z = -0.364
goal.target_pose.pose.orientation.w = 0.935
"""

""" temp stop
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 2.273
goal.target_pose.pose.position.y = -4.722
goal.target_pose.pose.orientation.z = 0.000
goal.target_pose.pose.orientation.w = -0.538
"""

"""
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 3.032
goal.target_pose.pose.position.y = -5.517
goal.target_pose.pose.orientation.z = -0.316
goal.target_pose.pose.orientation.w = 0.949
"""

"""
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 1.789
goal.target_pose.pose.position.y = -6.490
goal.target_pose.pose.orientation.z = 0.999
goal.target_pose.pose.orientation.w = -0.037
"""



move_base.send_goal(goal)
wait = move_base.wait_for_result()

if not wait:
    rospy.logerr("Action server not available!")
    rospy.signal_shutdown("Action server not available!")
else:
    rospy.loginfo("Goal execution done!")
    result_publisher.publish(1)