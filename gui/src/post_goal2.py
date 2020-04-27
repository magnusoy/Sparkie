#!/usr/bin/env python
import roslib
import rospy

from std_msgs.msg import String
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *


rospy.init_node('movebase_client_py')
move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
result_publisher = rospy.Publisher('goal_reached', String, queue_size=1)
move_base.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = 0.5
goal.target_pose.pose.orientation.w = 1.0

move_base.send_goal(goal)
wait = move_base.wait_for_result()

if not wait:
    rospy.logerr("Action server not available!")
    rospy.signal_shutdown("Action server not available!")
else:
    rospy.loginfo("Goal execution done!")
    result_publisher.publish('1')