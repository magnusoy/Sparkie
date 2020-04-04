
import rospy
from std_msgs.msg import String


class ROS_Subscriber:

    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
          
    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
    def run(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber(self.topic, self.msg_type, self.callback)
        rospy.spin()


sub = ROS_Subscriber('chatter', String)
if __name__ == "__main__":
    sub.run()