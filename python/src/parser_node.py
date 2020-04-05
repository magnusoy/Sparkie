#!/usr/bin/env python
import rospy
import math as m
from communication.serial_handler import SerialThread
import json
from threading import Thread


from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty
from std_msgs.msg import String


global_data = ""
global_xbox_data = {}
global_odom_data = {}

pub = rospy.Publisher('/teensy/input', String, queue_size=1000)

"""
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_BAUDRATE = 921600
SERIAL_INTERVAL = 0.02

serial = SerialThread(port=SERIAL_PORT, baudrate=SERIAL_BAUDRATE)
serial.connect()
print(serial.isConnected())
"""

def xbox_2_dict(data):
    buttons = data.buttons
    axes = data.axes
    msg = {"0": axes[0], "1": axes[1], "2": axes[2], "3": axes[3], "4": axes[4], "5": axes[5],
    "6": buttons[0], "7": buttons[1], "8": buttons[2], "9": buttons[3], "10": buttons[4],
    "11": buttons[5], "12": buttons[6], "13": buttons[7], "14": buttons[8], "15": buttons[9],
    "16": buttons[10]}
    return msg

def odom_2_dict(data):
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
    global global_odom_data
    msg = odom_2_dict(data)
    global_odom_data = msg
    #print(msg)

def callback1(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    global global_xbox_data
    msg = xbox_2_dict(data)
    global_xbox_data = msg
    #print(msg)

def timer_callback(event):
    global global_data, global_odom_data, global_xbox_data
    global_data = {**global_odom_data, **global_xbox_data}
    #data = str(global_data).replace("'", '"')
    json_msg = json.dumps(global_data)
    #serial.sendOutputStream(json.loads(json_msg))
    data = json.loads(json_msg)
    rospy.loginfo("%s", data)
    pub.publish("Hello")
    

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/odom/sample", Odometry, callback0)
    rospy.Subscriber("/joy", Joy, callback1)
    
    timer = rospy.Timer(rospy.Duration(1.0), timer_callback)
    rospy.spin()
    timer.shutdown()

"""
def reader():
    while True:
        reply = serial.readInputStream()
        print(reply)
        
read_thread = Thread(target=reader, daemon=True)
"""

if __name__ == '__main__':
    #read_thread.start()
    listener()
