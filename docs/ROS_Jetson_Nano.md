# Setup ROS in Jetson Nano

Press CTRL+SHIFT+T to open a new tab in the terminal.

## Terminal Tab 1

Run ROS Master

```bash
roscore
```

## Terminal Tab 2

Run Realsense SLAM

```bash
cd catwin_ws
source devel/setup.bash
roslaunch realsense2_camera rs_rtabmap.launch
```

## Terminal Tab 3

Run Joystick

```bash
cd ~
sudo chmod a+rw /dev/input/js0
Password: qwerty
rosparam set joy_node/dev "/dev/input/js0"
rosrun joy noy_node
```

## Terminal Tab 4

Run Teensy

```bash
rosrun rosserial_python serial_node.py /dev/ttyAMC0
```

## Terminal Tab 5 (Optional)

Run Navigation

```bash
cd catwin_ws
source devel/setup.bash
roslaunch sparkie move_base.launch
```