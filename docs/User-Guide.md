# User Guide

This user guide assumes that you have general knowledge about Python, ROS, and linux systems.

The IP address of the robot might be different.

Check ROS_Jetson_Nano.md for more details.

## Robot

- Step 1: Turn on the robot

Plug in the batteries and make sure all the cabels are connected to the Jetson Nano.

- Step 2: Connect to robot

ssh into the robot with the following command:

```bash
ssh sparkie@10.0.0.16
password: qwerty
```

- Step 3: Start ROS on robot

```bash
export DISPLAY=0.0
./startup.sh
```

Use this command if anything freezes during execution:

```bash
killall bash
```

## Cloud computing

```bash
cd ~/sparkie/remote/python/src
start python main.py

 cd ~/sparkie/api
 export FLASK_APP=app.py
 flask run
```

## Graphical User Interface

- Step 1: Start navigation

Locate the ROS workspace and launch navigation.

```bash
export ROS_MASTER_URI=http://10.0.0.16:11311
export ROS_IP=10.0.0.121

cd ~/catkin_ws
source devel/setup.bash
roslaunch sparkie move_base.launch
```

Locate GUI and start it.

```bash
export ROS_MASTER_URI=http://10.0.0.16:11311

export ROS_IP=10.0.0.121
cd ~/sparkie/gui/src
python main.py
```
 



