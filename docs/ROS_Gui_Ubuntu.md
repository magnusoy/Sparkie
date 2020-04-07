# Setup ROS in Ubuntu

ROS_MASTER_URI is the IP of the robot, change it accordingly.

ROS_IP is your own IP addressm change it accordingly.

Press CTRL+SHIFT+T to open a new tab in the terminal.

On your Ubuntu machine run:

```bash
ifconfig

eth0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 00:04:4b:e6:b3:95  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device interrupt 150  base 0xc000  

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1  (Local Loopback)
        RX packets 3587049  bytes 47025167018 (47.0 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 3587049  bytes 47025167018 (47.0 GB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

rndis0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 8a:ea:2a:0f:92:59  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 8a:ea:2a:0f:92:5b  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.0.0.9  netmask 255.255.255.0  broadcast 10.0.0.255 (inet is your ROS_IP)
        inet6 fe80::bcf0:857a:3fc8:fcc9  prefixlen 64  scopeid 0x20<link>
        ether 00:e1:8c:fd:70:d7  txqueuelen 1000  (Ethernet)
        RX packets 301388  bytes 22059366 (22.0 MB)
        RX errors 0  dropped 6  overruns 0  frame 0
        TX packets 326424  bytes 221193444 (221.1 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

On your Robot run:

```bash
ifconfig

eth0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 00:04:4b:e6:b3:95  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
        device interrupt 150  base 0xc000  

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1  (Local Loopback)
        RX packets 3587049  bytes 47025167018 (47.0 GB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 3587049  bytes 47025167018 (47.0 GB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

rndis0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 8a:ea:2a:0f:92:59  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

usb0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether 8a:ea:2a:0f:92:5b  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 10.0.0.16  netmask 255.255.255.0  broadcast 10.0.0.255 (inet is your ROS_MASTER_IP)
        inet6 fe80::bcf0:857a:3fc8:fcc9  prefixlen 64  scopeid 0x20<link>
        ether 00:e1:8c:fd:70:d7  txqueuelen 1000  (Ethernet)
        RX packets 301388  bytes 22059366 (22.0 MB)
        RX errors 0  dropped 6  overruns 0  frame 0
        TX packets 326424  bytes 221193444 (221.1 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

## Terminal Tab 1

List all topics and echo message

```bash
export ROS_MASTER_URI=https://10.0.0.16:11311
export ROS_IP=10.0.0.9

rostopic list

rostopic echo /topic_name
```

## Terminal Tab 2

Run Navigation

```bash
export ROS_MASTER_URI=https://10.0.0.16:11311
export ROS_IP=10.0.0.9

cd catwin_ws
source devel/setup.bash
roslaunch sparkie move_base.launch
```

## Terminal Tab 3 (Optional)

Run Teensy

```bash
export ROS_MASTER_URI=https://10.0.0.16:11311
export ROS_IP=10.0.0.9

rosrun rosserial_python serial_node.py /dev/ttyAMC0
```