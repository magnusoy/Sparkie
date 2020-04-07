# Setup ROS in GUI (Windows)

Follow [ROS Windows installation](http://wiki.ros.org/Installation/Windows) to have the same
folder structure.

ROS_MASTER_URI is the IP of the robot, change it accordingly.

ROS_IP is your own IP addressm change it accordingly.


Open a Terminal and run:

```bash
ipconfig

   Connection-specific DNS Suffix  . : home
   Link-local IPv6 Address . . . . . : fe80::904f:6d68:1c6a:d4cb%7
   IPv4 Address. . . . . . . . . . . : 10.0.0.121 (This is your ROS_IP address)
   Subnet Mask . . . . . . . . . . . : 255.255.255.0 
   Default Gateway . . . . . . . . . : 10.0.0.138 
```

If you are unsure about the ip address of ROS Master.

Go to your robot and run:

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
        inet 10.0.0.16  netmask 255.255.255.0  broadcast 10.0.0.255 (inet is your ROS_MASTER_URI)
        inet6 fe80::bcf0:857a:3fc8:fcc9  prefixlen 64  scopeid 0x20<link>
        ether 00:e1:8c:fd:70:d7  txqueuelen 1000  (Ethernet)
        RX packets 301388  bytes 22059366 (22.0 MB)
        RX errors 0  dropped 6  overruns 0  frame 0
        TX packets 326424  bytes 221193444 (221.1 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```


## Terminal 1

Run Rviz

```bash
set ROS_MASTER_URI=http://10.0.0.16:11311
set ROS_IP=10.0.0.121

rviz
```

## Terminal 2

Run Navigation

```bash
set ROS_MASTER_URI=http://10.0.0.16:11311
set ROS_IP=10.0.0.121
cd ..
cd ..
cd opt
cd catwin_ws
cd devel
setup.bash
cd ..

roslaunch sparkie move_base.launch
```