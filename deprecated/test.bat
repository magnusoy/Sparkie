type sparkie.txt

@echo off

timeout 3 >nul
 

echo Starting Tracking camera
start "Tracking Camera" /min python python/src/tracking_camera_process.py
timeout 5 >nul


echo Starting Depth camera
start "Depth Camera" /min python python/src/depth_camera_process.py
timeout 2 >nul


echo Starting Serial communcation
start "Serial communcation" /min python python/src/serial_process.py
timeout 2 >nul


echo Starting Cloud service
start "Cloud service" /min python python/src/cloud_process.py
timeout 2 >nul

echo Internal server service
start "Internal server service" /min python python/src/gui_server.py


echo Pressing any key will close all of the above applications ...
pause 

taskkill /IM python.exe