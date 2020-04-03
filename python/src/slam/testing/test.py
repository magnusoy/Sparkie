from lidar import RPLidar
import time
import numpy as np

lidar = RPLidar(port='COM3')
lidar.connect()
print(lidar.get_health())
lidar.start_motor()
lidar.clear_input()
iterator = lidar.iter_scans(max_buf_meas=8000)
while True:
    scan = next(iterator)
    intens = np.array([meas[0] for meas in scan])
    print(len(intens))
lidar.stop_motor()
lidar.disconnect()