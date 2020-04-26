import cv2
import numpy as np
import requests
import json


URL = 'http://localhost:5000/manometers/1'

response = requests.get(URL)

content = response.json()

IMG = content['img'].encode('latin1')
arr = np.fromstring(IMG, dtype=np.uint8).reshape((720, 1280, 3))
#arr = np.fromstring(IMG, np.uint8)
# print(arr.shape)
#frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
#print(arr.reshape(1280, 720))

cv2.imshow('Prediction', arr)

cv2.waitKey(0)
cv2.destroyAllWindows()
