import cv2

img = cv2.imread('img/tmp/valvee.jpg', cv2.IMREAD_UNCHANGED)
dim = (640, 480)
resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

cv2.imwrite('img/tmp/valves.jpg', resized)