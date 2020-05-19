import numpy as np
import cv2 
import pytesseract

# get grayscale image
def get_grayscale(image):
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# noise removal
def remove_noise(image):
    return cv2.medianBlur(image,5)
 
#thresholding
def thresholding(image):
    return cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]

#dilation
def dilate(image):
    kernel = np.ones((5,5),np.uint8)
    return cv2.dilate(image, kernel, iterations = 1)
    
#erosion
def erode(image):
    kernel = np.ones((5,5),np.uint8)
    return cv2.erode(image, kernel, iterations = 1)

#opening - erosion followed by dilation
def opening(image):
    kernel = np.ones((5,5),np.uint8)
    return cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)

#canny edge detection
def canny(image):
    return cv2.Canny(image, 100, 200)

#skew correction
def deskew(image):
    coords = np.column_stack(np.where(image > 0))
    angle = cv2.minAreaRect(coords)[-1]
    if angle < -45:
        angle = -(90 + angle)
    else:
        angle = -angle
    (h, w) = image.shape[:2]
    center = (w // 2, h // 2)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(image, M, (w, h), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
    return rotated


def get_tag(img):
    custom_config = r'--oem 3 --psm 6'
    gray_img = get_grayscale(img)
    gray_img = deskew(gray_img)
    gray_img = erode(gray_img)
    result = pytesseract.image_to_string(gray_img, config=custom_config)
    return [x for x in result.split() if len(x) > 5]


# Exmaple of usage
if __name__ == "__main__":
    img = cv2.imread('PSV100-47.jpg')

    # Adding custom options
    custom_config = r'--oem 3 --psm 6'
    gray_img = get_grayscale(img)
    cv2.imwrite('ocr_gray.jpg', gray_img)
    gray_img = deskew(gray_img)
    cv2.imwrite('ocr_deskew.jpg', gray_img)
    gray_img = erode(gray_img)
    cv2.imwrite('ocr_erode.jpg', gray_img)
    result = pytesseract.image_to_string(gray_img, config=custom_config)
    print(result)
    print(result.split())
    print([x for x in result.split() if len(x) > 5])