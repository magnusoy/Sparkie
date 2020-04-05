import cv2
import numpy as np

import numpy as np
import cv2


def gradient_abs_value_mask(image, sobel_kernel=3, axis='x', threshold=(0, 255)):
    """
    Masks the image based on gradient absolute value.

    Parameters
    ----------
    image           : Image to mask.
    sobel_kernel    : Kernel of the Sobel gradient operation.
    axis            : Axis of the gradient, 'x' or 'y'.
    threshold       : Value threshold for it to make it to appear in the mask.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Take the absolute value of derivative in x or y given orient = 'x' or 'y'
    if axis == 'x':
        sobel = np.absolute(cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel))
    if axis == 'y':
        sobel = np.absolute(cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel))
    # Scale to 8-bit (0 - 255) then convert to type = np.uint8
    sobel = np.uint8(255 * sobel / np.max(sobel))
    # Create a mask of 1's where the scaled gradient magnitude is > thresh_min and < thresh_max
    mask = np.zeros_like(sobel)
    # Return this mask as your binary_output image
    mask[(sobel >= threshold[0]) & (sobel <= threshold[1])] = 1
    return mask

def gradient_magnitude_mask(image, sobel_kernel=3, threshold=(0, 255)):
    """
    Masks the image based on gradient magnitude.

    Parameters
    ----------
    image           : Image to mask.
    sobel_kernel    : Kernel of the Sobel gradient operation.
    threshold       : Magnitude threshold for it to make it to appear in the mask.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Take the gradient in x and y separately
    sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Calculate the magnitude
    magnitude = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
    # Scale to 8-bit (0 - 255) and convert to type = np.uint8
    magnitude = (magnitude * 255 / np.max(magnitude)).astype(np.uint8)
    # Create a binary mask where mag thresholds are met
    mask = np.zeros_like(magnitude)
    mask[(magnitude >= threshold[0]) & (magnitude <= threshold[1])] = 1
    # Return this mask as your binary_output image
    return mask

def gradient_direction_mask(image, sobel_kernel=3, threshold=(0, np.pi / 2)):
    """
    Masks the image based on gradient direction.

    Parameters
    ----------
    image           : Image to mask.
    sobel_kernel    : Kernel of the Sobel gradient operation.
    threshold       : Direction threshold for it to make it to appear in the mask.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Take the gradient in x and y separately
    sobel_x = cv2.Sobel(image, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobel_y = cv2.Sobel(image, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    # Take the absolute value of the x and y gradients and calculate the direction of the gradient
    direction = np.arctan2(np.absolute(sobel_y), np.absolute(sobel_x))
    # Create a binary mask where direction thresholds are met
    mask = np.zeros_like(direction)
    # Return this mask as your binary_output image
    mask[(direction >= threshold[0]) & (direction <= threshold[1])] = 1
    return mask

def color_threshold_mask(image, threshold=(0, 255)):
    """
    Masks the image based on color intensity.

    Parameters
    ----------
    image           : Image to mask.
    threshold       : Color intensity threshold.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    mask = np.zeros_like(image)
    mask[(image > threshold[0]) & (image <= threshold[1])] = 1
    return mask

def get_edges(image, separate_channels=False):
    """
    Masks the image based on a composition of edge detectors: gradient value,
    gradient magnitude, gradient direction and color.

    Parameters
    ----------
    image               : Image to mask.
    separate_channels   : Flag indicating if we need to put masks in different color channels.

    Returns
    -------
    Image mask with 1s in activations and 0 in other pixels.
    """
    # Convert to HLS color space and separate required channel
    hls = cv2.cvtColor(np.copy(image), cv2.COLOR_RGB2HLS).astype(np.float)
    s_channel = hls[:, :, 2]
    # Get a combination of all gradient thresholding masks
    gradient_x = gradient_abs_value_mask(s_channel, axis='x', sobel_kernel=3, threshold=(20, 100))
    gradient_y = gradient_abs_value_mask(s_channel, axis='y', sobel_kernel=3, threshold=(20, 100))
    magnitude = gradient_magnitude_mask(s_channel, sobel_kernel=3, threshold=(20, 100))
    direction = gradient_direction_mask(s_channel, sobel_kernel=3, threshold=(0.7, 1.3))
    gradient_mask = np.zeros_like(s_channel)
    gradient_mask[((gradient_x == 1) & (gradient_y == 1)) | ((magnitude == 1) & (direction == 1))] = 1
    # Get a color thresholding mask
    color_mask = color_threshold_mask(s_channel, threshold=(170, 255))

    if separate_channels:
        return np.dstack((np.zeros_like(s_channel), gradient_mask, color_mask))
    else:
        mask = np.zeros_like(gradient_mask)
        mask[(gradient_mask == 1) | (color_mask == 1)] = 1
        return mask

def flatten_perspective(image):
    """
    Warps the image from the vehicle front-facing camera mapping hte road to a bird view perspective.
    Parameters
    ----------
    image       : Image from the vehicle front-facing camera.
    Returns
    -------
    Warped image.
    """
    # Get image dimensions
    (h, w) = (image.shape[0], image.shape[1])
    # Define source points
    source = np.float32([[w // 2 - 76, h * .625], [w // 2 + 76, h * .625], [-100, h], [w + 100, h]])
    # Define corresponding destination points
    destination = np.float32([[100, 0], [w - 100, 0], [100, h], [w - 100, h]])
    transform_matrix = cv2.getPerspectiveTransform(source, destination)
    unwarp_matrix = cv2.getPerspectiveTransform(destination, source)
    return (cv2.warpPerspective(image, transform_matrix, (w, h)), unwarp_matrix)

"""
cap = cv2.VideoCapture(0)

while True:
    ret, image = cap.read()
    edges = get_edges(image)
    _, flatten = flatten_perspective(image)

    cv2.imshow("Original", image)
    #cv2.imshow("Edges", edges)
    cv2.imshow("Flatten", _)
    key = cv2.waitKey(1)
    
    if key == 27:
        break
cv2.destroyAllWindows()
"""


"""
import matplotlib.pyplot as plt


def unwarp(img, src, dst, testing):
    h, w = img.shape[:2]
    # use cv2.getPerspectiveTransform() to get M, the transform matrix, and Minv, the inverse
    M = cv2.getPerspectiveTransform(src, dst)
    # use cv2.warpPerspective() to warp your image to a top-down view
    warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)

    if testing:
        f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        f.subplots_adjust(hspace=.2, wspace=.05)
        ax1.imshow(img)
        x = [src[0][0], src[2][0], src[3][0], src[1][0], src[0][0]]
        y = [src[0][1], src[2][1], src[3][1], src[1][1], src[0][1]]
        ax1.plot(x, y, color='red', alpha=0.4, linewidth=3, solid_capstyle='round', zorder=2)
        ax1.set_ylim([h, 0])
        ax1.set_xlim([0, w])
        ax1.set_title('Original Image', fontsize=30)
        ax2.imshow(cv2.flip(warped, 1))
        ax2.set_title('Unwarped Image', fontsize=30)
        plt.show()
    else:
        return warped, M


im = cv2.imread("img.jpg")


(h, w) = (im.shape[0], im.shape[1])
# Define source points
source = np.float32([[w // 2 - 76, h * .625], [w // 2 + 76, h * .625], [-100, h], [w + 100, h]])
# Define corresponding destination points
destination = np.float32([[100, 0], [w - 100, 0], [100, h], [w - 100, h]])

unwarp(im, source, destination, True)

cv2.imshow("Image", im)
cv2.waitKey(0)
cv2.destroyAllWindows()
"""


cap = cv2.VideoCapture(0)

while True:
    ret, im = cap.read()
    (h, w) = (im.shape[0], im.shape[1])
    source = np.float32([[w // 2 - 76, h * .625], [w // 2 + 76, h * .625], [-100, h], [w + 100, h]])
    # Define corresponding destination points
    destination = np.float32([[100, 0], [w - 100, 0], [100, h], [w - 100, h]])

    matrix = cv2.getPerspectiveTransform(source, destination)
    result = cv2.warpPerspective(im, matrix, (w, h))

    cv2.imshow("Image", im)
    cv2.imshow("Perspective transformation", result)
    key = cv2.waitKey(1)
    
    if key == 27:
        break
cv2.destroyAllWindows()
