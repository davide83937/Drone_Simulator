import cv2
import numpy as np


def in_range_red(image):
    lower_red = np.array([0, 0, 150])
    upper_red = np.array([100, 100, 255])

    mask = cv2.inRange(image, lower_red, upper_red)
    return mask



def in_range_blue(image):
    # Range BGR per il blu:
    # Il primo valore (Blue) è alto, Green e Red sono bassi.
    lower_blue = np.array([120, 0, 0])
    upper_blue = np.array([255, 100, 100])

    mask = cv2.inRange(image, lower_blue, upper_blue)
    return mask
