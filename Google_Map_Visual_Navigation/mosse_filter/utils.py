#!/usr/bin/python3
"""@brief Defines Mosse object tracking filter helper methods"""

##
# @file utils.py
#
# @brief Defines Mosse object tracking filter helper methods
#
# @section description_utils Description
# Defines Mosse object tracking filter algorithm helper methods.
#
# @section libraries_utils Libraries/Modules
# - Numpy (https://numpy.org/)
#   - Access to numerical computing tools for matrix calculations
# - OpenCV (https://docs.opencv.org/4.x/index.html)
#   - Access to computer vision and image processing algorithms
#
# @section author_utils Author(s)
# - Originally authored by TianhongDai 05/28/2018
# - Adapted by Justin Carrel on 10/15/2023
#
# @section references_mosse References
# - https://github.com/TianhongDai/mosse-object-tracking
#
# Copyright © 2023 Teledatics. All rights reserved.

import numpy as np
import cv2

# used for linear mapping...
def linear_mapping(img):
    return (img - img.min()) / (img.max() - img.min())

# pre-processing the image...
def pre_process(img):
    # get the size of the img...
    height, width = img.shape
    img = np.log(img + 1)
    img = (img - np.mean(img)) / (np.std(img) + 1e-5)
    # use the hanning window...
    window = window_func_2d(height, width)
    img = img * window

    return img

def window_func_2d(height, width):
    win_col = np.hanning(width)
    win_row = np.hanning(height)
    mask_col, mask_row = np.meshgrid(win_col, win_row)

    win = mask_col * mask_row

    return win

def random_warp(img):
    a = -180 / 16
    b = 180 / 16
    r = a + (b - a) * np.random.uniform()
    # rotate the image...
    matrix_rot = cv2.getRotationMatrix2D((img.shape[1]/2, img.shape[0]/2), r, 1)
    img_rot = cv2.warpAffine(np.uint8(img * 255), matrix_rot, (img.shape[1], img.shape[0]))
    img_rot = img_rot.astype(np.float32) / 255
    return img_rot


