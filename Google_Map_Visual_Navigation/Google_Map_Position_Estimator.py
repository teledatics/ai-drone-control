#!/usr/bin/python3
"""@brief Defines GPS position estimation module based on drone image feed against onboard Google Map"""

##
# @file Google_Map_Position_Estimator.py
#
# @brief Defines GPS position estimation module based on drone image feed against onboard Google Map
#
# @section description_Google_Map_Position_Estimator Description
# Defines classes which encapsulate methodology of GPS position estimation from drone aerial images and onbaord Google Map data.
# - PositionEstimator
#
# @section libraries_Google_Map_Position_Estimator Libraries/Modules
# - OpenCV (https://docs.opencv.org/4.x/index.html)
#   - Access to computer vision and image processing algorithms
# - math standard library (https://docs.python.org/3/library/math.html)
#   - Access to essential trigonometric functions
# - Numpy (https://numpy.org/)
#   - Access to numerical computing tools for matrix calculations
# - sys (https://docs.python.org/3/library/sys.html)
#   - Access to system-specific parameters and functions
# - copy (https://docs.python.org/3/library/copy.html)
#   - Access to deep copy operations
# - random (https://docs.python.org/3/library/random.html)
#   - Access to random sampling functions
#
# @section todo_Google_Map_Position_Estimator TODO
# - 
#
# @section author_Google_Map_Position_Estimator Author(s)
# - Created by Justin Carrel on 10/15/2023
#
# @section references_Google_Map_Position_Estimator References
# - https://browse.arxiv.org/pdf/1703.10125.pdf
#
# Copyright © 2023 Teledatics. All rights reserved.

import numpy as np
import cv2

class PositionEstimator:
    """
    Position estimator class.

    Defines class which encapsulates methods for providing continuous GPS positon estimations via pattern recognition with
    drone telemetry, video feed, and an onboard Google Map reference image.
    """
    
    ##################
    # Public Methods #
    ##################
    
    """
    Constructor
    
    @name __init__
    """
    def __init__(self, googleMapImg):
        self.googleMapImg = googleMapImg
    
    @classmethod
    def initGlobalLocalization(self, droneImg):
        # TODO: normalize image frame based on drone telemetry
        f = cv2.cvtColor(droneImg, cv2.COLOR_BGR2GRAY)
        f = f.astype(np.float32)
        f = self.__pre_process(f)
        F = np.fft.fft2(f)
        
        h = cv2.cvtColor(self.googleMapImg, cv2.COLOR_BGR2GRAY)
        h = h.astype(np.float32)
        h = self.__pre_process(h)
        H_compConj = np.conjugate(np.fft.fft2(h)) # H*
        
        # NEXT: Need to create kernel based operation correlation method
        
        return
    
    ###################
    # Private Methods #
    ###################
    
    @classmethod
    # pre-processing the image...
    def __pre_process(self, img):
        # get the size of the img...
        height, width = img.shape
        img = np.log(img + 1)
        img = (img - np.mean(img)) / (np.std(img) + 1e-5)
        # use the hanning window...
        window = self.__window_func_2d(height, width)
        img = img * window

        return img
    
    @classmethod
    def __window_func_2d(self, height, width):
        win_col = np.hanning(width)
        win_row = np.hanning(height)
        mask_col, mask_row = np.meshgrid(win_col, win_row)

        win = mask_col * mask_row

        return win