#!/usr/bin/python3
"""@brief Defines GPS position estimation module based on drone image feed against onboard Google Map"""

##
# @file Google_Map_Position_Estimator.py
#
# @brief Defines GPS position estimation module based on drone image feed against onboard Google Map
#
# @section description_Google_Map_Position_Estimator Description
# Defines classes which encapsulate methodology of GPS position estimation from drone aerial images and onbaord Google Map data.
# - FOO
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