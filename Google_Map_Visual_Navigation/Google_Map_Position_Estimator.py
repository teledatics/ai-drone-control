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
import math

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
    def __init__(self, googleMapImg, Hx, Hy):
        self.grayGoogleMapImg = cv2.cvtColor(googleMapImg, cv2.COLOR_BGR2GRAY)
        self.currentPosition = None
        self.Rbn_t0 = None
        
        # HOG parameters
        self.cell_size = (32, 32)  # Size of each cell in pixels
        self.block_size = (64, 64)  # Size of each block in cells
        self.nbins = 9  # Number of histogram bins
        
        self.HOGLookupTable = CreateHOGLookupTable(self.grayGoogleMapImg, self.cell_size, self.block_size, self.nbins, Hx, Hy)
        
        # self.MapHogDescriptor, self.MapHog = GetHogDescriptor(self.grayGoogleMapImg, self.cell_size, self.block_size, self.nbins)
        # self.MapHogDescriptorFlattened = self.MapHogDescriptor.flatten()
    
    @classmethod
    def initGlobalLocalization(self, grayDroneImg, yawAngleRad, pitchAngleRad, rollAngleRad):
        # TODO: normalize image frame based on drone telemetry
        
        # Get 2D Fourier transform of input image
        f, _, _, _, _ = ZeroPadImg(grayDroneImg, self.grayGoogleMapImg.shape)
        f = f.astype(np.float32)
        f = PreProcess(f)
        F = np.fft.fft2(f)
        
        # Get complex conjugate of 2D Fourier transform of map
        h = self.grayGoogleMapImg.astype(np.float32)
        h = PreProcess(h)
        H_compConj = np.conjugate(np.fft.fft2(h)) # H*
        
        # Perform cross-correlation in the frequency domain (G = F ⊙ H*)
        cross_correlation = LinearMapping(np.fft.ifft2(F * H_compConj))
        
        # TODO: Determine if this step is necessary
        # Crop the result to the size of the larger image
        yPad = (cross_correlation.shape[0] - self.grayGoogleMapImg.shape[0]) // 2
        xPad = (cross_correlation.shape[1] - self.grayGoogleMapImg.shape[1]) // 2
        if yPad > 0:
            cross_correlation = cross_correlation[yPad : cross_correlation.shape[0] - yPad,:]
        if xPad > 0:
            cross_correlation = cross_correlation[:,xPad : cross_correlation.shape[1] - xPad]
        
        # Crop the result to the size of the larger image
        # if cross_correlation.shape != self.grayGoogleMapImg.shape:
        #     cross_correlation = cross_correlation[pad_top : cross_correlation.shape[0] - pad_bottom,
        #                                           pad_left : cross_correlation.shape[1] - pad_right]
        
        # Find max (i.e. current) position
        max_pos = np.where(cross_correlation == np.max(cross_correlation))
        self.currentPosition = max_pos[1], max_pos[0] # x,y
        
        # Get initial Rbn matrix at time of drone snapshot
        self.Rbn_t0 = self.__computeRbnMatrix(yawAngleRad, pitchAngleRad, rollAngleRad)
    
    ###################
    # Private Methods #
    ###################
    
    @classmethod
    def __computePointVelocity(self, translationVector, droneRotationalVelocityRads, x, y, Z, focalLength): # Z is altitude, typically from barometer
        Tx = translationVector[0, 0]
        Ty = translationVector[0, 1]
        Tz = translationVector[0, 2]
        ωx = droneRotationalVelocityRads[0, 0]
        ωy = droneRotationalVelocityRads[0, 1]
        ωz = droneRotationalVelocityRads[0, 2]
        
        vx = (Tz*x - Tx*focalLength)/Z - ωy*focalLength + ωz*y + (ωx*x*y - ωy*x**2)/focalLength
        vy = (Tz*y - Ty*focalLength)/Z + ωx*focalLength - ωz*x + (ωx*y**2 - ωy*x*y)/focalLength
        
        return vx, vy
    
    @classmethod
    def __computeTranslationVector(self, grayFrame1, grayFrame2, height, yawAngleRad, pitchAngleRad, rollAngleRad, Rbn_t0, prevPts = None):
        h = abs(height)
        H, newPts = self.__computeHomographyMatrix(grayFrame1, grayFrame2, prevPts)
        R, Rbn_t = self.__computeRotationMatrix(yawAngleRad, pitchAngleRad, rollAngleRad, Rbn_t0)
        N = self.__computeNormalVector(Rbn_t0)
        T = np.dot(h * (H - R), N)
        return T, newPts, Rbn_t
    
    @classmethod
    def __computeHomographyMatrix(self, grayFrame1, grayFrame2, prevPts = None):
        # Convert frames to grayscale
        # grayFrame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        # grayFrame2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        
        # Compute optical flow using Shi-Tomasi corner detector and [iterative] Lucas-Kanade method
        lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        if (prevPts is None):
            prevPts = cv2.goodFeaturesToTrack(grayFrame1, maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        newPts, status, err = cv2.calcOpticalFlowPyrLK(grayFrame1, grayFrame2, prevPts, None, **lk_params)
        
        # Filter and match key points
        goodPtsPrev = prevPts[status == 1]
        goodPtsNew = newPts[status == 1]
        
        # Estimate the homography matrix
        H, _ = cv2.findHomography(goodPtsPrev, goodPtsNew, cv2.RANSAC, 5.0)
        
        # Return the homography matrix and the new points for successive iterative calculations
        return H, goodPtsNew.reshape(-1, 1, 2)
    
    @classmethod
    def __computeNormalVector(self, Rbn_t0):
        e3 = np.array[0, 0, 1]
        # N = Rbn(t0) @ e3
        return np.dot(Rbn_t0, e3)
    
    @classmethod
    def __computeRotationMatrix(self, yawAngleRad, pitchAngleRad, rollAngleRad, Rbn_t0):
        Rbn_t = self.__computeRbnMatrix(yawAngleRad, pitchAngleRad, rollAngleRad)
        return Rbn_t @ np.transpose(Rbn_t0), Rbn_t
    
    @classmethod
    def __computeRbnMatrix(self, yawAngleRad, pitchAngleRad, rollAngleRad):
        # Compute yaw rotation matrix Rz(α)
        cosα_rad = np.cos(yawAngleRad)
        sinα_rad = np.sin(yawAngleRad)
        Rz_α = np.array([[cosα_rad, -sinα_rad, 0],
                         [sinα_rad, cosα_rad, 0],
                         [0, 0, 1]])
        
        # Compute pitch rotation matrix Ry(β)
        cosβ_rad = np.cos(pitchAngleRad)
        sinβ_rad = np.sin(pitchAngleRad)
        Ry_β = np.array([[cosβ_rad, 0, sinβ_rad],
                         [0, 1, 0],
                         [-sinβ_rad, 0, cosβ_rad]])
        
        # Compute roll rotation matrix Rx(γ)
        cosγ_rad = np.cos(rollAngleRad)
        sinγ_rad = np.sin(rollAngleRad)
        Rx_γ = np.array([[1, 0, 0],
                         [0, cosγ_rad, -sinγ_rad],
                         [0, sinγ_rad, cosγ_rad]])
        
        # Rnb = Rz(α)Ry(β)Rx(γ)
        Rnb = Rz_α @ Ry_β @ Rx_γ
        
        # Rbn = transpose(Rnb)
        return np.transpose(Rnb)



class ParticleFilter:
    def __init__(self,
                 num_particles,
                 HOGLookupTable,
                 mapShape,
                 σ,
                 τd):
        self.num_particles = num_particles
        self.HOGLookupTable = HOGLookupTable
        self.mapShape = mapShape
        self.σ = σ
        self.τd = τd
    
    @classmethod
    def run(self, currentFrameHOGDescriptorFlattened, searchSquareSideLength, searchInterval, xOffset, yOffset):
        # Initialize particles [randomly]
        particles = np.empty((self.num_particles, 2))
        particles[:, 0] = np.random.uniform(xOffset, searchSquareSideLength + xOffset, self.num_particles) // searchInterval * searchInterval
        particles[:, 1] = np.random.uniform(yOffset, searchSquareSideLength + yOffset, self.num_particles) // searchInterval * searchInterval
        
        # Get normalized distance weights
        distances = np.array([self.__distance(p, currentFrameHOGDescriptorFlattened) for p in particles])
        likelihoods = self.__gaussianLikelihood(distances)
        
        # Get minimum distance (highest likelihood)
        # TODO: Check that this implementation is not backwards
        minPosIndex = np.argmax(likelihoods)
        
        # Check minimum distance against threshold
        if (likelihoods[minPosIndex] >= self.τd):
            return (particles[minPosIndex, 0], particles[minPosIndex, 1])
        
        # If minimum distance violates threshold, return null
        return None
    
    @classmethod
    def __distance(self, particle, currentFrameHOGDescriptorFlattened):
        x_coordinate = particle[0]
        y_coordinate = particle[1]
        
        # Calculate Euclidean distance between reference and candidate descriptors
        candidateRegionDescriptor = self.HOGLookupTable[(x_coordinate, y_coordinate)]
        return np.linalg.norm(currentFrameHOGDescriptorFlattened - candidateRegionDescriptor)
    
    @classmethod
    def __gaussianLikelihood(self, distances):
        # Calculate Gaussian likelihoods
        likelihoods = (1/math.sqrt(2*math.pi*self.σ**2)) * np.exp(-0.5 * (distances**2) / (self.σ**2))
        
        # Normalize the likelihoods
        total_likelihood = np.sum(likelihoods)
        if total_likelihood != 0:
            likelihoods /= total_likelihood
        
        return likelihoods

#####################
# Utility Functions #
#####################

# pre-processing the image...
def PreProcess(img):
    # get the size of the img...
    height, width = img.shape
    img = np.log(img + 1)
    img = (img - np.mean(img)) / (np.std(img) + 1e-5)
    # use the hanning window...
    window = WindowFunc2d(height, width)
    img = img * window

    return img
    
def WindowFunc2d(height, width):
    win_col = np.hanning(width)
    win_row = np.hanning(height)
    mask_col, mask_row = np.meshgrid(win_col, win_row)

    win = mask_col * mask_row

    return win
    
def LinearMapping(img):
    return (img - img.min()) / (img.max() - img.min())
    
def ZeroPadImg(img, targetShape):
    # Get the current shape of the input image
    originalShape = img.shape
    
    # Compute the padding amounts for both dimensions (height and width)
    pad_height = targetShape[0] - originalShape[0]
    pad_width = targetShape[1] - originalShape[1]
    
    # Calculate the padding for each side (top, bottom, left, right)
    pad_top = pad_height // 2
    pad_bottom = pad_height - pad_top
    pad_left = pad_width // 2
    pad_right = pad_width - pad_left
        
    # Use openCV's copyMakeBorder to perform zero-padding
    paddedImg = cv2.copyMakeBorder(img, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=0)
        
    return paddedImg, pad_top, pad_bottom, pad_left, pad_right

def CreateHOGLookupTable(grayScale, cellSize, blockSize, nbins, Hx, Hy):
    
    hogLookupTable = {}
    height, width = grayScale.shape[:2]
    
    hog = cv2.HOGDescriptor(_winSize=(Hx // cellSize[1] * cellSize[1],
                                      Hy // cellSize[0] * cellSize[0]),
                            _blockSize=(blockSize[1] * cellSize[1],
                                        blockSize[0] * cellSize[0]),
                            _blockStride=(cellSize[1], cellSize[0]),
                            _cellSize=(cellSize[1], cellSize[0]),
                            _nbins=nbins)
    
    # Iterate over each pixel
    for y in range(height - Hy + 1):
        for x in range(width - Hx + 1):
            # Extract sub-image
            sub_img = grayScale[y:y+Hy, x:x+Hx]
            
            # Compute HOG descriptor
            descriptor = hog.compute(sub_img)
            hogLookupTable[(x, y)] = descriptor.flatten() # flatten for vector-based comparison

    return hogLookupTable
    
def GetHogDescriptor(grayScale, cellSize, blockSize, nbins):
    # Convert map to grayscale
    # grayScale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
    # Calculate HOG descriptors
    hog = cv2.HOGDescriptor(_winSize=(grayScale.shape[1] // cellSize[1] * cellSize[1],
                                      grayScale.shape[0] // cellSize[0] * cellSize[0]),
                            _blockSize=(blockSize[1] * cellSize[1],
                                        blockSize[0] * cellSize[0]),
                            _blockStride=(cellSize[1], cellSize[0]),
                            _cellSize=(cellSize[1], cellSize[0]),
                            _nbins=nbins)
    
    hogDescriptor = hog.compute(grayScale)
        
    # Normalize HOG features
    # hogDescriptor /= np.linalg.norm(hogDescriptor)
        
    return hogDescriptor, hog

# def ExtractSubImageHOG(hogDescriptorFlattened, full_image_shape, cell_size, x, y, h, w):
#     """
#     Extract the HOG descriptor of a sub-image.
    
#     WARNING: does not take into account cases where sub-image does not align perfectly with cell boundaries
#              and does not consider block normalization.

#     :param hog_descriptor: The HOG descriptor of the full image.
#     :param full_image_shape: The shape of the full image (height, width).
#     :param cell_size: The size of each cell (height, width).
#     :param x, y: The top-left coordinates of the sub-image.
#     :param h, w: The height and width of the sub-image.
#     :return: The HOG descriptor of the sub-image.
#     """

#     # Calculate the number of cells in the full image
#     num_cells_x = full_image_shape[1] // cell_size[1]

#     # Calculate the range of cells covered by the sub-image
#     start_cell_x = x // cell_size[1]
#     start_cell_y = y // cell_size[0]
#     end_cell_x = (x + w) // cell_size[1]
#     end_cell_y = (y + h) // cell_size[0]

#     # Extract the relevant portion of the HOG descriptor
#     # Note: The actual extraction depends on how the hog_descriptor is structured
#     sub_hog = []
#     for i in range(start_cell_y, end_cell_y):
#         for j in range(start_cell_x, end_cell_x):
#             cell_index = i * num_cells_x + j
#             cell_features = hogDescriptorFlattened[cell_index]  # Extract cell features
#             sub_hog.append(cell_features)

#     return np.concatenate(sub_hog)
    
def GetHogImage(hogDescriptor, imgShape, cellSize, blockSize, nbins, scale=1):
    # Reshape the HOG features to match the cell layout (num_cells_x, num_cells_y, num_blocks_x, num_blocks_y, num_bins)
    hogDescriptorReshaped = hogDescriptor.reshape((imgShape[0] // cellSize[0] - 1,
                                                   imgShape[1] // cellSize[1] - 1,
                                                   blockSize[1],
                                                   blockSize[0],
                                                   nbins))
    num_cells_x, num_cells_y, num_blocks_x, num_blocks_y, num_bins = hogDescriptorReshaped.shape
    cell_width = cellSize[0]
    cell_height = cellSize[1]
        
    # Create an empty image to draw the HOG visualization
    hogImage = np.zeros((num_cells_y * cell_height * scale, num_cells_x * cell_width * scale))
        
    for x in range(num_cells_x):
        for y in range(num_cells_y):
            for b in range(num_blocks_x):
                for a in range(num_blocks_y):
                    for angle in range(num_bins):
                        # Compute the center of the cell
                        cx = x * cell_width + cell_width // 2
                        cy = y * cell_height + cell_height // 2
                        
                        # Calculate the angle and magnitude of the gradient
                        angle_rad = (angle + 0.5) * np.pi / num_bins
                        magnitude = hogDescriptorReshaped[y, x, a, b, angle]
                        
                        # Calculate the endpoint of the gradient vector
                        x_endpoint = int(cx + magnitude * cell_width / 2 * np.cos(angle_rad))
                        y_endpoint = int(cy + magnitude * cell_height / 2 * np.sin(angle_rad))
                    
                        # Draw the arrow representing the gradient
                        cv2.line(hogImage, (cx * scale, cy * scale), (x_endpoint * scale, y_endpoint * scale), 255, 1)
    return hogImage