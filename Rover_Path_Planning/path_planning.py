#!/usr/bin/python3
"""@brief Defines rover path planning module"""

##
# @file path_planning.py
#
# @brief Defines rover path planning module
#
# @section description_path_planning Description
# Defines classes which encapsulate methodology of rover path planning from drone aerial images.
# - CFMTSP
# - ImageToGPSConverter
#
# @section libraries_path_planning Libraries/Modules
# - math standard library (https://docs.python.org/3/library/math.html)
#   - Access to essential trigonometric functions
# - Numpy (https://numpy.org/)
#   - Access to numerical computing tools for matrix calculations
#
# @section todo_path_planning TODO
# - Test modules.
#
# @section author_path_planning Author(s)
# - Created by Justin Carrel on 02/26/2023
#
# @section references_path_planning References
# - https://www.researchgate.net/publication/342492385_Novel_Graph_Model_for_Solving_Collision-Free_Multiple-Vehicle_Traveling_Salesman_Problem_Using_Ant_Colony_Optimization
# - https://blog.roboflow.com/georeferencing-drone-videos/
#
# Copyright (c) 2023 Teledatics. All rights reserved.

import math
import numpy as np

class CFMTSP:
    """
    Collision-Free Multiple Traveling Salesman Problem class.

    Defines class which implements multiple traveling salesman solution for rovers.
    """
    
    """
    Constructor
    
    @name __init__
    """
    def __init__(self):
        return
    
    """
    Initialize adjacency square matrix with input number of nodes
    
    @name initAdjacencyMatrix
    @param {number} numNodes number of nodes in adjacency matrix
    @returns {ndarray} initialzed numNodes x numNodes adjacency matrix
    """
    @classmethod
    def initAdjacencyMatrix(self, numNodes):
        return np.zeros((numNodes, numNodes))
    
    """
    Add directed edge between nodes
    
    @name addDirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {ndarray} [mutable] adjacency matrix
    @param {number} weight node1 -> node2 edge weight
    """
    @classmethod
    def addDirectedEdge(self, node1, node2, adjMatrix, weight):
        if (adjMatrix == None):
            raise IndexError("Adjacency matrix is empty!")
        if (adjMatrix.shape[0] < node1):
            raise ValueError("node1 value out of bounds!")
        if (adjMatrix.shape[1] < node2):
            raise ValueError("node2 value out of bounds!")
        if (node1 == node2):
            raise ValueError("node1 and node2 value must not match!")
        if (weight == 0):
            raise ValueError("edge weight must be non-zero value!")
        
        # Mutable object
        adjMatrix[node1][node2] = weight
    
    """
    Add undirected edge between nodes
    
    @name addUndirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {ndarray} [mutable] adjacency matrix
    @param {number} weight node1 <-> node2 edge weight
    """
    @classmethod
    def addUndirectedEdge(self, node1, node2, adjMatrix, weight):
        # An undirected edge is just a bi-directional edge between nodes
        self.addDirectedEdge(node1, node2, adjMatrix, weight)
        self.addDirectedEdge(node2, node1, adjMatrix, weight)
    
    """
    Create edge matrix eB from adjacency matrix, via Algorithm 1 of CFMTSP paper
    
    @name createEdgeMatrix
    @param {ndarray} adjacency matrix
    @returns {ndarray} edge matrix
             {dictionary} edge end node look-up table
             {dictionary} edge start node look-up table
             {number} number of edges
    """
    @classmethod
    def createEdgeMatrix(self, adjMatrix):
        if (adjMatrix == None):
            raise IndexError("Adjacency matrix is empty!")
        
        eB = adjMatrix.copy() # make deep copy
        edgeEndDict = {}
        edgeStartDict = {}
        q = 0
        
        for i in range(eB.shape[0]): # rows
            for j in range(eB.shape[1]): # columns
                if (i != j) and (eB[i][j] != 0):
                    q += 1
                    eB[i][j] = q
                    edgeEndDict[q] = j + 1
                    edgeStartDict[q] = i + 1
                    
        return eB, edgeEndDict, edgeStartDict, q
    
    """
    Create trajectory adjacency matrix 𝜓B from edge matrix eB, via equation (7) of CFMTSP paper
    
    @name createTrajectoryAdjacencyMatrix
    @param {number} numEdges number of edges in edge matrix
    @param {number} numSpeeds number of discrete rover velocity settings
    @returns {ndarray} trajectory adjacency matrix
             {dictionary} row index look-up table
    """
    @classmethod
    def createTrajectoryAdjacencyMatrix(self, numEdges, numSpeeds):
        𝜓B = np.zeros((numEdges, numSpeeds))
        𝜓B_rowDict = {}
        
        for i in range(𝜓B.shape[0]): # rows
            for j in range(𝜓B.shape[1]): # columns
                𝜓p = i*numSpeeds + (j + 1)
                𝜓B[i][j] = 𝜓p
                𝜓B_rowDict[𝜓p] = i + 1
        
        return 𝜓B, 𝜓B_rowDict
    
    """
    Create augmented trajectory adjacency matrix 𝜉B from trajectory adjacency matrix 𝜓B and edge matrix eB, via Algorithm 2 of CFMTSP paper
    
    @name createAugmentedEdgeAdjacencyMatrix
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜓B_rowDict row index look-up table
    @param {dictionary} edgeEndDict end node look-up table
    @param {dictionary} edgeStartDict start node look-up table
    @returns {ndarray} augmented trajectory matrix
             {number} index count
             {dictionary} 𝜉h row look-up table
             {dictionary} 𝜉h column look-up table
    """
    @classmethod
    def createAugmentedEdgeAdjacencyMatrix(self, 𝜓B, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        if (𝜓B == None):
            raise IndexError("Trajectory adjacency matrix is empty!")
        
        # |𝜓B|
        num_elements = 𝜓B.shape[0] * 𝜓B.shape[1]
        
        # Initialize 𝜉B with zeros
        𝜉B = np.zeros((num_elements, num_elements))
        
        # Algorithm body
        𝜉h_rowDict = {}
        𝜉h_columnDict = {}
        𝜉h = h = 0
        for i in range(num_elements): # For i=1,...,|𝜓B|
            for j in range(num_elements): # For j=1,...,|𝜓B|
                c1 = 𝜓B_rowDict[i + 1]
                c2 = 𝜓B_rowDict[j + 1]
                if c1 != c2:
                    if edgeEndDict[c1] == edgeStartDict[c2]:
                        h += 1
                        # Update 𝜉b_ij
                        𝜉B[i][j] = h
                        𝜉h = h
                        𝜉h_rowDict[𝜉h] = i + 1
                        𝜉h_columnDict[𝜉h] = j + 1
        
        return 𝜉B, 𝜉h, 𝜉h_rowDict, 𝜉h_columnDict
    
    """
    Initialize pheromone adjacency matrix edges with default value
    
    @name createPheromoneAdjacencyMatrix
    @param {ndarray} augmented edge adjacency matrix
    @returns {ndarray} pheromone adjacency matrix
    """
    @classmethod
    def createPheromoneAdjacencyMatrix(self, 𝜉B):
        if (𝜉B == None):
            raise IndexError("Augmented edge adjacency matrix is empty!")
        
        τ = 𝜉B.copy() # make deep copy
        τ[τ != 0] = 0.5 # initialize non-empty edges
        
        return τ
    
    """
    Adds phermone along augmented route of pheromone matrix
    
    @name __calculatePheromoneTrailsAmount
    @param {array} L𝜉sel list of augmented edges traversed by ant
    @param {ndarray} τ pheromone matrix
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} adjMatrix adjacency matrix with edge weights
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    """
    @classmethod
    def __calculatePheromoneTrailsAmount(self, L𝜉sel, τ, speeds, adjMatrix, 𝜓B, 𝜉h_rowDict,
                                         𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        totalPathTime = self.__getPathTravelTime(speeds, adjMatrix, 𝜓B, L𝜉sel, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict,
                                                 edgeEndDict, edgeStartDict)
        for 𝜉h in L𝜉sel:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            
            # Mutable object
            τ[i - 1][j - 1] += 1/totalPathTime # Single ant of single species so we don't need to worry
                                               # about including delta tau of other ants in 1 iteration
                                               # TODO: Verify this is true
    
    """
    Reduces phermone along augmented route of pheromone matrix
    
    @name __reducePheromoneTrailAmount
    @param {array} L𝜉sel list of augmented edges traversed by ant
    @param {ndarray} τ pheromone matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {number} evaporationRate evaporation rate of pheromone along agumented edge
    """
    @classmethod
    def __reducePheromoneTrailAmount(self, L𝜉sel, τ, 𝜉h_rowDict, 𝜉h_columnDict, evaporationRate=0.0):
        for 𝜉h in L𝜉sel:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            
            # Mutable object
            τ[i - 1][j - 1] *= (1 - evaporationRate)
    
    """
    Calculate uniform acceleration along augmented edge
    
    @name __getAcceleration
    @param {number} si initial speed at node i
    @param {number} sj final speed at node j
    @param {number} Lij edge length between nodes i and j
    @returns {number} uniform acceleration from node i to node j
    """
    @classmethod
    def __getAcceleration(self, si, sj, Lij):
        return (sj**2 - si**2) / (2*Lij)
    
    """
    Calculate travel time along graph edge
    
    @name __getTravelTime
    @param {number} si initial speed at node i
    @param {number} sj final speed at node j
    @param {number} Lij edge length between nodes i and j
    @returns {number} travel time from node i to node j
    """
    @classmethod
    def __getTravelTime(self, si, sj, Lij):
        a_ij = self.__getAcceleration(si, sj, Lij)
        
        if (a_ij == 0):
            return Lij/si
        else:
            return (-si + math.sqrt(si**2 + 2*a_ij*Lij)) / a_ij
    
    """
    Calculate travel time along augmented edge
    
    @name __getEdgeTravelTime
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} adjMatrix adjacency matrix with edge weights
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {number} 𝜉h augmented edge index
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @returns {number} travel time for augmented edge indexed by 𝜉h
    """
    @classmethod
    def __getEdgeTravelTime(self, speeds, adjMatrix, 𝜓B, 𝜉h, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        i = 𝜉h_rowDict[𝜉h]
        j = 𝜉h_columnDict[𝜉h]
        c1_edge = 𝜓B_rowDict[i] # Edge 1
        c2_edge = 𝜓B_rowDict[j] # Edge 2
        si = speeds[(c1_edge - 1) % 𝜓B.shape[1]] # Speed defined by initial trajectory node in 𝜓B
        sj = speeds[(c2_edge - 1) % 𝜓B.shape[1]] # Speed defined by target trajectory node in 𝜓B
        Lij = adjMatrix[edgeStartDict[c1_edge]][edgeEndDict[c1_edge]] # Edge 1 weight/distance
        
        return self.__getTravelTime(si, sj, Lij)
    
    """
    Calculate travel time along augmented path
    
    @name __getPathTravelTime
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} adjMatrix adjacency matrix with edge weights
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {array} L𝜉sel array of augmented edge indexes
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @returns {number} travel time for augmented edge indexed by 𝜉h
    """
    @classmethod
    def __getPathTravelTime(self, speeds, adjMatrix, 𝜓B, L𝜉sel, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        totalDistance = 0
        
        for 𝜉h in L𝜉sel:
            totalDistance += self.__getEdgeTravelTime(speeds, adjMatrix, 𝜓B, 𝜉h, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
        
        return totalDistance


class ImageToGPSConverter:
    """
    Image conversion class.

    Defines class which encapsulates methodology of converting drone image pixel x-y cooridinates to GPS coordinates.
    """

    """
    Calculate GPS Coordinates of pixel on an image with given drone position, FOV, and compass heading
    
    @name getPixelGPSCoordinate
    @param {Array<number>} pixelCoord pixel x-y coordinate
    @param {number} imageWidth image pixel width
    @param {number} imageHeight image pixel height
    @param {Array<number>} droneCoord longitude and latitude of drone
    @param {number} droneAltitude drone altitude in feet
    @param {number} droneFOV drone camera field-of-view in degrees
    @param {number} droneCompassHeading drone compass heading in degrees
    @returns {Array<number>} calculated GPS coordinate
    """
    @staticmethod
    def getPixelGPSCoordinate(pixelCoord, imageWidth, imageHeight, droneCoord, droneAltitude, droneFOV, droneCompassHeading):
        pixel_x = pixelCoord[0]
        pixel_y = pixelCoord[1]

        droneAltitude *= 0.3048  # Convert from feet to meters

        fovRad = ImageToGPSConverter.__degreesToRadians(
            droneFOV)  # Convert to radians
        # Multiply by altitude to get distance across the image's diagonal
        fovAtan = math.tan(fovRad)

        # Calculate the ground distance shown (diagonal distance from top-left to bottom-right corner)
        diagonalDistance = droneAltitude * fovAtan

        # The direction the drone is pointed
        bearing = math.fmod(droneCompassHeading - 90, 360.0)

        # Change coordinate system so the center point of the image is (0, 0) (instead of the top-left point)
        # this means that (0, 0) is where our drone is and makes our math easier
        normalized = [pixel_y - imageHeight/2, pixel_x - imageWidth/2]

        # Calculate the distance and bearing of the pixel relative to the center point
        distanceFromCenterInPixels = math.sqrt(
            (imageWidth/2 - pixel_x)**2 + (imageHeight/2 - pixel_y)**2)
        diagonalDistanceInPixels = math.sqrt(imageWidth**2 + imageHeight**2)
        percentOfDiagonal = distanceFromCenterInPixels / diagonalDistanceInPixels
        distance = percentOfDiagonal * diagonalDistance  # In meters

        angle = math.atan(
            normalized[0] / (normalized[1] or 0.000001)) * 180 / math.pi
        # If the detection is in the right half of the frame we need to rotate it 180 degrees
        if normalized[1] >= 0:
            angle += 180

        options = {"units": "meters"}

        # Use that distance and bearing to get the GPS location of the pixel coordinate
        return ImageToGPSConverter.__rhumbDestination(droneCoord, distance, math.fmod(bearing + angle, 360.0), options)

    """
    Returns the destination coordinates having traveled the given distance along a Rhumb line from the 
    origin Point with the (varant) given bearing.
    
    @name  __rhumbDestination
    @param {Array<number>} origin starting point
    @param {number} distance distance from the starting point
    @param {number} bearing bearing angle ranging from -180 to 180 degrees from north
    @param {Object} [options={}] Optional parameters
    @param {string} [options.units='kilometers'] can be degrees, radians, miles, or kilometers
    @returns {Array<number>} Destination point.
    """
    @staticmethod
    def __rhumbDestination(origin, distance, bearing, options):
        if options == None:
            options = {"units": None}

        wasNegativeDistance = distance < 0
        distanceInMeters = ImageToGPSConverter.__convertLength(
            abs(distance), options["units"], "meters")
        if wasNegativeDistance:
            distanceInMeters = -abs(distanceInMeters)
        destination = ImageToGPSConverter.__calculateRhumbDestination(
            origin, distanceInMeters, bearing)

        # Compensate the crossing of the 180th meridian (https://macwright.org/2016/09/26/the-180th-meridian.html)
        # Solution from https://github.com/mapbox/mapbox-gl-js/issues/3250#issuecomment-294887678

        destination[0] += -360 if (destination[0] - origin[0] > 180) else (
            360 if (origin[0] - destination[0] > 180) else 0)

        return destination

    """
    Returns the destination point having travelled along a rhumb line from origin point the given distance on the given bearing.
    
    Adapted from Geodesy: http://www.movable-type.co.uk/scripts/latlong.html#rhumblines
    
    @name    __calculateRhumbDestination
    @param   {Array<number>} origin - point
    @param   {number} distance - Distance traveled, in same units as earth radius (default: metres).
    @param   {number} bearing - Bearing in degrees from north.
    @param   {number} [radius=6371e3] - (Mean) radius of earth (defaults to radius in metres).
    
    @returns {Array<number>} Destination point.
    """
    @staticmethod
    def __calculateRhumbDestination(origin, distance, bearing, radius=None):
        """
        φ => phi
        λ => lambda
        ψ => psi
        Δ => Delta
        δ => delta
        θ => theta
        """

        radius = ImageToGPSConverter.__earthRadius if radius == None else radius
        delta = distance / radius  # angular distance in radians
        # to radians, but without normalizing to 𝜋
        lambda1 = (origin[0] * math.pi) / 180
        phi1 = ImageToGPSConverter.__degreesToRadians(origin[1])
        theta = ImageToGPSConverter.__degreesToRadians(bearing)
        DeltaPhi = delta * math.cos(theta)
        phi2 = phi1 + DeltaPhi

        # Check if going past the pole, normalize latitude if so
        if abs(phi2) > (math.pi / 2):
            phi2 = (math.pi - phi2) if phi2 > 0 else (-math.pi - phi2)

        DeltaPsi = math.log(math.tan(phi2 / 2 + math.pi / 4) /
                            math.tan(phi1 / 2 + math.pi / 4))
        # E-W course becomes ill-conditioned with 0/0
        q = (DeltaPhi / DeltaPsi) if abs(DeltaPsi) > 10e-12 else math.cos(phi1)
        DeltaLambda = (delta * math.sin(theta)) / q
        lambda2 = lambda1 + DeltaLambda
        return [
            (((lambda2 * 180) / math.pi + 540) % 360) - 180,
            (phi2 * 180) / math.pi,
        ]  # normalise to −180..+180°

    """
    Converts a length to the requested unit. 
    
    Valid units: miles, nauticalmiles, inches, yards, meters, metres, kilometers, centimeters, feet
    
    @name  __convertLength
    @param {number} length to be converted
    @param {Units} [originalUnit="kilometers"] of the length
    @param {Units} [finalUnit="kilometers"] returned unit
    @returns {number} the converted length
    """
    @staticmethod
    def __convertLength(length, originalUnit, finalUnit):
        if originalUnit == None:
            originalUnit = "kilometers"
        if finalUnit == None:
            finalUnit = "kilometers"
        if not (length >= 0):
            raise ValueError("length must be a positive number")

        return ImageToGPSConverter.__radiansToLength(
            ImageToGPSConverter.__lengthToRadians(length, originalUnit),
            finalUnit
        )

    """
    Convert a distance measurement (assuming a spherical Earth) from radians to a more friendly unit. 
    
    Valid units: miles, nauticalmiles, inches, yards, meters, metres, kilometers, centimeters, feet
    
    @name  __radiansToLength
    @param {number} radians in radians across the sphere
    @param {string} [units="kilometers"] can be degrees, radians, miles, inches, yards, metres,
    meters, kilometres, kilometers.
    @returns {number} distance
    """
    @staticmethod
    def __radiansToLength(radians, units):
        if units == None:
            units = "kilometers"
        factor = ImageToGPSConverter.__factors[units]

        return radians * factor

    """
    Convert a distance measurement (assuming a spherical Earth) from a real-world unit into radians. 
    
    Valid units: miles, nauticalmiles, inches, yards, meters, metres, kilometers, centimeters, feet
    
    @name  __lengthToRadians
    @param {number} distance in real units
    @param {string} [units="kilometers"] can be degrees, radians, miles, inches, yards, metres,
    meters, kilometres, kilometers.
    @returns {number} radians
    """
    @staticmethod
    def __lengthToRadians(distance, units):
        if units == None:
            units = "kilometers"
        factor = ImageToGPSConverter.__factors[units]

        return distance / factor

    """! Converts an angle in degrees to radians
    
    @name  __degreesToRadians
    @param {number} degrees angle between 0 and 360 degrees
    @returns {number} angle in radians
    """
    @staticmethod
    def __degreesToRadians(degrees):
        radians = math.fmod(degrees, 360.0)
        return (radians * math.pi) / 180

    """
    Earth Radius used with the Harvesine formula and approximates using a spherical (non-ellipsoid) Earth.
    
    @memberof ImageToGPSConverter
    @type {number}
    """
    __earthRadius = 6371008.8

    """
    Unit of measurement factors using a spherical (non-ellipsoid) earth radius.
    
    @memberof ImageToGPSConverter
    @type {Object}
    """
    __factors = {
        "centimeters": __earthRadius * 100,
        "centimetres": __earthRadius * 100,
        "degrees": __earthRadius / 111325,
        "feet": __earthRadius * 3.28084,
        "inches": __earthRadius * 39.37,
        "kilometers": __earthRadius / 1000,
        "kilometres": __earthRadius / 1000,
        "meters": __earthRadius,
        "metres": __earthRadius,
        "miles": __earthRadius / 1609.344,
        "millimeters": __earthRadius * 1000,
        "millimetres": __earthRadius * 1000,
        "nauticalmiles": __earthRadius / 1852,
        "radians": 1,
        "yards": __earthRadius * 1.0936
    }

    """
    Units of measurement factors based on 1 meter.
    
    @memberof ImageToGPSConverter
    @type {Object}
    """
    __unitsFactors = {
        "centimeters": 100,
        "centimetres": 100,
        "degrees": 1 / 111325,
        "feet": 3.28084,
        "inches": 39.37,
        "kilometers": 1 / 1000,
        "kilometres": 1 / 1000,
        "meters": 1,
        "metres": 1,
        "miles": 1 / 1609.344,
        "millimeters": 1000,
        "millimetres": 1000,
        "nauticalmiles": 1 / 1852,
        "radians": 1 / __earthRadius,
        "yards": 1.0936133
    }

    """
    Area of measurement factors based on 1 square meter.
    
    @memberof ImageToGPSConverter
    @type {Object}
    """
    __areaFactors = {
        "acres": 0.000247105,
        "centimeters": 10000,
        "centimetres": 10000,
        "feet": 10.763910417,
        "hectares": 0.0001,
        "inches": 1550.003100006,
        "kilometers": 0.000001,
        "kilometres": 0.000001,
        "meters": 1,
        "metres": 1,
        "miles": 3.86e-7,
        "millimeters": 1000000,
        "millimetres": 1000000,
        "yards": 1.195990046
    }
