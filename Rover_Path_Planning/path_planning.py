#!/usr/bin/python3
"""@brief Defines ImageToGPSConverter class"""

##
# @file path_planning.py
#
# @brief Defines rover path planning module
#
# @section description_path_planning Description
# Defines classes which encapsulate methodology of rover path planning from drone aerial images.
# - ImageToGPSConverter
#
# @section libraries_path_planning Libraries/Modules
# - math standard library (https://docs.python.org/3/library/math.html)
#   - Access to essential trigonometric functions
#
# @section todo_path_planning TODO
# - None.
#
# @section author_path_planning Author(s)
# - Created by Justin Carrel on 02/26/2023
#
# @section references_path_planning References
# - https://blog.roboflow.com/georeferencing-drone-videos/
#
# Copyright (c) 2023 Teledatics. All rights reserved.

import math


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
