#!/usr/bin/python3
"""! @brief Exercises path_planning module classes"""

from path_planning import *

def main():
    
    image_width = 3840
    image_height = 2160
    pixelCoord = [image_width/4, image_height/4]
    droneCoord = [-117.1666879, 33.03300795]
    altitude = 394.028884
    droneFOV = 59
    compassHeading = 183.8
    
    gpsCoordinate = ImageToGPSConverter.getPixelGPSCoordinate(pixelCoord, image_width, image_height, droneCoord, altitude, droneFOV, compassHeading)
    for idx in gpsCoordinate:
        print(idx)

if __name__ == "__main__":
    main()