#!/usr/bin/python3
"""! @brief Exercises path_planning module classes"""

from path_planning import *
import math

def main():
    
    # Simulate image pixel coordinate to gps coordinate conversion
    image_width = 3840
    image_height = 2160
    pixelCoord = [image_width/4, image_height/4]
    droneCoord = [-117.1666879, 33.03300795]
    altitude = 394.028884
    droneFOV = 59
    compassHeading = 183.8

    gpsCoordinate = ImageToGPSConverter.getPixelGPSCoordinate(
        pixelCoord, image_width, image_height, droneCoord, altitude, droneFOV, compassHeading)
    print("GPS Coordinates: " + str(gpsCoordinate) + "\n")
    
    # Simulate rover patrol path planning
    nodes = [
        [150.0, -125.0], # NODE 0
        [-150.0, 50.0],  # NODE 1
        [110.0, -25.0],  # NODE 2
        [60.0, 150.0],   # NODE 3
        [-50.0, -100.0], # NODE 4
        [0.0, 0.0],      # NODE 5
        [-10.0, -10.0],  # NODE 6
        [75.0, -75.0],   # NODE 7
        [-110.0, 160.0], # NODE 8
        [-10.0, -40.0]   # NODE 9
    ]
    numNodes = len(nodes)
    speeds = [0.1, 0.5, 1.0, 1.5]
    vi = [4, 1, 3] # 3 Rovers
    
    roverPatrolPathPlanner = CFMTSP()
    roverPatrolPathPlanner.initAdjacencyMatrix(numNodes)
    
    # Add edges with euclidean distances as weights
    roverPatrolPathPlanner.addUndirectedEdge(0, 2, math.dist(nodes[0], nodes[2])) 
    roverPatrolPathPlanner.addUndirectedEdge(0, 4, math.dist(nodes[0], nodes[4]))
    roverPatrolPathPlanner.addUndirectedEdge(1, 5, math.dist(nodes[1], nodes[5]))
    roverPatrolPathPlanner.addUndirectedEdge(1, 8, math.dist(nodes[1], nodes[8]))
    roverPatrolPathPlanner.addUndirectedEdge(1, 9, math.dist(nodes[1], nodes[9]))
    roverPatrolPathPlanner.addUndirectedEdge(2, 0, math.dist(nodes[2], nodes[0]))
    roverPatrolPathPlanner.addUndirectedEdge(2, 3, math.dist(nodes[2], nodes[3]))
    roverPatrolPathPlanner.addUndirectedEdge(2, 6, math.dist(nodes[2], nodes[6]))
    roverPatrolPathPlanner.addUndirectedEdge(2, 7, math.dist(nodes[2], nodes[7]))
    roverPatrolPathPlanner.addUndirectedEdge(3, 2, math.dist(nodes[3], nodes[2]))
    roverPatrolPathPlanner.addUndirectedEdge(3, 5, math.dist(nodes[3], nodes[5]))
    roverPatrolPathPlanner.addUndirectedEdge(3, 8, math.dist(nodes[3], nodes[8]))
    roverPatrolPathPlanner.addUndirectedEdge(4, 0, math.dist(nodes[4], nodes[0]))
    roverPatrolPathPlanner.addUndirectedEdge(4, 9, math.dist(nodes[4], nodes[9]))
    roverPatrolPathPlanner.addUndirectedEdge(5, 1, math.dist(nodes[5], nodes[1]))
    roverPatrolPathPlanner.addUndirectedEdge(5, 3, math.dist(nodes[5], nodes[3]))
    roverPatrolPathPlanner.addUndirectedEdge(5, 6, math.dist(nodes[5], nodes[6]))
    roverPatrolPathPlanner.addUndirectedEdge(5, 8, math.dist(nodes[5], nodes[8]))
    roverPatrolPathPlanner.addUndirectedEdge(5, 9, math.dist(nodes[5], nodes[9]))
    roverPatrolPathPlanner.addUndirectedEdge(6, 2, math.dist(nodes[6], nodes[2]))
    roverPatrolPathPlanner.addUndirectedEdge(6, 5, math.dist(nodes[6], nodes[5]))
    roverPatrolPathPlanner.addUndirectedEdge(6, 7, math.dist(nodes[6], nodes[7]))
    roverPatrolPathPlanner.addUndirectedEdge(7, 2, math.dist(nodes[7], nodes[2]))
    roverPatrolPathPlanner.addUndirectedEdge(7, 6, math.dist(nodes[7], nodes[6]))
    roverPatrolPathPlanner.addUndirectedEdge(7, 9, math.dist(nodes[7], nodes[9]))
    roverPatrolPathPlanner.addUndirectedEdge(8, 1, math.dist(nodes[8], nodes[1]))
    roverPatrolPathPlanner.addUndirectedEdge(8, 3, math.dist(nodes[8], nodes[3]))
    roverPatrolPathPlanner.addUndirectedEdge(8, 5, math.dist(nodes[8], nodes[5]))
    roverPatrolPathPlanner.addUndirectedEdge(9, 1, math.dist(nodes[9], nodes[1]))
    roverPatrolPathPlanner.addUndirectedEdge(9, 4, math.dist(nodes[9], nodes[4]))
    roverPatrolPathPlanner.addUndirectedEdge(9, 5, math.dist(nodes[9], nodes[5]))
    roverPatrolPathPlanner.addUndirectedEdge(9, 7, math.dist(nodes[9], nodes[7]))
    
    selectedVertices, selectedSpeeds = roverPatrolPathPlanner.calculateRoverPaths(vi, speeds, Nm=0, β=1, gamma=1, evaporationRate=0.05, top=5.0)
    if selectedVertices is None:
        print("No solution found.")
    else:
        for k in range(len(selectedVertices)):
            print("\nPath for rover " + str(k) + ":")
            print(selectedVertices[k])
            print("Path speeds for rover " + str(k) + ":")
            print(selectedSpeeds[k])

if __name__ == "__main__":
    main()
