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
    
    NODE_1 = 0
    NODE_2 = 1
    NODE_3 = 2
    NODE_4 = 3
    NODE_5 = 4
    NODE_6 = 5
    NODE_7 = 6
    NODE_8 = 7
    NODE_9 = 8
    NODE_10 = 9
    NODE_11 = 10
    NODE_12 = 11
    NODE_13 = 12
    NODE_14 = 13
    NODE_15 = 14
    NODE_16 = 15
    NODE_17 = 16
    NODE_18 = 17
    NODE_19 = 18
    NODE_20 = 19
    
    # Simulate rover patrol path planning, Case 1
    nodes = [
        [-150.0, 50.0],  # NODE 1
        [110.0, -25.0],  # NODE 2
        [60.0, 150.0],   # NODE 3
        [-50.0, -100.0], # NODE 4
        [0.0, 0.0],      # NODE 5
        [10.0, -10.0],   # NODE 6
        [75.0, -75.0],   # NODE 7
        [-90.0, 160.0],  # NODE 8
        [-10.0, -40.0],  # NODE 9
        [150.0, -125.0]  # NODE 10
    ]
    numNodes = len(nodes)
    speeds = [0.1, 0.5, 1.0, 1.5]
    vi = [NODE_4, NODE_1, NODE_3] # 3 Rovers
    
    roverPatrolPathPlanner = CFMTSP()
    roverPatrolPathPlanner.initAdjacencyMatrix(numNodes)
    
    # Add edges with euclidean distances as weights
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_5,  math.dist(nodes[NODE_1],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_8,  math.dist(nodes[NODE_1],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_9,  math.dist(nodes[NODE_1],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_3,  math.dist(nodes[NODE_2],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_6,  math.dist(nodes[NODE_2],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_7,  math.dist(nodes[NODE_2],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_10, math.dist(nodes[NODE_2],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_2,  math.dist(nodes[NODE_3],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_5,  math.dist(nodes[NODE_3],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_8,  math.dist(nodes[NODE_3],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_9,  math.dist(nodes[NODE_4],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_10, math.dist(nodes[NODE_4],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_1,  math.dist(nodes[NODE_5],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_3,  math.dist(nodes[NODE_5],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_6,  math.dist(nodes[NODE_5],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_8,  math.dist(nodes[NODE_5],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_9,  math.dist(nodes[NODE_5],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_2,  math.dist(nodes[NODE_6],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_5,  math.dist(nodes[NODE_6],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_7,  math.dist(nodes[NODE_6],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_2,  math.dist(nodes[NODE_7],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_6,  math.dist(nodes[NODE_7],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_9,  math.dist(nodes[NODE_7],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_1,  math.dist(nodes[NODE_8],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_3,  math.dist(nodes[NODE_8],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_5,  math.dist(nodes[NODE_8],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_1,  math.dist(nodes[NODE_9],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_4,  math.dist(nodes[NODE_9],  nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_5,  math.dist(nodes[NODE_9],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_7,  math.dist(nodes[NODE_9],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_2,  math.dist(nodes[NODE_10], nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_4,  math.dist(nodes[NODE_10], nodes[NODE_4]))
    
    print("**********************")
    print("* SIMULATION CASE 1: *")
    print("**********************\n")
    
    selectedVertices, selectedSpeeds = roverPatrolPathPlanner.calculateRoverPaths(vi, speeds, Nm=0, β=1, gamma=1, evaporationRate=0.4, top=5.0,
                                                                                  alwaysSelectHighestProb=False, convergenceLimit = 10)
    
    if selectedVertices is None:
        print("No solution found.\n")
    else:
        for k in range(len(selectedVertices)):
            print("Path for rover " + str(k) + ": " + str(list(map(lambda i : i + 1, selectedVertices[k]))))
            print("Path speeds for rover " + str(k) + ": " + str(selectedSpeeds[k]) + "\n")
    
    # Simulate rover patrol path planning, Case 2
    nodes = [
        [-150.0, 50.0],   # NODE 1
        [110.0, -25.0],   # NODE 2
        [60.0, 150.0],    # NODE 3
        [-50.0, -100.0],  # NODE 4
        [0.0, 0.0],       # NODE 5
        [10.0, -10.0],    # NODE 6
        [75.0, -75.0],    # NODE 7
        [-90.0, 160.0],   # NODE 8
        [-10.0, -40.0],   # NODE 9
        [150.0, -125.0],  # NODE 10
        [-150.0, -125.0], # NODE 11
        [160.0, 60.0],    # NODE 12
        [100.0, 125.0],   # NODE 13
        [-100.0, 55.0],   # NODE 14
        [0, -175.0]       # NODE 15
    ]
    numNodes = len(nodes)
    speeds = [0.1, 0.5, 1.0, 1.5]
    vi = [NODE_4, NODE_1, NODE_3] # 3 Rovers
    
    roverPatrolPathPlanner = CFMTSP()
    roverPatrolPathPlanner.initAdjacencyMatrix(numNodes)
    
    # Add edges with euclidean distances as weights
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_5,  math.dist(nodes[NODE_1],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_8,  math.dist(nodes[NODE_1],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_9,  math.dist(nodes[NODE_1],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_11, math.dist(nodes[NODE_1],  nodes[NODE_11]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_14, math.dist(nodes[NODE_1],  nodes[NODE_14]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_3,  math.dist(nodes[NODE_2],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_6,  math.dist(nodes[NODE_2],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_7,  math.dist(nodes[NODE_2],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_10, math.dist(nodes[NODE_2],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_13, math.dist(nodes[NODE_2],  nodes[NODE_13]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_2,  math.dist(nodes[NODE_3],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_5,  math.dist(nodes[NODE_3],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_8,  math.dist(nodes[NODE_3],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_13, math.dist(nodes[NODE_3],  nodes[NODE_13]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_9,  math.dist(nodes[NODE_4],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_10, math.dist(nodes[NODE_4],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_11, math.dist(nodes[NODE_4],  nodes[NODE_11]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_15, math.dist(nodes[NODE_4],  nodes[NODE_15]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_1,  math.dist(nodes[NODE_5],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_3,  math.dist(nodes[NODE_5],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_6,  math.dist(nodes[NODE_5],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_8,  math.dist(nodes[NODE_5],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_9,  math.dist(nodes[NODE_5],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_2,  math.dist(nodes[NODE_6],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_5,  math.dist(nodes[NODE_6],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_7,  math.dist(nodes[NODE_6],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_2,  math.dist(nodes[NODE_7],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_6,  math.dist(nodes[NODE_7],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_9,  math.dist(nodes[NODE_7],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_1,  math.dist(nodes[NODE_8],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_3,  math.dist(nodes[NODE_8],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_5,  math.dist(nodes[NODE_8],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_14, math.dist(nodes[NODE_8],  nodes[NODE_14]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_1,  math.dist(nodes[NODE_9],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_4,  math.dist(nodes[NODE_9],  nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_5,  math.dist(nodes[NODE_9],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_7,  math.dist(nodes[NODE_9],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_2,  math.dist(nodes[NODE_10], nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_4,  math.dist(nodes[NODE_10], nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_12, math.dist(nodes[NODE_10], nodes[NODE_12]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_15, math.dist(nodes[NODE_10], nodes[NODE_15]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_11, NODE_1,  math.dist(nodes[NODE_11], nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_11, NODE_4,  math.dist(nodes[NODE_11], nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_11, NODE_15, math.dist(nodes[NODE_11], nodes[NODE_15]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_12, NODE_10, math.dist(nodes[NODE_12], nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_12, NODE_13, math.dist(nodes[NODE_12], nodes[NODE_13]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_13, NODE_2,  math.dist(nodes[NODE_13], nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_13, NODE_3,  math.dist(nodes[NODE_13], nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_13, NODE_12, math.dist(nodes[NODE_13], nodes[NODE_12]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_14, NODE_1,  math.dist(nodes[NODE_14], nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_14, NODE_8,  math.dist(nodes[NODE_14], nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_15, NODE_4,  math.dist(nodes[NODE_15], nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_15, NODE_10, math.dist(nodes[NODE_15], nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_15, NODE_11, math.dist(nodes[NODE_15], nodes[NODE_11]))
    
    print("**********************")
    print("* SIMULATION CASE 2: *")
    print("**********************\n")
    
    selectedVertices, selectedSpeeds = roverPatrolPathPlanner.calculateRoverPaths(vi, speeds, Nm=0, β=1, gamma=1, evaporationRate=0.4, top=5.0,
                                                                                  alwaysSelectHighestProb=False, convergenceLimit = 10)
    
    if selectedVertices is None:
        print("No solution found.\n")
    else:
        for k in range(len(selectedVertices)):
            print("Path for rover " + str(k) + ": " + str(list(map(lambda i : i + 1, selectedVertices[k]))))
            print("Path speeds for rover " + str(k) + ": " + str(selectedSpeeds[k]) + "\n")
    
    # Simulate rover patrol path planning, Case 3
    nodes = [
        [-150.0, 50.0],   # NODE 1
        [110.0, -25.0],   # NODE 2
        [60.0, 150.0],    # NODE 3
        [-50.0, -100.0],  # NODE 4
        [0.0, 0.0],       # NODE 5
        [10.0, -10.0],    # NODE 6
        [75.0, -75.0],    # NODE 7
        [-90.0, 160.0],   # NODE 8
        [-10.0, -40.0],   # NODE 9
        [150.0, -125.0],  # NODE 10
        [-150.0, -125.0], # NODE 11
        [160.0, 60.0],    # NODE 12
        [100.0, 125.0],   # NODE 13
        [-100.0, 55.0],   # NODE 14
        [0, -175.0],      # NODE 15
        [-175.0, -180.0], # NODE 16
        [0, 100.0],       # NODE 17
        [200.0, -180.0],  # NODE 18
        [200.0, 190.0],   # NODE 19
        [160.0, 140.0]    # NODE 20
    ]
    numNodes = len(nodes)
    speeds = [0.1, 0.5, 1.0, 1.5]
    vi = [NODE_4, NODE_1, NODE_3] # 3 Rovers
    
    roverPatrolPathPlanner = CFMTSP()
    roverPatrolPathPlanner.initAdjacencyMatrix(numNodes)
    
    # Add edges with euclidean distances as weights
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_4,  math.dist(nodes[NODE_1],  nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_5,  math.dist(nodes[NODE_1],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_8,  math.dist(nodes[NODE_1],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_9,  math.dist(nodes[NODE_1],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_11, math.dist(nodes[NODE_1],  nodes[NODE_11]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_1,  NODE_14, math.dist(nodes[NODE_1],  nodes[NODE_14]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_3,  math.dist(nodes[NODE_2],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_5,  math.dist(nodes[NODE_2],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_6,  math.dist(nodes[NODE_2],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_7,  math.dist(nodes[NODE_2],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_10, math.dist(nodes[NODE_2],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_12, math.dist(nodes[NODE_2],  nodes[NODE_12]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_2,  NODE_13, math.dist(nodes[NODE_2],  nodes[NODE_13]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_2,  math.dist(nodes[NODE_3],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_5,  math.dist(nodes[NODE_3],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_8,  math.dist(nodes[NODE_3],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_13, math.dist(nodes[NODE_3],  nodes[NODE_13]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_17, math.dist(nodes[NODE_3],  nodes[NODE_17]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_3,  NODE_19, math.dist(nodes[NODE_3],  nodes[NODE_19]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_1,  math.dist(nodes[NODE_4],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_7,  math.dist(nodes[NODE_4],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_9,  math.dist(nodes[NODE_4],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_10, math.dist(nodes[NODE_4],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_11, math.dist(nodes[NODE_4],  nodes[NODE_11]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_4,  NODE_15, math.dist(nodes[NODE_4],  nodes[NODE_15]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_1,  math.dist(nodes[NODE_5],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_2,  math.dist(nodes[NODE_5],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_3,  math.dist(nodes[NODE_5],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_6,  math.dist(nodes[NODE_5],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_8,  math.dist(nodes[NODE_5],  nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_9,  math.dist(nodes[NODE_5],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_14,  math.dist(nodes[NODE_5],  nodes[NODE_14]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_5,  NODE_17,  math.dist(nodes[NODE_5],  nodes[NODE_17]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_2,  math.dist(nodes[NODE_6],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_5,  math.dist(nodes[NODE_6],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_6,  NODE_7,  math.dist(nodes[NODE_6],  nodes[NODE_7]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_2,  math.dist(nodes[NODE_7],  nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_4,  math.dist(nodes[NODE_7],  nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_6,  math.dist(nodes[NODE_7],  nodes[NODE_6]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_9,  math.dist(nodes[NODE_7],  nodes[NODE_9]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_7,  NODE_10,  math.dist(nodes[NODE_7],  nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_1,  math.dist(nodes[NODE_8],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_3,  math.dist(nodes[NODE_8],  nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_5,  math.dist(nodes[NODE_8],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_14, math.dist(nodes[NODE_8],  nodes[NODE_14]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_8,  NODE_17, math.dist(nodes[NODE_8],  nodes[NODE_17]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_1,  math.dist(nodes[NODE_9],  nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_4,  math.dist(nodes[NODE_9],  nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_5,  math.dist(nodes[NODE_9],  nodes[NODE_5]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_9,  NODE_7,  math.dist(nodes[NODE_9],  nodes[NODE_7]))
    
    # TODO: Finish adding extra nodes
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_2,  math.dist(nodes[NODE_10], nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_4,  math.dist(nodes[NODE_10], nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_12, math.dist(nodes[NODE_10], nodes[NODE_12]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_10, NODE_15, math.dist(nodes[NODE_10], nodes[NODE_15]))
    
    roverPatrolPathPlanner.addUndirectedEdge(NODE_11, NODE_1,  math.dist(nodes[NODE_11], nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_11, NODE_4,  math.dist(nodes[NODE_11], nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_11, NODE_15, math.dist(nodes[NODE_11], nodes[NODE_15]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_12, NODE_10, math.dist(nodes[NODE_12], nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_12, NODE_13, math.dist(nodes[NODE_12], nodes[NODE_13]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_13, NODE_2,  math.dist(nodes[NODE_13], nodes[NODE_2]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_13, NODE_3,  math.dist(nodes[NODE_13], nodes[NODE_3]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_13, NODE_12, math.dist(nodes[NODE_13], nodes[NODE_12]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_14, NODE_1,  math.dist(nodes[NODE_14], nodes[NODE_1]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_14, NODE_8,  math.dist(nodes[NODE_14], nodes[NODE_8]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_15, NODE_4,  math.dist(nodes[NODE_15], nodes[NODE_4]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_15, NODE_10, math.dist(nodes[NODE_15], nodes[NODE_10]))
    roverPatrolPathPlanner.addUndirectedEdge(NODE_15, NODE_11, math.dist(nodes[NODE_15], nodes[NODE_11]))
    
    print("**********************")
    print("* SIMULATION CASE 3: *")
    print("**********************\n")
    
    selectedVertices, selectedSpeeds = roverPatrolPathPlanner.calculateRoverPaths(vi, speeds, Nm=0, β=1, gamma=1, evaporationRate=0.4, top=5.0,
                                                                                  alwaysSelectHighestProb=False, convergenceLimit = 10)
    
    if selectedVertices is None:
        print("No solution found.\n")
    else:
        for k in range(len(selectedVertices)):
            print("Path for rover " + str(k) + ": " + str(list(map(lambda i : i + 1, selectedVertices[k]))))
            print("Path speeds for rover " + str(k) + ": " + str(selectedSpeeds[k]) + "\n")

if __name__ == "__main__":
    main()
