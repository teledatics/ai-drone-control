#!/usr/bin/python3
"""@brief Defines rover path planning module based on drone image feed"""

##
# @file path_planning.py
#
# @brief Defines rover path planning module based on drone image feed
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
# - sys (https://docs.python.org/3/library/sys.html)
#   - Access to system-specific parameters and functions
# - copy (https://docs.python.org/3/library/copy.html)
#   - Access to deep copy operations
# - random (https://docs.python.org/3/library/random.html)
#   - Access to random sampling functions
#
# @section todo_path_planning TODO
# - Modify CFMTSP solution to accomodate NON-uniform acceleration
# - Modify CFMTSP solution to account for speed limits needed to accomodate sharp turns; until then, keep max speed low
# - Modify CFMTSP solution to predict dynamic top value at each vertex depending on velocities and turn angle
# - Consider changing matrix data structures to adjacency lists instead of Numpy ndarrays
# - Test and verify pixel to GPS coordinate converter
#
# @section author_path_planning Author(s)
# - Created by Justin Carrel on 02/26/2023
# - Modified by Justin Carrel on 07/09/2023
#
# @section references_path_planning References
# - https://www.researchgate.net/publication/342492385_Novel_Graph_Model_for_Solving_Collision-Free_Multiple-Vehicle_Traveling_Salesman_Problem_Using_Ant_Colony_Optimization
# - https://blog.roboflow.com/georeferencing-drone-videos/
#
# Copyright © 2023 Teledatics. All rights reserved.

import math
import numpy as np
import sys
import copy
import random

class ShortestPaths:
    """
    Shortest paths class.

    Defines class which finds and stores shortest path information between all vertices on a graph.
    """
    
    ##################
    # Public Methods #
    ##################
    
    """
    Constructor
    
    @name __init__
    """
    def __init__(self):
        self.adjMatrix = None
        self.distMatrix = None
        self.predecessorMatrix = None
    
    """
    Initialize adjacency square matrix with input number of nodes
    
    @name initAdjacencyMatrix
    @param {number} numNodes number of nodes in adjacency matrix
    """
    @classmethod
    def initAdjacencyMatrix(self, numNodes: int):
        if (numNodes <= 0):
            raise ValueError("Number of nodes must be greater than zero")
        self.adjMatrix = np.full((numNodes, numNodes), sys.float_info.max, dtype=np.float64)
        np.fill_diagonal(self.adjMatrix, 0.0)
    
    """
    Checks if adjaceny matrix is initialized or not
    
    @name isAdjacencyMatrixEmpty
    @returns {boolean} True if adjacency matrix is null; False otherwise
    """
    @classmethod
    def isAdjacencyMatrixEmpty(self):
        return self.adjMatrix is None
    
    """
    Add undirected edge between nodes
    
    @name addUndirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {number} weight node1 <-> node2 edge weight
    """
    @classmethod
    def addUndirectedEdge(self, node1: int, node2: int, weight: np.float64):
        # An undirected edge is just a bi-directional edge between nodes
        self.__addDirectedEdge(node1, node2, weight)
        self.__addDirectedEdge(node2, node1, weight)
    
    """
    Check for existence of Hamiltonian Cycle using TSP Nearest Neighbor approximation algorithm
    
    @name hasHamiltonianCycle
    @param {number} numStartVertices Maximum number of vertices to try starting the search from
    @returns {boolean} True if Hamiltonian circuit is found within adjacency matrix; False otherwise
    """
    @classmethod
    def hasHamiltonianCycle(self, numStartVertices):
        if self.adjMatrix is None:
            raise IndexError("Adjacency matrix is uninitialized!")
        
        # Since this uses an approximation algorithm, we have no guarantee that an existing cycle will be found from a particular starting node
        # even if a cycle definitely exists. Increase our chances by choosing a large sample of starting vertices to try.
        num_vertices = self.adjMatrix.shape[0]
        start_vertices = range(num_vertices) if num_vertices <= numStartVertices else random.sample(range(num_vertices), numStartVertices)
        
        # Try all vertices as starting points
        for start_vertex in start_vertices:
            
            visited = [False] * num_vertices
            path = []
            
            path.append(start_vertex)
            visited[start_vertex] = True
        
            while len(path) < num_vertices:
                current_vertex = path[-1]
                nearest_vertex = None
                min_distance = sys.float_info.max
            
                # Find the nearest unvisited vertex
                for v in range(num_vertices):
                    if not visited[v] and self.adjMatrix[current_vertex][v] < min_distance:
                        nearest_vertex = v
                        min_distance = self.adjMatrix[current_vertex][v]
            
                if nearest_vertex is not None:
                    path.append(nearest_vertex)
                    visited[nearest_vertex] = True
                else:
                    # No unvisited neighbor from current vertex
                    break
            # END while len(path) < num_vertices:
            
            if len(path) == num_vertices:
                return True
        # END for start_vertex in range(num_vertices):
        
        return False
    
    """
    Compute shortest paths between all vertices
    
    @name computeShortestPaths
    @param {boolean} hamiltonianCycleExists flag to indicate if Hamiltonian cycles exist in the graph
    """
    @classmethod
    def computeShortestPaths(self, hamiltonianCycleExists):
        if self.adjMatrix is None:
            raise IndexError("Adjacency matrix is uninitialized!")
        
        # Floyd Warshall Algorithm
        
        # Initialize distance and predecessor matrices
        self.distMatrix = self.adjMatrix.copy() # Deep copy
        self.predecessorMatrix = np.full((self.adjMatrix.shape[0], self.adjMatrix.shape[1]), 0, dtype=int)
        for index, _ in np.ndenumerate(self.predecessorMatrix):
            self.predecessorMatrix[index[0]][index[1]] = index[0]
        
        # If Hamiltonian cycles exist; shortcut the calculations
        if hamiltonianCycleExists:
            return
        
        # Add vertices individually
        for k in range(self.distMatrix.shape[0]):
            for i in range(self.distMatrix.shape[0]):
                for j in range(self.distMatrix.shape[0]):
                    if ((self.distMatrix[i][k] != sys.float_info.max) and # Help avoid arithmetic overflow
                        (self.distMatrix[k][j] != sys.float_info.max) and
                        ((self.distMatrix[i][k] + self.distMatrix[k][j]) < self.distMatrix[i][j])):
                        self.distMatrix[i][j] = self.distMatrix[i][k] + self.distMatrix[k][j]
                        self.predecessorMatrix[i][j] = self.predecessorMatrix[k][j]
    
    """
    Retrieves reference to computed distance matrix
    
    @name getDistanceMatrix
    @returns {ndarray} reference to distance matrix
    """
    @classmethod
    def getDistanceMatrix(self):
        return self.distMatrix
    
    """
    Retrieve shortest path between 2 vertices, reversed
    
    @name getShortestPath
    @param {number} node1 start vertex
    @param {number} node2 end vertex
    @returns {array} list of vertices in shortest path
    """
    @classmethod
    def getShortestPath(self, node1: int, node2: int):
        if self.predecessorMatrix is None:
            raise IndexError("Predecessor matrix is uninitialized!")
        
        path = []
        while node1 != node2:
            path.append(node2)
            node2 = self.predecessorMatrix[node1][node2]
        path.append(node1)
        
        return path
    
    ###################
    # Private Methods #
    ###################
    
    """
    Add directed edge between nodes
    
    @name addDirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {number} weight node1 -> node2 edge weight
    """
    @classmethod
    def __addDirectedEdge(self, node1: int, node2: int, weight: np.float64):
        if self.adjMatrix is None:
            raise IndexError("Adjacency matrix is uninitialized!")
        if self.adjMatrix.shape[0] <= node1:
            raise ValueError("node1 value out of bounds!")
        if self.adjMatrix.shape[1] <= node2:
            raise ValueError("node2 value out of bounds!")
        if node1 == node2:
            raise ValueError("node1 and node2 value must not match!")
        if weight == 0.0:
            raise ValueError("edge weight must be non-zero value!")
        
        # Mutable object
        self.adjMatrix[node1][node2] = weight

class CFMTSP:
    """
    Collision-Free Multiple Traveling Salesman Problem class.

    Defines class which implements multiple traveling salesman solution for rovers.
    """
    
    ##################
    # Public Methods #
    ##################
    
    """
    Constructor
    
    @name __init__
    """
    def __init__(self):
        self.adjMatrix = None
    
    """
    Initialize adjacency square matrix with input number of nodes
    
    @name initAdjacencyMatrix
    @param {number} numNodes number of nodes in adjacency matrix
    """
    @classmethod
    def initAdjacencyMatrix(self, numNodes: int):
        self.adjMatrix = ShortestPaths()
        self.adjMatrix.initAdjacencyMatrix(numNodes)
    
    """
    Add undirected edge between nodes
    
    @name addUndirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {number} weight node1 <-> node2 edge weight
    """
    @classmethod
    def addUndirectedEdge(self, node1: int, node2: int, weight: np.float64):
        self.adjMatrix.addUndirectedEdge(node1, node2, weight)
    
    """
    Applies Algorithm 3, of CFMTSP paper, to find viable rover patrol route solution to a defined graph
    
    @name calculateRoverPaths
    @param {array} vi list of starting graph verticies for each rover/ant
    @param {array} speeds list of rover/ant velocity options (positive, non-zero, float values) along graph edges
    @param {number} Q positive constant used for calculating ant pheromone delta to strengthen pheromone trails
    @param {number} Nm number of iterations/ants to run algorithm
    @param {number} β exponential factor for controlling amount of weight edge travel times have in edge selection
    @param {number} gamma exponential factor for controlling amount of weight ant pheromone levels have in edge selection
    @param {number} evaporationRate rate at which ant pheromones along graph edges diminish per iteration
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
    @param {boolean} alwaysSelectHighestProb flag which determines if algorithm always chooses largest weighted edge
                     or selects edge based on non-uniform distribution (i.e. Greedy & Quick vs. Slow & Optimal)
    @param {number} convergenceLimit number of consecutive successful iterations before assuming solution convergence
    @returns {ndarray} selected graph vertex path, per rover, or None if no viable solution found
             {ndarray} selected [initial] velocities, per graph vertex, or None if no viable solution found
             {number} completion time of slowest rover
    """
    @classmethod
    def calculateRoverPaths(self, vi, speeds, Q=100.0, Nm=0, β=1, gamma=1, evaporationRate=0.01, top=5.0,
                            alwaysSelectHighestProb=True, convergenceLimit=5):
        if self.adjMatrix.isAdjacencyMatrixEmpty():
            raise IndexError("Adjacency matrix is uninitialized!")
        if not vi:
            raise ValueError("Starting vertex list must not be empty")
        if not speeds:
            raise ValueError("Velocity list must not be empty")
        if Q <= 0.0:
            raise ValueError("Q value must be a positive constant")
        if (evaporationRate < 0.0) or (evaporationRate > 1.0):
            raise ValueError("Pheromone evaporation rate must be between 0 and 1")
        if top < 0.0:
            raise ValueError("Operational time (at nodes) cannot be less than zero")
        if convergenceLimit < 1:
            raise ValueError("Convergence count limit cannot be less than one")
        
        # Initialize variables
        Nu = len(vi)
        foundHamiltonianCycles = self.adjMatrix.hasHamiltonianCycle(numStartVertices=20)
        if (foundHamiltonianCycles):
            print("Detected Hamiltonian cycles in graph; simplifying calculations...")
        self.adjMatrix.computeShortestPaths(foundHamiltonianCycles)
        
        eB, edgeEndDict, edgeStartDict, numEdges = self.__createEdgeMatrix()
        𝜓B, 𝜓B_rowDict = self.__createTrajectoryAdjacencyMatrix(numEdges, len(speeds))
        𝜉B, _, 𝜉h_rowDict, 𝜉h_columnDict, _ = self.__createAugmentedEdgeAdjacencyMatrix(𝜓B, 𝜓B_rowDict,
                                                                                           edgeEndDict, edgeStartDict)
        𝜂 = self.__createAugmentedEdgeWeightMatrix(𝜉B, speeds, top, 𝜓B, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
        
        # Important note: CFMTSP paper suggests using acceleration as the weight factor for augmented edges but
        # we use the edge travel time instead to accommodate non-uniform acceleration (if used)
        
        # If Nm is not specified, choose arbitrarily large number of iterations
        if Nm < 1:
            Nm = 100000
            print("Nm value not specified, defaulting to " + str(Nm))
        
        τk = []
        𝜓kbest = None
        tkbest = sys.float_info.max
        selectedVertices = None
        selectedSpeeds = None
        convergenceCount = 0
        prev_max_tkimax = sys.float_info.max
        
        # Initialize pheromone matrices for each rover/ant "species"
        for k in range(Nu):
            τk.append(self.__createPheromoneAdjacencyMatrix(𝜉B))
        
        for r in range(Nm): # For each ant/iteration
            vkcurr = copy.deepcopy(vi)
            Lkunv = [set(range(1, self.adjMatrix.getDistanceMatrix().shape[0] + 1)) for _ in range(Nu)]
            L𝜓k = [[] for _ in range(Nu)]
            Lk𝜉sel = [[] for _ in range(Nu)]
            Livis = [{node: [] for node in range(1, self.adjMatrix.getDistanceMatrix().shape[0] + 1)} for _ in range(Nu)]
            tkimax = [0.0] * Nu
            
            print("Running iteration " + str(r) + "...")
            
            for k in range(Nu): # For each k-th ant species
                LivisIndex = [[0] * self.adjMatrix.getDistanceMatrix().shape[0] for _ in range(Nu)]
                while len(Lkunv[k]) != 0:
                    
                    # CFMTSP paper pseudo-code ONLY selects first edge with no collision and then proceeds to process the augmented
                    # edges of that edge's associated trajectories; it never explores other [non-augmented] edges in future iterations.
                    # This appears to be a design flaw. __chooseAugmentedEdge() modifies algorithm to explore ALL augmented and
                    # non-augmented edges that span the current graph vertex.
                    
                    nextAugmentedEdge = self.__chooseAugmentedEdge(k, speeds, top, tkimax, Livis, LivisIndex, Lkunv, L𝜓k, Lk𝜉sel,
                                                                   eB, edgeStartDict, edgeEndDict, 𝜓B, 𝜓B_rowDict, vkcurr, vi,
                                                                   𝜉B, 𝜉h_columnDict, 𝜉h_rowDict, 𝜂, τk, β, gamma, alwaysSelectHighestProb)
                    if nextAugmentedEdge:
                        Lk𝜉sel[k].append(nextAugmentedEdge)
                        
                        # CalculateMaxArrivalTime()
                        𝜓i = 𝜉h_rowDict[Lk𝜉sel[k][-1]]
                        𝜓j = 𝜉h_columnDict[Lk𝜉sel[k][-1]]
                        tkimax[k] += self.__getEdgeTravelTime(speeds, top, 𝜓B, 𝜓i, 𝜓j, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
                        ei = 𝜓B_rowDict[𝜓i]
                        
                        # Update visted/unvisited node lists
                        # TODO: Wrap this in a function
                        startNode = edgeStartDict[ei] - 1
                        endNode = edgeEndDict[ei] - 1
                        completePath = self.adjMatrix.getShortestPath(startNode, endNode)
                        si = speeds[(𝜓i - 1) % 𝜓B.shape[1]] # Speed defined by initial trajectory node in 𝜓B
                        sj = speeds[(𝜓j - 1) % 𝜓B.shape[1]] # Speed defined by target trajectory node in 𝜓B
                        totalTravelTime = 0.0
                        for pathIndex in range(len(completePath) - 1, 0, -1): # Path is in reverse order
                            startNode = completePath[pathIndex]
                            endNode = completePath[pathIndex - 1]
                            Lij = self.adjMatrix.getDistanceMatrix()[startNode][endNode] # Edge weight (i.e. distance)
                            totalTravelTime += top + self.__getTravelTime(si, sj, Lij)
                            Livis[k][endNode + 1].append(totalTravelTime)
                            si = sj # Assume acceleration takes place only in first sub-branch and acceleration is zero in sub-sequent branches
                                    # TODO: May want to change this for dynamic acceleration options
                        
                        Lkunv[k].remove(edgeEndDict[ei])
                        vkcurr[k] = edgeEndDict[ei] - 1
                        
                        # Update Livis lookup bookmarks
                        for v in range(len(Livis)):
                            if v == k: # Don't need to compare ant's node visit times against self or future ants
                                break
                            for node in range(1, self.adjMatrix.getDistanceMatrix().shape[0] + 1):
                                if node in Livis[v]:
                                    for i in range(LivisIndex[v][node - 1], len(Livis[v][node])):
                                        if (tkimax[k] > Livis[v][node][i]) and (abs(tkimax[k] - Livis[v][node][i]) > top):
                                            LivisIndex[v][node - 1] = i if (i + 1) >= len(Livis[v][node]) else (i + 1)
                                        else:
                                            break # Timestamps are in increasing order so if we reach here, then we do not
                                                  # need to finish iterating through the list of timestamps
                    # END if nextAugmentedEdge:
                    if not L𝜓k[k]:
                        if alwaysSelectHighestProb:
                            # Reduce pheromones along chosen path
                            self.__reducePheromoneTrailAmount(k, Lk𝜉sel, τk, 𝜉h_rowDict, 𝜉h_columnDict, evaporationRate)
                        break # "goto"; there is no "goto" command so we have to mimic the ability via a series of breaks and continues
                    # "L𝜓k <- {}" line moved to top of while loop (inside __chooseAugmentedEdge()) to support double break
                    # logic equivalent of goto
                # END while len(Lkunv[k]) != 0
                if not L𝜓k[k]:
                    break # Break out of "for k in range(Nu)" loop
                # Increase pheromones along chosen path.
                # Contrary to the description in the paper, it makes more sense to move the function here instead of
                # after the "for k in range(Nu)" loop
                self.__calculatePheromoneTrailsAmount(k, Lk𝜉sel, τk, speeds, top, 𝜓B, 𝜉h_rowDict,
                                                      𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict, Q)
                if not alwaysSelectHighestProb:
                    # Evaporate all pheremone trails
                    τk[k] *= (1.0 - evaporationRate)
                
            # END for k in range(Nu)
            if not L𝜓k[k]:
                continue # Move to top of "for r in range(Nm)" loop
            
            # Save the iteration with the minimal worst-case time
            max_tkimax = max(tkimax)
            
            if max_tkimax == prev_max_tkimax:
                convergenceCount += 1
            else:
                convergenceCount = 0 # Reset convergence counter
                prev_max_tkimax = max_tkimax
                
            if max_tkimax < tkbest:
                tkbest = max_tkimax
                𝜓kbest = copy.deepcopy(Lk𝜉sel)
            
            if convergenceCount >= convergenceLimit:
                print("\nReached convergence!\n")
                break
        # END for r in range(Nm)
        
        # Found a viable solution
        if 𝜓kbest:
            selectedVertices = [[] for _ in range(Nu)]
            selectedSpeeds = [[] for _ in range(Nu)]
            
            for k in range(Nu):
                for 𝜉h_index in range(len(𝜓kbest[k])):
                    𝜉h = 𝜓kbest[k][𝜉h_index]
                    𝜓i = 𝜉h_rowDict[𝜉h]
                    𝜓j = 𝜉h_columnDict[𝜉h]
                    ei = 𝜓B_rowDict[𝜓i]
                    si = speeds[(𝜓i - 1) % 𝜓B.shape[1]] # Speed defined by initial trajectory node in 𝜓B
                    sj = speeds[(𝜓j - 1) % 𝜓B.shape[1]] # Speed defined by target trajectory node in 𝜓B
                    startNode = edgeStartDict[ei] - 1
                    endNode = edgeEndDict[ei] - 1
                    completePath = self.adjMatrix.getShortestPath(startNode, endNode)
                    completePath.reverse() # getShortestPaths returns path in reverse order; corrected here
                    
                    # Remove last vertex in path list since it will be included in next iteration
                    # unless we have reached the end
                    if 𝜉h_index < (len(𝜓kbest[k]) - 1):
                        completePath.pop()
                    
                    selectedVertices[k] += completePath
                    selectedSpeeds[k].append(si)
                    # Assume acceleration takes place only in first sub-branch and acceleration is zero in sub-sequent branches
                    # TODO: May want to change this for dynamic acceleration options
                    selectedSpeeds[k] += [sj] * (len(completePath) - 1)
                # END for 𝜉h_index in range(len(𝜓kbest[k])):
            # END for k in range(Nu):
        # END if 𝜓kbest:
        
        return selectedVertices, selectedSpeeds, tkbest
    
    ###################
    # Private Methods #
    ###################
    
    """
    Traverse all augmented edges that span current graph vertex and return augmented edge with highest probability
    weight.
    
    @name __chooseAugmentedEdge
    @param {number} k index for mutable objects
    @param {array} speeds list of rover/ant velocity options (positive, non-zero, float values) along graph edges
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
    @param {ndarray} tkimax maximum route travel time of each ant/rover in current iteration
    @param {array} Livis list of visited nodes, per ant/rover, and associated arrival times
    @param {array} LivisIndex list of visited node bookmarks to help skip unnecessary repeat checks
    @param {array} Lkunv list of unvisited nodes, per ant/rover
    @param {array} L𝜓k list of trajectories which span current graph vertex
    @param {array} L𝜉sel list of augmented edges traversed by ant/rover
    @param {ndarray} eB edge matrix
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {array} vkcurr currently occupied vertex for each rover/ant
    @param {array} vi starting vertex for each rover/ant
    @param {ndarray} 𝜉B augmented-edge adjacency matrix
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {ndarray} 𝜂 augmented edge weight matrix
    @param {ndarray} τk augmented-edge pheromone matrix
    @param {number} β exponential factor for controlling amount of weight edge travel times have in edge selection
    @param {number} gamma exponential factor for controlling amount of weight ant pheromone levels have in edge selection
    @param {boolean} alwaysSelectHighestProb flag which determines if algorithm always chooses largest weighted edge
                     or selects edge based on non-uniform distribution
    @returns {number} augmented edge index
    """
    @classmethod
    def __chooseAugmentedEdge(self, k, speeds, top, tkimax, Livis, LivisIndex, Lkunv, L𝜓k, Lk𝜉sel, eB, edgeStartDict, edgeEndDict,
                              𝜓B, 𝜓B_rowDict, vkcurr, vi, 𝜉B, 𝜉h_columnDict, 𝜉h_rowDict, 𝜂, τk, β, gamma, alwaysSelectHighestProb):
        L𝜓k[k].clear()
        viableEdges = []
        L𝜉k_total = []
        bestAugmentedEdge = None
        
        # From cold start, we need to select from all possible edges
        if not Lk𝜉sel[k]:
            Lek = self.__createEdgeList(eB, vkcurr[k])
            
            # Collect viable edge candidates
            for edge in Lek:
                if edgeEndDict[edge] in Lkunv[k]:
                    viableEdges.append(edge)
        
        # A trajectory was selected before, therefore an edge was already chosen
        else:
            nextTrajectory = 𝜉h_columnDict[Lk𝜉sel[k][-1]]
            nextEdge = 𝜓B_rowDict[nextTrajectory]
            viableEdges.append(nextEdge)
        
        # Find viable edge with best probability
        for edge in viableEdges:
            
            # CreateSubTrajectoriesList()
            if not Lk𝜉sel[k]:
                L𝜓k[k] = [𝜓B[edge - 1][0]] # Bot starting from dead stop, so use lowest initial speed
            else:
                L𝜓k[k] = list(𝜓B[edge - 1])
            
            # CreateAugmentedEdgesList()
            L𝜉k = []
            for 𝜓p in L𝜓k[k]: # For each p-th trajectory in L𝜓k
                L𝜉k += list(𝜉B[𝜓p - 1][𝜉B[𝜓p - 1] != 0])
            
            # Make sure we do not attempt to traverse edges that lead back to the starting vertex until the very end
            if len(Lkunv[k]) > 2:
                L𝜉k = list(filter(lambda 𝜉h: (edgeEndDict[𝜓B_rowDict[𝜉h_columnDict[𝜉h]]] - 1) != vi[k], L𝜉k))
            
            # Perform extra filtering iff the next [un-augmented] vertex is not the last one
            if len(Lkunv[k]) > 1:
                # Remove edges that do not lead to an unvisited vertex
                L𝜉k = list(filter(lambda 𝜉h: edgeEndDict[𝜓B_rowDict[𝜉h_columnDict[𝜉h]]] in Lkunv[k], L𝜉k))
                
            # Check for collisions
            L𝜉k_CollisionFree = []
            for 𝜉h in L𝜉k:
                𝜓i = 𝜉h_rowDict[𝜉h]
                𝜓j = 𝜉h_columnDict[𝜉h]
                startNode = edgeStartDict[𝜓B_rowDict[𝜓i]] - 1
                endNode = edgeEndDict[𝜓B_rowDict[𝜓i]] - 1
                completePath = self.adjMatrix.getShortestPath(startNode, endNode)
                si = speeds[(𝜓i - 1) % 𝜓B.shape[1]] # Speed defined by initial trajectory node in 𝜓B
                sj = speeds[(𝜓j - 1) % 𝜓B.shape[1]] # Speed defined by target trajectory node in 𝜓B
                prevPathTime = 0.0
                collided = False
                
                for pathIndex in range(len(completePath) - 1, 0, -1): # Path is in reverse order
                    startNode = completePath[pathIndex]
                    endNode = completePath[pathIndex - 1]
                    Lij = self.adjMatrix.getDistanceMatrix()[startNode][endNode] # Edge weight (i.e. distance)
                    
                    collided = self.__isCollided(k, si, sj, top, prevPathTime + tkimax[k], Livis, LivisIndex, endNode + 1,
                                                 self.adjMatrix.getDistanceMatrix()[startNode][endNode])
                    if collided:
                        break
                    
                    prevPathTime += top + self.__getTravelTime(si, sj, Lij)
                    si = sj # Assume acceleration takes place only in first sub-branch and acceleration is zero in sub-sequent branches
                            # TODO: May want to change this for dynamic acceleration options
                
                # TODO: Should we add a collision check for next edge here?
                            
                if not collided:
                    L𝜉k_CollisionFree.append(𝜉h)
            # END for 𝜉h in L𝜉k:
            L𝜉k = L𝜉k_CollisionFree
            
            L𝜉k_total += L𝜉k
        # END for edge in viableEdges:
            
        if not L𝜉k_total:
            L𝜓k[k].clear()
            return None # No viable augmented edges
            
        # Select augmented edge with highest probability
        bestAugmentedEdge = self.__selectAugmentedEdge(L𝜉k_total, τk[k], 𝜉h_rowDict, 𝜉h_columnDict, 𝜂, β, gamma, alwaysSelectHighestProb, edgeStartDict, edgeEndDict, 𝜓B_rowDict)
        
        # For debugging
        # ei = 𝜓B_rowDict[𝜉h_rowDict[bestAugmentedEdge]]
        # ej = 𝜓B_rowDict[𝜉h_columnDict[bestAugmentedEdge]]
        # chosen_e1_path = self.adjMatrix.getShortestPath(edgeStartDict[ei] - 1, edgeEndDict[ei] - 1)
        # chosen_e1_path.reverse()
        # chosen_e2_path = self.adjMatrix.getShortestPath(edgeStartDict[ej] - 1, edgeEndDict[ej] - 1)
        # chosen_e2_path.reverse()
        
        return bestAugmentedEdge
    
    """
    Selects augmented edge with highest probability weight, via equation (11) of CFMTSP paper
    
    @name __selectAugmentedEdge
    @param {array} L𝜉k list of augmented edges
    @param {ndarray} τ augmented-edge pheromone matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {ndarray} 𝜂 augmented edge weight matrix
    @param {number} β variable for determining strength of 𝜂 factor in probability formula
    @param {number} gamma variable for determining strength of pheromone factor in probability formula
    @param {boolean} alwaysSelectHighestProb flag which determines if algorithm always chooses largest weighted edge
                     or selects edge based on non-uniform distribution
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @returns {number} augmented edge index
    """
    @classmethod
    def __selectAugmentedEdge(self, L𝜉k, τ, 𝜉h_rowDict, 𝜉h_columnDict, 𝜂, β, gamma, alwaysSelectHighestProb, edgeStartDict, edgeEndDict, 𝜓B_rowDict):
        𝜉h_select = None
        
        # If there is only one choice available, just return it
        if len(L𝜉k) == 1:
            return L𝜉k[0]
        
        # First calculate the sum of ALLOWED neighbor augmented edge choices
        Σneighbors = 0.0
        for 𝜉h in L𝜉k:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            𝜂_hi = 𝜂[i - 1][j - 1]
            τ_hi = τ[i - 1][j - 1]
            Σneighbors += (𝜂_hi**β) * (τ_hi**gamma)
        
        # Finally determine candidate edge with best probability of being chosen among neighbor edges
        # METHOD 1: Always select highest weighted augmented edge
        if alwaysSelectHighestProb:
            highestProb = 0.0
            for 𝜉h in L𝜉k:
                i = 𝜉h_rowDict[𝜉h]
                j = 𝜉h_columnDict[𝜉h]
                
                # For debugging
                # ei = 𝜓B_rowDict[i]
                # ej = 𝜓B_rowDict[j]
                # ei_path = self.adjMatrix.getShortestPath(edgeStartDict[ei] - 1, edgeEndDict[ei] - 1)
                # ei_path.reverse()
                # ej_path = self.adjMatrix.getShortestPath(edgeStartDict[ej] - 1, edgeEndDict[ej] - 1)
                # ej_path.reverse()
                
                𝜂_h = 𝜂[i - 1][j - 1]
                τ_h = τ[i - 1][j - 1]
                Pr𝜉h = ((𝜂_h**β) * (τ_h**gamma)) / Σneighbors
                
                if Pr𝜉h > highestProb:
                    𝜉h_select = 𝜉h
                    highestProb = Pr𝜉h
        # METHOD 2: Select augmented edge based on non-uniform random selection
        else:
            probabilities = []
            for 𝜉h in L𝜉k:
                i = 𝜉h_rowDict[𝜉h]
                j = 𝜉h_columnDict[𝜉h]
                
                # For debugging
                # ei = 𝜓B_rowDict[i]
                # ej = 𝜓B_rowDict[j]
                # ei_path = self.adjMatrix.getShortestPath(edgeStartDict[ei] - 1, edgeEndDict[ei] - 1)
                # ei_path.reverse()
                # ej_path = self.adjMatrix.getShortestPath(edgeStartDict[ej] - 1, edgeEndDict[ej] - 1)
                # ej_path.reverse()
                
                𝜂_h = 𝜂[i - 1][j - 1]
                τ_h = τ[i - 1][j - 1]
                Pr𝜉h = ((𝜂_h**β) * (τ_h**gamma)) / Σneighbors
                probabilities.append(Pr𝜉h)
            
            𝜉h_select = np.random.choice(L𝜉k, p=probabilities)
        
        return 𝜉h_select
    
    """
    Create edge matrix eB from adjacency matrix, via Algorithm 1 of CFMTSP paper
    
    @name __createEdgeMatrix
    @returns {ndarray} edge matrix
             {dictionary} edge end node look-up table
             {dictionary} edge start node look-up table
             {number} number of edges
    """
    @classmethod
    def __createEdgeMatrix(self):
        eB = self.adjMatrix.getDistanceMatrix().copy() # make deep copy
        eB[eB == sys.float_info.max] = 0.0
        eB[eB != 0.0] += 1.0 # cover for possible round-off errors
        edgeEndDict = {}
        edgeStartDict = {}
        q = 0
        eB = eB.astype(type(q)) # change to int type
        
        for index, _ in np.ndenumerate(eB):
            if (index[0] != index[1]) and (eB[index[0]][index[1]] != 0):
                q += 1
                eB[index[0]][index[1]] = q
                edgeEndDict[q] = index[1] + 1
                edgeStartDict[q] = index[0] + 1
                    
        return eB, edgeEndDict, edgeStartDict, q
    
    """
    Create edge list, with specific starting node, from edge matrix
    
    @name __createEdgeList
    @param {ndarray} eB edge matrix
    @param {number} vkcurr starting node
    @returns {array} list of edges with specified starting node
    """
    @classmethod
    def __createEdgeList(self, eB, vkcurr):
        if eB is None:
            raise IndexError("Edge matrix is empty!")
        if eB.shape[0] != eB.shape[1]:
            raise IndexError("Edge matrix must be square!")
        if eB.shape[0] <= vkcurr:
            raise ValueError("Start node value out of bounds!")
        
        # Return non-zero edges in eB with starting node vkcurr
        return list(eB[vkcurr][eB[vkcurr] != 0])
    
    """
    Create trajectory adjacency matrix 𝜓B from edge matrix eB, via equation (7) of CFMTSP paper
    
    @name __createTrajectoryAdjacencyMatrix
    @param {number} numEdges number of edges in edge matrix
    @param {number} numSpeeds number of discrete rover velocity settings
    @returns {ndarray} trajectory adjacency matrix
             {dictionary} row index look-up table
    """
    @classmethod
    def __createTrajectoryAdjacencyMatrix(self, numEdges, numSpeeds):
        𝜓B = np.zeros((numEdges, numSpeeds))
        𝜓B = 𝜓B.astype('int') # change to int type
        𝜓B_rowDict = {}
        
        for index, 𝜓p in np.ndenumerate(𝜓B):
            𝜓p = index[0]*numSpeeds + (index[1] + 1)
            𝜓B[index[0]][index[1]] = 𝜓p
            𝜓B_rowDict[𝜓p] = index[0] + 1
        
        return 𝜓B, 𝜓B_rowDict
    
    """
    Create augmented trajectory adjacency matrix 𝜉B from trajectory adjacency matrix 𝜓B and edge matrix eB, via Algorithm 2 of CFMTSP paper
    
    @name __createAugmentedEdgeAdjacencyMatrix
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜓B_rowDict row index look-up table
    @param {dictionary} edgeEndDict end node look-up table
    @param {dictionary} edgeStartDict start node look-up table
    @returns {ndarray} augmented trajectory matrix
             {number} index count
             {dictionary} 𝜉h row look-up table
             {dictionary} 𝜉h column look-up table
             {dictionary} sub-trajectory (𝜓) neighbor look-up table
    """
    @classmethod
    def __createAugmentedEdgeAdjacencyMatrix(self, 𝜓B, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        if 𝜓B is None:
            raise IndexError("Trajectory adjacency matrix is empty!")
        
        # |𝜓B|
        num_elements = 𝜓B.shape[0] * 𝜓B.shape[1]
        
        # Initialize 𝜉B with zeros
        𝜉B = np.zeros((num_elements, num_elements))
        𝜉B = 𝜉B.astype('int') # change to int type
        
        # Algorithm body
        𝜉h_rowDict = {}
        𝜉h_columnDict = {}
        𝜓i_neighbors = {}
        𝜉h = h = 0
        for i in range(num_elements): # For i=1,...,|𝜓B|
            neighbors = []
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
                        neighbors.append(j + 1) # Add 𝜓j to neighbor list of 𝜓i
            𝜓i_neighbors[i + 1] = neighbors
        
        return 𝜉B, 𝜉h, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors
    
    """
    Creates weight matrix associated with augmented edge adjacency matrix
    
    @name __createAugmentedEdgeWeightMatrix
    @param {ndarray} 𝜉B augmented edge adjacency matrix
    @param {array} speeds list of available speed selections for vehicle
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @returns {ndarray} augmented edge weight matrix
    """
    @classmethod
    def __createAugmentedEdgeWeightMatrix(self, 𝜉B, speeds, top, 𝜓B, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        if 𝜉B is None:
            raise IndexError("Augmented edge adjacency matrix is empty!")
        
        𝜂 = 𝜉B.copy() # make deep copy
        𝜂 = 𝜂.astype('float64') # Change to floating precision
        
        for index, 𝜉h in np.ndenumerate(𝜉B):
            if 𝜉h != 0:
                # We set 𝜂 to be the multiplicative inverse of the edge travel time
                edgeTravelTime = self.__getEdgeTravelTime(speeds, top, 𝜓B, index[0] + 1, index[1] + 1,
                                                          𝜓B_rowDict, edgeEndDict, edgeStartDict)
                inverse = 1.0 / edgeTravelTime
                𝜂[index[0]][index[1]] = inverse
        
        return 𝜂
    
    """
    Initialize pheromone adjacency matrix edges with default value
    
    @name __createPheromoneAdjacencyMatrix
    @param {ndarray} 𝜉B augmented edge adjacency matrix
    @returns {ndarray} pheromone adjacency matrix
    """
    @classmethod
    def __createPheromoneAdjacencyMatrix(self, 𝜉B):
        if 𝜉B is None:
            raise IndexError("Augmented edge adjacency matrix is empty!")
        
        τ = 𝜉B.copy() # make deep copy
        τ = τ.astype('float64') # Change to floating precision
        τ[τ != 0.0] = 1.0 # initialize non-empty edges
        
        return τ
    
    """
    Adds phermone along augmented route of pheromone matrix
    
    @name __calculatePheromoneTrailsAmount
    @param {number} k index for mutable objects
    @param {array} L𝜉sel list of augmented edges traversed by ant
    @param {ndarray} τ pheromone matrix
    @param {array} speeds list of available speed selections for vehicle
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @param {number} Q positive constant used for calculating ant pheromone delta to strengthen pheromone trails
    """
    @classmethod
    def __calculatePheromoneTrailsAmount(self, k, L𝜉sel, τ, speeds, top, 𝜓B, 𝜉h_rowDict,
                                         𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict, Q):
        totalPathTime = self.__getPathTravelTime(speeds, top, 𝜓B, L𝜉sel[k], 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict,
                                                 edgeEndDict, edgeStartDict)
        # print("CURRENT VALUE: " + str(Q/totalPathTime))
        for 𝜉h in L𝜉sel[k]:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            
            # Mutable object
            τ[k][i - 1][j - 1] += Q/totalPathTime # Single ant of single species so we don't need to worry
                                                  # about including delta tau of other ants in 1 iteration
    
    """
    Reduces phermone along augmented route of pheromone matrix
    
    @name __reducePheromoneTrailAmount
    @param {number} k index for mutable objects
    @param {array} L𝜉sel list of augmented edges traversed by ant
    @param {ndarray} τ pheromone matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {number} evaporationRate evaporation rate of pheromone along agumented edge
    """
    @classmethod
    def __reducePheromoneTrailAmount(self, k, L𝜉sel, τ, 𝜉h_rowDict, 𝜉h_columnDict, evaporationRate):
        for 𝜉h in L𝜉sel[k]:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            
            # Mutable object
            τ[k][i - 1][j - 1] *= (1.0 - evaporationRate)
    
    """
    Calculate uniform acceleration along augmented edge
    
    @name __getAcceleration
    @param {number} si initial speed at node i
    @param {number} sj final speed at node j
    @param {number} Lij edge length between nodes i and j
    @returns {number} uniform acceleration from node i to node j
    """
    @classmethod # TODO: Change this for non-uniform acceleration?
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
    @classmethod # TODO: Change this for non-uniform acceleration?
    def __getTravelTime(self, si, sj, Lij):
        a_ij = self.__getAcceleration(si, sj, Lij)
        
        if (a_ij == 0):
            return Lij/si
        else:
            return (-si + math.sqrt(si**2 + 2*a_ij*Lij)) / a_ij
    
    """
    Determines if collision can/will happen along selected edge
    
    @name __isCollided
    @param {number} k index for mutable objects
    @param {number} si initial speed of vehicle along edge
    @param {number} sj target speed of vehicle along edge
    @param {number} top operational time of vehicle at a node
    @param {number} Livis list of visited nodes and arrival times
    @param {array} LivisIndex list of visited node bookmarks to help skip unnecessary repeat checks
    @param {number} edgeEnd target node of edge
    @param {number} edgeDistance edge weight
    @returns {bool} collision prediction
    """
    @classmethod
    def __isCollided(self, k, si, sj, top, currentTime, Livis, LivisIndex, edgeEnd, edgeDistance):
        for v in range(len(Livis)):
            if v == k: # Don't need to compare ant's node visit times against self or future ants
                break
            if edgeEnd in Livis[v]:
                # TODO: Add if condition to limit speed selection to lowest speeds for
                #       starting vertex? Makes sense since bots would be rolling from dead start
                for i in range(LivisIndex[v][edgeEnd - 1], len(Livis[v][edgeEnd])):
                    tik1 = currentTime + top + self.__getTravelTime(si, sj, edgeDistance)
                    tik2 = Livis[v][edgeEnd][i]
                    if abs(tik1 - tik2) <= top:
                        return True
                    # END if abs(tik1 - tik2) <= top:
                # END for i in range(LivisIndex[v][edgeEnd - 1], len(Livis[v][edgeEnd])):
            # END if edgeEnd in Livis[v]:
        # END for v in range(len(Livis)):
        return False
    
    """
    Calculate travel time along augmented edge
    
    @name __getEdgeTravelTime
    @param {array} speeds list of available speed selections for vehicle
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {number} i augmented edge row index in 𝜉B
    @param {number} j augmented edge column index in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @returns {number} travel time for augmented edge indexed by 𝜉h
    """
    @classmethod
    def __getEdgeTravelTime(self, speeds, top, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        c1_edge = 𝜓B_rowDict[i] # Edge 1
        startNode = edgeStartDict[c1_edge] - 1
        endNode = edgeEndDict[c1_edge] - 1
        completePath = self.adjMatrix.getShortestPath(startNode, endNode)
        si = speeds[(i - 1) % 𝜓B.shape[1]] # Speed defined by initial trajectory node in 𝜓B
        sj = speeds[(j - 1) % 𝜓B.shape[1]] # Speed defined by target trajectory node in 𝜓B
        totalTravelTime = 0.0
        
        for pathIndex in range(len(completePath) - 1, 0, -1): # Path is in reverse order
            startNode = completePath[pathIndex]
            endNode = completePath[pathIndex - 1]
            Lij = self.adjMatrix.getDistanceMatrix()[startNode][endNode] # Edge weight (i.e. distance)
            totalTravelTime += top + self.__getTravelTime(si, sj, Lij)
            si = sj # Assume acceleration takes place only in first sub-branch and acceleration is zero in sub-sequent branches
                    # TODO: May want to change this for dynamic acceleration options
        
        return totalTravelTime
    
    """
    Calculate travel time along augmented path
    
    @name __getPathTravelTime
    @param {array} speeds list of available speed selections for vehicle
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
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
    def __getPathTravelTime(self, speeds, top, 𝜓B, L𝜉sel, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        totalTravelTime = 0.0
        
        for 𝜉h in L𝜉sel:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            totalTravelTime += self.__getEdgeTravelTime(speeds, top, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
        
        return totalTravelTime


class ImageToGPSConverter:
    """
    Image conversion class.

    Defines class which encapsulates methodology of converting drone image pixel x-y cooridinates to GPS coordinates.
    """
    
    ##################
    # Public Methods #
    ##################

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
    
    ###################
    # Private Methods #
    ###################
    
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
            math.fmod(((lambda2 * 180) / math.pi + 540), 360) - 180,
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
