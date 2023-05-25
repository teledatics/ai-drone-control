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
# - random (https://docs.python.org/3/library/random.html)
#   - Access to pseudo-random number generators for CFMTSP matrix weight initializations
# - sys (https://docs.python.org/3/library/sys.html)
#   - Access to system-specific parameters and functions
# - copy (https://docs.python.org/3/library/copy.html)
#   - Access to deep copy operations
#
# @section todo_path_planning TODO
# - Test and verify CFMTSP solution (with simulated data)
# - Modify CFMTSP solution to accomodate NON-uniform acceleration
# - Modify CFMTSP solution to account for speed limits needed to accomodate sharp turns; until then, keep max speed low
# - Add matrix to cache augmented graph edge weights for CFMTSP solution
# - Test and verify pixel to GPS coordinate converter
#
# @section author_path_planning Author(s)
# - Created by Justin Carrel on 02/26/2023
#
# @section references_path_planning References
# - https://www.researchgate.net/publication/342492385_Novel_Graph_Model_for_Solving_Collision-Free_Multiple-Vehicle_Traveling_Salesman_Problem_Using_Ant_Colony_Optimization
# - https://blog.roboflow.com/georeferencing-drone-videos/
#
# Copyright © 2023 Teledatics. All rights reserved.

import math
import numpy as np
import random
import sys
import copy

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
    def initAdjacencyMatrix(self, numNodes):
        if (numNodes <= 0):
            raise ValueError("Number of nodes must be greater than zero")
        self.adjMatrix = np.zeros((numNodes, numNodes))
    
    """
    Add undirected edge between nodes
    
    @name addUndirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {number} weight node1 <-> node2 edge weight
    """
    @classmethod
    def addUndirectedEdge(self, node1, node2, weight):
        # An undirected edge is just a bi-directional edge between nodes
        self.__addDirectedEdge(node1, node2, weight)
        self.__addDirectedEdge(node2, node1, weight)
    
    """
    Applies Algorithm 3, of CFMTSP paper, to find viable rover patrol route solution to a defined graph
    
    @name calculateRoverPaths
    @param {array} vi list of starting graph verticies for each rover/ant
    @param {array} speeds list of rover/ant velocity options (positive, non-zero, float values) along graph edges
    @param {number} Nm number of iterations/ants to run algorithm
    @param {number} β exponential factor for controlling amount of weight edge travel times have in edge selection
    @param {number} gamma exponential factor for controlling amount of weight ant pheromone levels have in edge selection
    @param {number} evaporationRate rate at which ant pheromones along graph edges diminish per iteration
    @param {number} top operational time; time a rover/ant spends at a node between entering and leaving the node
    @returns {ndarray} selected graph vertex path, per rover, or None if no viable solution found
             {ndarray} selected [initial] velocities, per graph vertex, or None if no viable solution found
    """
    @classmethod
    def calculateRoverPaths(self, vi, speeds, Nm, β=1, gamma=1, evaporationRate=0.01, top=5.0):
        if self.adjMatrix is None:
            raise IndexError("Adjacency matrix is uninitialized!")
        if not vi:
            raise ValueError("Starting vertex list must not be empty")
        if not speeds:
            raise ValueError("Velocity list must not be empty")
        if (evaporationRate < 0.0) or (evaporationRate > 1.0):
            raise ValueError("Pheromone evaporation rate must be between 0 and 1")
        if top < 0.0:
            raise ValueError("Operational time (at nodes) cannot be less than zero")
        
        # Initialize variables
        Nu = len(vi)
        
        eB, edgeEndDict, edgeStartDict, numEdges = self.__createEdgeMatrix()
        𝜓B, 𝜓B_rowDict = self.__createTrajectoryAdjacencyMatrix(numEdges, len(speeds))
        𝜉B, 𝜉h_count, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors = self.__createAugmentedEdgeAdjacencyMatrix(𝜓B, 𝜓B_rowDict,
                                                                                                        edgeEndDict, edgeStartDict)
        # TODO: Remove these printouts after simulation testing finished
        print("Adjacency Matrix:")
        print(self.adjMatrix)
        print("\nEdge Matrix:")
        print(eB)
        print("\nTrajectory Adjacency Matrix:")
        print(𝜓B)
        print("\nAugmented Edge Adjacency Matrix:")
        print(𝜉B)
        
        # Important note: CFMTSP paper suggests using acceleration as the weight factor for augmented edges but
        # we use the edge travel time instead to accommodate non-uniform acceleration (if used)
        
        τk = []
        Pr𝜉hk = []
        𝜓kbest = None
        tkbest = sys.float_info.max
        selectedVertices = None
        selectedSpeeds = None
        
        # Initialize pheromone and probability matrices for each rover/ant "species"
        for k in range(Nu):
            τk.append(self.__createPheromoneAdjacencyMatrix(𝜉B))
            Pr𝜉hk.append(self.__createPr𝜉hMatrix(𝜉B, τk[k], speeds, 𝜓B, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors,
                                                   𝜓B_rowDict, edgeEndDict, edgeStartDict, β, gamma))
        
        for r in range(Nm): # For each ant/iteration
            vkcurr = copy.deepcopy(vi)
            Lkunv = [set(range(1, self.adjMatrix.shape[0] + 1)) for _ in range(Nu)]
            Lek = [[] for _ in range(Nu)]
            L𝜓k = [[] for _ in range(Nu)]
            L𝜉k = [[] for _ in range(Nu)]
            Lk𝜉sel = [[] for _ in range(Nu)]
            Livis = [{} for _ in range(Nu)]
            tkimax = [0] * Nu
            
            for k in range(Nu): # For each k-th ant species
                while len(Lkunv[k]) != 0:
                    Lek[k] = self.__createEdgeList(eB, vkcurr[k])
                    L𝜓k[k].clear()
                    for edge in Lek[k]:
                        if edgeEndDict[edge] in Lkunv[k]: # TODO: Combine if conditions
                            if self.__isCollided(speeds, top, tkimax[k], Livis, edgeEndDict[edge],
                                                 self.adjMatrix[edgeStartDict[edge]][edgeEndDict[edge]]):
                                continue
                            
                            # CreateSubTrajectoriesList()
                            if vkcurr[k] == vi[k]:
                                L𝜓k[k] = [𝜓B[edge - 1][0]] # Bot starting from dead stop, so use lowest initial speed
                            else:
                                # Filter augmented edges that do not connect previously chosen edge to current edge
                                L𝜉k[k] = list(filter(lambda x: (𝜉h_columnDict[x] in 𝜓B[edge - 1]), L𝜉k[k]))
                                # Select augmented edge with highest probability
                                Lk𝜉sel[k].append(self.__selectAugmentedEdge(L𝜉k[k], Pr𝜉hk[k], 𝜉h_rowDict, 𝜉h_columnDict))
                                # Extract trajectory that is connected to previous trajectory via chosen augmented edge
                                L𝜓k[k] = [𝜉h_columnDict[Lk𝜉sel[k][-1]]]
                            
                            # CreateAugmentedEdgesList()
                            L𝜉k[k].clear()
                            for 𝜓p in L𝜓k[k]: # For each p-th trajectory in L𝜓k
                                L𝜉k[k] += list(𝜉B[𝜓p - 1][𝜉B[𝜓p - 1] != 0])
                            
                            # CFMTSP paper pseudo-code places augmented edge selection here but that doesn't make sense;
                            # we first need to know the next viable [normal] edge to determine which target trajectory
                            # nodes at the endpoints of the augmented edges in L𝜉k are valid.
                            # See CreateSubTrajectoriesList() implementation above
                            
                            # CalculateMaxArrivalTime()
                            if Lk𝜉sel[k]:
                                i = 𝜉h_rowDict[Lk𝜉sel[k][-1]]
                                j = 𝜉h_columnDict[Lk𝜉sel[k][-1]]
                                tkimax[k] += self.__getEdgeTravelTime(speeds, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict) + top
                                Livis[k][edgeStartDict[edge]] = tkimax[k]
                            
                            Lkunv[k].remove(edgeEndDict[edge])
                            vkcurr[k] = edgeEndDict[edge] - 1
                            
                            # Additional step for when we have reached the final node, since augmented edges
                            # are added 1 loop iteration later instead of current iteration
                            if len(Lkunv[k]) == 0: # We have visited the last node so add final edge to Lk𝜉sel
                                # Select augmented edge with highest probability (skip filtering since we do not care
                                # about the next edge connection)
                                Lk𝜉sel[k].append(self.__selectAugmentedEdge(L𝜉k[k], Pr𝜉hk[k], 𝜉h_rowDict, 𝜉h_columnDict))
                                
                                i = 𝜉h_rowDict[Lk𝜉sel[k][-1]]
                                j = 𝜉h_columnDict[Lk𝜉sel[k][-1]]
                                tkimax[k] += self.__getEdgeTravelTime(speeds, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict) + top
                                Livis[k][edgeEndDict[edge]] = tkimax[k]
                            
                            # CFMTSP paper pseudo-code does not show break statement, but we need it; we do not need to
                            # look at other edges since we successfully chose one
                            break
                        # END if edgeEndDict[edge] in Lkunv[k]
                        else:
                            continue
                    # END for edge in Lek[k]
                    if not L𝜓k[k]:
                        self.__reducePheromoneTrailAmount(Lk𝜉sel[k], τk[k], 𝜉h_rowDict, 𝜉h_columnDict, evaporationRate)
                        break # "goto"; there is no "goto" command so we have to mimic the ability via a series of breaks and continues
                    # "L𝜓k <- {}" line moved to top of while loop to support double break logic equivalent of goto
                # END while len(Lkunv[k]) != 0
                if not L𝜓k[k]:
                    break # Break out of "for k in range(Nu)" loop
            # END for k in range(Nu)
            if not L𝜓k[k]:
                continue # Move to top of "for r in range(Nm)" loop
            
            for k in range(Nu): # for all k ε {1,2,...,Nu}
                self.__calculatePheromoneTrailsAmount(Lk𝜉sel[k], τk[k], speeds, 𝜓B, 𝜉h_rowDict,
                                                      𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
                calculateProbabilities = lambda 𝜉h: self.__Pr𝜉h(𝜉h, τk[k], speeds, 𝜓B, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors,
                                                                  𝜓B_rowDict, edgeEndDict, edgeStartDict, β, gamma)
                # Calculate probabilities only for existing edges
                Pr𝜉hk[k][Pr𝜉hk[k] != 0] = np.array(list(map(calculateProbabilities, Pr𝜉hk[k][Pr𝜉hk[k] != 0])))
            
            # Save the iteration with the minimal worst-case time
            max_tkimax = max(tkimax)
            if max_tkimax < tkbest:
                tkbest = max_tkimax
                𝜓kbest = copy.deepcopy(Lk𝜉sel)
        # END for r in range(Nm)
        
        # TODO: Add some debug printouts here
        
        # Found a viable solution
        if 𝜓kbest is not None:
            selectedVertices = [[] for _ in range(Nu)]
            selectedSpeeds = [[] for _ in range(Nu)]
            
            for k in range(Nu):
                for 𝜉h in 𝜓kbest[k]:
                    𝜓i = 𝜉h_rowDict[𝜉h]
                    selectedVertices.append(edgeStartDict(𝜓B_rowDict[𝜓i]) - 1) # Set to 0-based index
                    selectedSpeeds.append(speeds[(𝜓i - 1) % 𝜓B.shape[1]])
        
        return selectedVertices, selectedSpeeds
    
    ###################
    # Private Methods #
    ###################
    
    """
    Selects augmented edge with highest probability weight
    
    @name __isCollided
    @param {array} L𝜉k list of augmented edges
    @param {ndarray} Pr𝜉hk augmented edge probability matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in Pr𝜉hk
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in Pr𝜉hk
    @returns {number} augmented edge index
    """
    @classmethod
    def __selectAugmentedEdge(self, L𝜉k, Pr𝜉hk, 𝜉h_rowDict, 𝜉h_columnDict):
        𝜉h_select = 0
        highestProb = 0.0
        
        for 𝜉h in L𝜉k:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            if Pr𝜉hk[i - 1][j - 1] > highestProb:
                𝜉h_select = 𝜉h
                highestProb = Pr𝜉hk[i - 1][j - 1]
            
        return 𝜉h_select
    
    """
    Add directed edge between nodes
    
    @name addDirectedEdge
    @param {number} node1 start node in adjacency matrix
    @param {number} node2 end node in adjacency matrix
    @param {number} weight node1 -> node2 edge weight
    """
    @classmethod
    def __addDirectedEdge(self, node1, node2, weight):
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
        if self.adjMatrix is None:
            raise IndexError("Adjacency matrix is uninitialized!")
        
        eB = self.adjMatrix.copy() # make deep copy
        eB[eB != 0.0] += 1.0 # cover for possible round-off errors
        edgeEndDict = {}
        edgeStartDict = {}
        q = 0
        eB = eB.astype(type(q)) # change to int type
        
        for i in range(eB.shape[0]): # rows
            for j in range(eB.shape[1]): # columns
                if (i != j) and (eB[i][j] != 0):
                    q += 1
                    eB[i][j] = q
                    edgeEndDict[q] = j + 1
                    edgeStartDict[q] = i + 1
                    
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
        
        for i in range(𝜓B.shape[0]): # rows
            for j in range(𝜓B.shape[1]): # columns
                𝜓p = i*numSpeeds + (j + 1)
                𝜓B[i][j] = 𝜓p
                𝜓B_rowDict[𝜓p] = i + 1
        
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
    Initialize pheromone adjacency matrix edges with default value
    
    @name __createPheromoneAdjacencyMatrix
    @param {ndarray} augmented edge adjacency matrix
    @returns {ndarray} pheromone adjacency matrix
    """
    @classmethod
    def __createPheromoneAdjacencyMatrix(self, 𝜉B):
        if 𝜉B is None:
            raise IndexError("Augmented edge adjacency matrix is empty!")
        
        τ = 𝜉B.copy() # make deep copy
        τ = τ.astype('float64') # Change to floating precision
        # randomInit = lambda: math.fmod((random.random() + 0.1), 1.0) # Prevents zero from being generated
        # τ = randomInit(τ[τ != 0]) # initialize non-empty edges
        τ[τ != 0] = 0.5 # initialize non-empty edges
        
        return τ
    
    """
    Initialize augmented edge selection probability matrix
    
    @name __createPr𝜉hMatrix
    @param {ndarray} 𝜉B augmented edge adjacency matrix
    @param {ndarray} τ pheromone matrix
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓i_neighbors sub-trajectory neighbor look-up table
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @param {number} β variable for determining strength of 𝜂 factor in probability formula
    @param {number} gamma variable for determining strength of pheromone factor in probability formula
    @returns {ndarray} augmented edge slection probability matrix
    """
    @classmethod
    def __createPr𝜉hMatrix(self, 𝜉B, τ, speeds, 𝜓B, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors,
                          𝜓B_rowDict, edgeEndDict, edgeStartDict, β=1, gamma=1):
        if 𝜉B is None:
            raise IndexError("Augmented edge adjacency matrix is empty!")
        if τ is None:
            raise IndexError("Pheromone adjacency matrix is empty!")
        
        Pr𝜉h = 𝜉B.copy() # make deep copy
        Pr𝜉h = Pr𝜉h.astype('float64') # Change to floating precision
        calculateProbabilities = lambda 𝜉h: self.__Pr𝜉h(𝜉h, τ, speeds, 𝜓B, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors,
                                                          𝜓B_rowDict, edgeEndDict, edgeStartDict, β, gamma)
         # calculate probabilities only for exisiting edges
        Pr𝜉h[Pr𝜉h != 0] = np.array(list(map(calculateProbabilities, Pr𝜉h[Pr𝜉h != 0])))
        
        return Pr𝜉h
    
    """
    Adds phermone along augmented route of pheromone matrix
    
    @name __calculatePheromoneTrailsAmount
    @param {array} L𝜉sel list of augmented edges traversed by ant
    @param {ndarray} τ pheromone matrix
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    """
    @classmethod
    def __calculatePheromoneTrailsAmount(self, L𝜉sel, τ, speeds, 𝜓B, 𝜉h_rowDict,
                                         𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        totalPathTime = self.__getPathTravelTime(speeds, 𝜓B, L𝜉sel, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict,
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
    def __reducePheromoneTrailAmount(self, L𝜉sel, τ, 𝜉h_rowDict, 𝜉h_columnDict, evaporationRate=0.01):
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
    @param {array} speeds list of available speed selections for vehicle
    @param {number} top operational time of vehicle at a node
    @param {number} Livis list of visited nodes and arrival times
    @param {number} edgeEnd target node of edge
    @param {number} edgeDistance edge weight
    @returns {bool} collision prediction
    """
    @classmethod
    def __isCollided(self, speeds, top, currentTime, Livis, edgeEnd, edgeDistance):
        for k in range(len(Livis)):
            if edgeEnd in Livis[k]:
                # TODO: Add if condition to limit speed selection to lowest speeds for
                #       starting vertex? Makes sense since bots would be rolling from dead start
                for si in speeds:
                    for sj in speeds:
                        tik1 = currentTime + top + self.__getTravelTime(si, sj, edgeDistance)
                        tik2 = Livis[k][edgeEnd]
                        if abs(tik1 - tik2) <= top: # Collision condition according to Definition 6 of CFMTSP paper
                            # CFMTSP paper is fuzzy on the details but seems to imply that
                            # "minimum arrival time difference" determines collision state. If that is the case,
                            # then we just return TRUE the moment we find 1 possibility of collision among the
                            # various combinations of velocity selections, and try another edge. However it looks like this
                            # could lead to the [increased] possibility of the algorithm getting stuck if the bots are
                            # clustered together. May want to consider changing the main algorithm to allow sub-set of
                            # trajectories that will not result in a collision at the common end node.
                            return True
        
        return False
    
    """
    Calculate travel time along augmented edge
    
    @name __getEdgeTravelTime
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {number} i augmented edge row index in 𝜉B
    @param {number} j augmented edge column index in 𝜉B
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @returns {number} travel time for augmented edge indexed by 𝜉h
    """
    @classmethod
    def __getEdgeTravelTime(self, speeds, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        c1_edge = 𝜓B_rowDict[i] # Edge 1
        si = speeds[(i - 1) % 𝜓B.shape[1]] # Speed defined by initial trajectory node in 𝜓B
        sj = speeds[(j - 1) % 𝜓B.shape[1]] # Speed defined by target trajectory node in 𝜓B
        Lij = self.adjMatrix[edgeStartDict[c1_edge] - 1][edgeEndDict[c1_edge] - 1] # Edge 1 weight (i.e. distance)
        
        return self.__getTravelTime(si, sj, Lij)
    
    """
    Calculate travel time along augmented path
    
    @name __getPathTravelTime
    @param {array} speeds list of available speed selections for vehicle
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
    def __getPathTravelTime(self, speeds, 𝜓B, L𝜉sel, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓B_rowDict, edgeEndDict, edgeStartDict):
        totalTravelTime = 0.0
        
        for 𝜉h in L𝜉sel:
            i = 𝜉h_rowDict[𝜉h]
            j = 𝜉h_columnDict[𝜉h]
            totalTravelTime += self.__getEdgeTravelTime(speeds, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
        
        return totalTravelTime
    
    """
    Calculate augmented edge selection probability, via equation (11) of CFMTSP paper
    
    @name __Pr𝜉h
    @param {number} 𝜉h edge selection from augmented edge matrix 𝜉B
    @param {ndarray} τ pheromone matrix
    @param {array} speeds list of available speed selections for vehicle
    @param {ndarray} 𝜓B trajectory adjacency matrix
    @param {dictionary} 𝜉h_rowDict look-up table of augmented edge row indexes in 𝜉B
    @param {dictionary} 𝜉h_columnDict look-up table of augmented edge column indexes in 𝜉B
    @param {dictionary} 𝜓i_neighbors sub-trajectory neighbor look-up table
    @param {dictionary} 𝜓B_rowDict look-up table of edge row indexes in 𝜓B
    @param {dictionary} edgeEndDict look-up table of end nodes for edges
    @param {dictionary} edgeStartDict look-up table of start nodes for edges
    @param {number} β variable for determining strength of 𝜂 factor in probability formula
    @param {number} gamma variable for determining strength of pheromone factor in probability formula
    @returns {number} probability of ant selecting augmented edge 𝜉h
    """
    @classmethod
    def __Pr𝜉h(self, 𝜉h, τ, speeds, 𝜓B, 𝜉h_rowDict, 𝜉h_columnDict, 𝜓i_neighbors,
                               𝜓B_rowDict, edgeEndDict, edgeStartDict, β=1, gamma=1):
        i = 𝜉h_rowDict[𝜉h]
        j = 𝜉h_columnDict[𝜉h]
        Σneighbors = 0.0
        
        for neighbor𝜓 in 𝜓i_neighbors[i]:
            # We set 𝜂 to be the multiplicative inverse of the edge travel time
            𝜂_hi = 1/self.__getEdgeTravelTime(speeds, 𝜓B, i, neighbor𝜓, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
            τ_hi = τ[i - 1][neighbor𝜓 - 1]
            Σneighbors += (𝜂_hi**β) * (τ_hi**gamma)
        
        # We set 𝜂 to be the multiplicative inverse of the edge travel time
        𝜂_h = 1/self.__getEdgeTravelTime(speeds, 𝜓B, i, j, 𝜓B_rowDict, edgeEndDict, edgeStartDict)
        τ_h = τ[i - 1][j - 1]
        
        return ((𝜂_h**β) * (τ_h**gamma)) / Σneighbors


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
