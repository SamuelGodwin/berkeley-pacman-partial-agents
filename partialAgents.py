# partialAgent.py
# parsons/15-oct-2017
#
# Version 1
#
# The starting point for CW1.
#
# Intended to work with the PacMan AI projects from:
#
# http://ai.berkeley.edu/
#
# These use a simple API that allow us to control Pacman's interaction with
# the environment adding a layer on top of the AI Berkeley code.
#
# As required by the licensing agreement for the PacMan AI we have:
#
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

# The agent here is was written by Simon Parsons, based on the code in
# pacmanAgents.py

from pacman import Directions
from game import Agent
import api
import random
import game
import util

#PartialAgent
#
#Uses A* search to go to all turning points on the map, then clears up any remaining food
#Finds the corners if game still not won
#Evades ghosts as they come within sight with the aim of winning the game


class PartialAgent(Agent):

    # Constructor: this gets run when we first invoke pacman.py
    #
    # Initialise state variables that reset in between games
    def __init__(self):
        print "Starting up!"
        name = "Pacman"
        self.visitedWaypoints = []
        self.foodMap = []
        self.previousSteps = []

    # A* search algorithm, finds best route to given goal using
    # Manhattan distance from starting point as an heuristic
    # learnt from and inspired by website provided by Simon Parsons/Elizabeth Black on KEATS:
    #   https://www.redblobgames.com/pathfinding/a-star/
    def searchAStar(self, state, start, goal):

        furthestReach = util.PriorityQueue() # priority queue using distance from start point as priority
        currentTileDistance = {} # distance of current tile from start point
        previousTiles = {} # which tile the current tile came from (used in route path creation)
        currentTileDistance[start] = 0 # start point obviously has no distance
        previousTiles[start] = None # starting tile didn't have a route leading to it
        furthestReach.push(start, 0) # add starting tile to queue with a priority of 0

        pacman = api.whereAmI(state) # pacman's current location

        # setting up previousSteps list as a makeshift FIFO queue, containing pacman's 7 previous steps
        # fix for perpetually repeating 2 moves - caused by overlapping routes to different waypoints
        # using previousSteps queue to determine if pacman is in deadlock
        # 7 seems a sensible amount of repeats for 'back and forth' motion to be detected
        # without taking up too much time either.

        # don't let list get longer than 7 items
        if len(self.previousSteps) == 7:
            self.previousSteps.remove(self.previousSteps[0]) # remove front element of queue 

        # Add pacman's current position to queue
        self.previousSteps.append(pacman)

        
        if len(self.previousSteps) == 7:
            # if pacman is stuck travelling back and forth between two adjacent points, repeatedly...
            if (self.previousSteps[0] == self.previousSteps[2] == self.previousSteps[4] == self.previousSteps[6]) and (self.previousSteps[1] == self.previousSteps[3] == self.previousSteps[5]):
                # it's in deadlock
                # remove current closest waypoint. this should work at any given point
                self.visitedWaypoints.append(goal)

        # beginning A* search here
        while not furthestReach.isEmpty():
            # currentTile is first item in the priority queue
            currentTile = furthestReach.pop()

            # if we're still not at the goal, continue finding best route
            if currentTile != goal:

                # x and y coordinate of currentTile
                x = currentTile[0] 
                y = currentTile[1] 

                # checking all surrounding tiles for best route
                surroundingTiles = [(x,y-1), (x,y+1), (x-1,y), (x+1, y)]
                wallsList = api.walls(state)

                # for each of the surrounding tiles, calculate the distance from start
                for tile in surroundingTiles:
                    if tile not in wallsList: # don't want walls in path
                        nextTileDistance = currentTileDistance[currentTile] + 1 
                        if tile not in currentTileDistance or nextTileDistance < currentTileDistance[tile]:
                            currentTileDistance[tile] = nextTileDistance
                            # furthestReach puts tile into priorityqueue, with a priority. 
                            # if that's lowest, it means it has the smaller distance
                            # meaning a better route - goes in with lower value of priority and come out first as best route
                            # so lower value is higher priority
                            # sorts things in priority queue by which route is best
                            priorityOfPQueue = nextTileDistance + util.manhattanDistance(goal, tile) 
                            furthestReach.push(tile, priorityOfPQueue)
                            previousTiles[tile] = currentTile

            else:
                # if we've hit the goal, stop trying to find it
                break

        return previousTiles # return the set of coordinates in the best route

    # use A* search results to find the best route to the goal
    def getPathToGoal(self, previousTiles, start, goal):
        current = goal # start at the goal to work backwards through the route
        pathToGoal = []

        # add the previous tile in the route to the path list
        while current != start:
            pathToGoal.append(current)
            current = previousTiles[current]

        # reverse list to make a path from start to goal
        pathToGoal.reverse()
        
        return pathToGoal

    # map of accessible paths (no walls)
    def createMapOfAllPaths(self, state):

        # get the corners of the layout map
        corners = api.corners(state)
        # pathways, +1 to accommodate for edge walls
        width = corners[3][0] + 1
        height = corners[3][1] + 1
        coordsList = []
        wallsList = api.walls(state)

        # add all coordinates of the map to coordslist
        for x in range(0, width):
            for y in range(0, height):
                coordsList.append((x,y))

        # remove the coordinates of any walls from the list, leaving us with just the accessible paths
        for wall in wallsList:
            for coord in coordsList:
                if wall == coord:
                    coordsList.remove(coord)
        
        # final map of accessible pathways
        pathMap = coordsList

        return pathMap

    # find all waypoints that HAVEN'T been visited yet
    def createWaypointsMap(self, state):
        #calculating waypoints...
        waypoints = []
        pathMap = self.createMapOfAllPaths(state)

        # if the current tile is at a turning point (has two perpendicular squares leading away), make that tile a waypoint
        for tile in pathMap:            
            if (tile[0] + 1, tile[1]) and (tile[0], tile[1] + 1) in pathMap or \
            (tile[0] + 1, tile[1]) and (tile[0], tile[1] - 1) in pathMap or \
            (tile[0] - 1, tile[1]) and (tile[0], tile[1] + 1) in pathMap or \
            (tile[0] - 1, tile[1]) and (tile[0], tile[1] - 1) in pathMap:
                if tile in self.visitedWaypoints:
                    # won't be added to the list of waypoints, if that waypoint has already been visited
                    continue
                else:
                    # add to waypoints as a waypoint to visit
                    waypoints.append(tile)

        return waypoints 

    # This is what gets run in between multiple games
    def final(self, state):
        # reset the state variables from the constructor
        self.visitedWaypoints = []
        self.foodMap = []
        self.previousSteps = []

    # Main loop function - evading ghosts, calculating waypoints, finding best routes and moving pacman
    def getAction(self, state):

        # Get the actions we can try, and remove "STOP" if that is one of them.
        # this was taken from lab practicals
        legal = api.legalActions(state)

        # this was taken from lab practicals
        if Directions.STOP in legal:
            # Pacman won't stop
            legal.remove(Directions.STOP)

        pacman = api.whereAmI(state) # pacman's current location
        
        foods = api.food(state) # all visible food

        theGhosts = api.ghosts(state) # location of the ghosts
 
        # run away from ghosts if they get too close
        for ghost in theGhosts:
            if ghost[0] == pacman[0]:
                
                # pacman can move any direction necessary, except the direction the ghost is in
                # if ghost has same x coordinate as pacman, so is in same column
                if ghost[1] > pacman[1] and Directions.NORTH in legal:
                    legal.remove(Directions.NORTH)
                
                if ghost[1] < pacman[1] and Directions.SOUTH in legal:
                    legal.remove(Directions.SOUTH)
            
            if ghost[1] == pacman[1]:
                
                # ghost has same y coordinate as pacman, so is in same row
                if ghost[0] > pacman[0] and Directions.EAST in legal:
                    legal.remove(Directions.EAST)
                
                if ghost[0] < pacman[0] and Directions.WEST in legal:
                    legal.remove(Directions.WEST)
            
            if ghost[0] > pacman[0] and ghost[1] > pacman[1]:
                
                # ghost is northeast of pacman
                # move in a direction away from ghost
                if Directions.SOUTH in legal:
                    return api.makeMove(Directions.SOUTH, legal)
                if Directions.WEST in legal:
                    return api.makeMove(Directions.WEST, legal)

            elif ghost[0] > pacman[0] and ghost[1] < pacman[1]:

                # ghost is southeast of pacman
                if Directions.NORTH in legal:
                    return api.makeMove(Directions.NORTH, legal)
                if Directions.WEST in legal:
                    return api.makeMove(Directions.WEST, legal)

            elif ghost[0] < pacman[0] and ghost[1] < pacman[1]:

                # ghost is southwest of pacman
                if Directions.NORTH in legal:
                    return api.makeMove(Directions.NORTH, legal)
                if Directions.EAST in legal:
                    return api.makeMove(Directions.EAST, legal)
 
            elif ghost[0] < pacman[0] and ghost[1] > pacman[1]:

                # ghost is northwest of pacman
                if Directions.SOUTH in legal:
                    return api.makeMove(Directions.SOUTH, legal)
                if Directions.EAST in legal:
                    return api.makeMove(Directions.EAST, legal)

        # begin calculations for nearest waypoint
        waypoints = self.createWaypointsMap(state)

        # add all food currently visible to pacman to a map of all food seen (but not yet eaten)
        for food in foods:
            if not food in self.foodMap:
                self.foodMap.append(food)

        # remove any food pacman will have visited (i.e. has eaten)
        while pacman in self.foodMap:
            self.foodMap.remove(pacman)

        # remove any waypoints already visited from the map of waypoints yet to visit
        while pacman in waypoints:
                waypoints.remove(pacman)
                self.visitedWaypoints.append(pacman)

        # if waypoints empty
        if not waypoints:
            # if food map empty
            if not self.foodMap:

                # make the corners within the map (so pacman can go there) and not the corners of the walls
                corners = api.corners(state)
                cornerBLx = corners[0][0] + 1
                cornerBLy = corners[0][1] + 1
                cornerBRx = corners[1][0] - 1
                cornerBRy = corners[1][1] + 1
                cornerTLx = corners[2][0] + 1
                cornerTLy = corners[2][1] - 1
                cornerTRx = corners[3][0] - 1
                cornerTRy = corners[3][1] - 1 
                
                # api.corners gets the WALL coordinates, so cornerPathTiles is the list of coordinates of the paths that pacman can walk on
                cornerPathTiles = [(cornerBLx, cornerBLy), (cornerBRx, cornerBRy), (cornerTLx, cornerTLy), (cornerTRx, cornerTRy)]
                
                # add corners to foodMap to find them when needed
                for corner in cornerPathTiles:
                    if corner in self.visitedWaypoints:
                        continue
                    else:
                        self.foodMap.append(corner)

                # removes any food pacman will have eaten --- this version of this code is specifically for the corners situation
                # needs to run right after the corners are added otherwise an infinite back and forth, as described in report, occurs
                while pacman in self.foodMap:
                    self.foodMap.remove(pacman)

                # as a last resort (pacman has tried waypoints, tried food and tried corners in an attempt to find all food)
                # keep making a random move until foodmap isn't empty anymore (i.e. has seen food)
                while not self.foodMap:
                    return api.makeMove(random.choice(legal),legal)


                # calculating closest corner (corners are added to food map)
                currentNearestCorner = self.foodMap[0]
                currentNearestCornerDistance = util.manhattanDistance(pacman, currentNearestCorner)
            
                # since foodMap now additionally contains corners
                # calculate and update currentNearestCornerDistance
                for corner in self.foodMap:
                    cornerDistance = util.manhattanDistance(pacman, corner)

                    if cornerDistance < currentNearestCornerDistance:
                        currentNearestCorner = corner
                        currentNearestCornerDistance = cornerDistance

                # Calling A* search for finding best route to nearest corner
                previousTiles = self.searchAStar(state, api.whereAmI(state), currentNearestCorner)
                direction = self.getPathToGoal(previousTiles, api.whereAmI(state), currentNearestCorner)
            else: #if waypoints empty, but foodMap not empty
                # calculating closest food
                currentNearestFood = self.foodMap[0]
                currentNearestFoodDistance = util.manhattanDistance(pacman, currentNearestFood)
            
                # calculate and update currentNearestFoodDistance
                for food in self.foodMap:
                    foodDistance = util.manhattanDistance(pacman, food)

                    if foodDistance < currentNearestFoodDistance:
                        currentNearestFood = food
                        currentNearestFoodDistance = foodDistance

                # Calling A* search for finding best route to nearest food
                previousTiles = self.searchAStar(state, api.whereAmI(state), currentNearestFood)
                direction = self.getPathToGoal(previousTiles, api.whereAmI(state), currentNearestFood)

        else:# if waypoints is not empty, calculate nearest waypoint
            currentNearestWaypoint = waypoints[0]
            currentNearestWaypointDistance = util.manhattanDistance(pacman, currentNearestWaypoint)
            
            # calculate and update currentNearestWaypointDistance
            for waypoint in waypoints:
                waypointDistance = util.manhattanDistance(pacman, waypoint)

                if waypointDistance < currentNearestWaypointDistance:
                    currentNearestWaypoint = waypoint
                    currentNearestWaypointDistance = waypointDistance

            # Calling A* search for finding best route to nearest food
            previousTiles = self.searchAStar(state, api.whereAmI(state), currentNearestWaypoint)
            direction = self.getPathToGoal(previousTiles, api.whereAmI(state), currentNearestWaypoint)

        # Pacman moves based on the best route to the goal (found by A* search)
        # if the x coordinates are already the same
        if direction[0][0] == pacman[0]:
            # if pacman is to the left of first tile in best route
            if direction[0][1] > pacman[1]:
                if Directions.NORTH in legal:
                        return api.makeMove(Directions.NORTH, legal)    
                else:
                    # if no move is legal, stop for one move
                    if not legal:
                        return api.makeMove(Directions.STOP, legal)
                    
                    # if legal is not empty, make a random move
                    return api.makeMove(random.choice(legal), legal)

            else:
                if Directions.SOUTH in legal:
                    return api.makeMove(Directions.SOUTH, legal)
                else:
                    # if no move is legal, stop for one move
                    if not legal:
                        return api.makeMove(Directions.STOP, legal)

                    # if legal is not empty, make a random move
                    return api.makeMove(random.choice(legal), legal)

        # if the y coordinates are already the same
        if direction[0][1] == pacman[1]:                     
            if direction[0][0] > pacman[0]:
                if Directions.EAST in legal:
                        return api.makeMove(Directions.EAST, legal)
                else:
                    # if no move is legal, stop for one move
                    if not legal:
                        return api.makeMove(Directions.STOP, legal)

                    # if legal is not empty, make a random move
                    return api.makeMove(random.choice(legal), legal)

            else:
                if Directions.WEST in legal:
                    return api.makeMove(Directions.WEST, legal)
                else:
                    # if no move is legal, stop for one move
                    if not legal:
                        return api.makeMove(Directions.STOP, legal)

                    # if legal is not empty, make a random move
                    return api.makeMove(random.choice(legal), legal)
