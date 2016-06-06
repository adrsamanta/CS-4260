from collections import namedtuple

import search
from capture import SIGHT_RANGE
from captureAgents import CaptureAgent
import random, util
import game
from AgentExternals import *

def createTeam(firstIndex, secondIndex, isRed,
               first='HardwiredAgent', second='HardwiredAgent'):
    """
    This function should return a list of two agents that will form the
    team, initialized using firstIndex and secondIndex as their agent
    index numbers.  isRed is True if the red team is being created, and
    will be False if the blue team is being created.

    As a potentially helpful development aid, this function can take
    additional string-valued keyword arguments ("first" and "second" are
    such arguments in the case of this function), which will come from
    the --redOpts and --blueOpts command-line arguments to capture.py.
    For the nightly contest, however, your team will be created without
    any extra arguments, so you should make sure that the default
    behavior is what you want for the nightly contest.
    """

    # The following line is an example only; feel free to change it.
    return [eval(first)(firstIndex), eval(second)(secondIndex)]


class HLA:
    #due to dependency issues, set the members of HLA inside HardwiredAgent
    pass
    # set each of the HLAs to the method in HardwiredAgent that defines the behavior in that case
    # calling each of these should just require passing in the current object as the self parameter


class UtilAgen(CaptureAgent):
    def registerInitialState(self, gameState):
        """
        This method handles the initial setup of the
        agent to populate useful fields (such as what team
        we're on).

        A distanceCalculator instance caches the maze distances
        between each pair of positions, so your agents can use:
        self.distancer.getDistance(p1, p2)

        IMPORTANT: This method may run for at most 15 seconds.
        """
        # set the members of HLA to the HardwiredAgent methods
        # HLA.goHome = HardwiredAgent.goHomeAction
        # HLA.runAway = None
        # HLA.eatFood = HardwiredAgent.eatFoodAction
        # HLA.chaseEnemy = HardwiredAgent.chasePacmanAction
        # HLA.eatCapsule = HardwiredAgent.eatCapsuleAction

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)
        # set up data repository
        if self.red:
            if not TeamData.RedData:
                TeamData.RedData = TeamData(gameState, self.getTeam(gameState), self.getOpponents(gameState), self)
            self.data = TeamData.RedData

        else:
            if not TeamData.BlueData:
                TeamData.BlueData = TeamData(gameState, self.getTeam(gameState), self.getOpponents(gameState), self)
            self.data = TeamData.BlueData

        self.legalPositions = self.data.legalPositions
        self.offensive = self.data.getOffensive()

        # set up distribution list that will hold belief distributions for agents

    def chooseAction(self, gameState):

        self.knownEnemies = {}  # enemy position, key is enemy index
        self.data.logFood(gameState)
        self.updatePosDist(gameState)
        if self.getScaredMovesRemaining(gameState) > 2:
            self.offensive = True
        elif self.getScaredMovesRemaining(gameState) == 1:
            self.offensive = self.data.getOffensive()
        # print "infer time: ", time()-startTime
        self.displayDistributionsOverPositions(self.data.mDistribs)


    def action_search(self, gamestate):
        UTIL_DISCOUNT = .9
        toVisit = util.Queue()
        #each item on toVisit is a tuple (starting action, gamestate, depth)
        State = namedtuple("starting_action", "gamestate", "depth")

        action_utils = {}
        for action in gamestate.getLegalActions(self.index):
            newgs = gamestate.generateSuccessor(self.index, action)
            toVisit.push(action, newgs, 0)
            action_utils[action]=[]





    #######################################
    #        Features/Weights
    #######################################

    def getUtility(self, gamestate):
        pass

    def getWeights(self, gamestate):
        pass

    def getFeatures(self, gamestate):
        pass


    #######################################
    #        Utility Info Functions
    #######################################

    def getScaredMovesRemaining(self, gameState):
        return gameState.getAgentState(self.index).scaredTimer

    def getFoodEatenByEnemyAgent(self, gameState, agentIndex):
        return gameState.getAgentState(agentIndex).numCarrying

    # checks which side of the board a given position is, returns true if its my side
    def onMySide(self, gameState, pos):
        halfway = gameState.data.food.width / 2
        # copied from halfgrid
        # see comment on halfway in agent data
        if self.red:
            return pos[0] < halfway
        else:
            return pos[0] >= halfway


    ##################################################
    #       POSITION INFERENCE
    ##################################################

    def _setKnownPosDist(self, agentIndex, knownPos):
        self.knownEnemies[agentIndex] = knownPos
        dist = self.getmDistribs(agentIndex)
        dist.clear()
        dist[knownPos] = 1.0

    def _capturedAgent(self, agentIndex):
        self._setKnownPosDist(agentIndex, self.getCurrentObservation().getInitialPosition(agentIndex))

    def getPrevPlayer(self):
        return (self.index - 1) % self.getCurrentObservation().getNumAgents()

    # if this causes a significant slowdown, can just add mDistribs attribute to this class, initialize as ref to data
    def getmDistribs(self, agentIndex):
        return self.data.mDistribs[agentIndex]

    def setDistrib(self, agentIndex, newDistrib):
        i = 3
        for p in newDistrib.keys():
            if p not in self.data.legalPositions:
                pass
        self.data.mDistribs[agentIndex] = newDistrib

    # does inference based on noisy distance to agents and updates opponents distributions
    def positionDistanceInfer(self, agentIndex, gameState=None):
        if not gameState:
            gameState = self.getCurrentObservation()
        # myState=gameState.getAgentState(self.index)

        # noisyDistance = observation
        # emissionModel = busters.getObservationDistribution(noisyDistance)
        myPos = self.getMyPos(gameState)

        noisyDistance = gameState.getAgentDistances()[agentIndex]
        beliefs = self.getmDistribs(agentIndex)
        allPossible = util.Counter()

        for p in self.legalPositions:
            trueDistance = util.manhattanDistance(p, myPos)
            if beliefs[p] == 0:
                # don't need to do anything
                pass
            elif trueDistance <= SIGHT_RANGE:
                # agent would be visible if it were here, so its not here
                allPossible[p] = 0
            # if this position is not on the side of the board the agent is currently on, then the agent isn't at this location
            elif self.onMySide(gameState, p) != gameState.getAgentState(agentIndex).isPacman:
                allPossible[p] = 0
            # NOTE: original code had the check below, but this isn't a good idea because if that prob is 0, the belief
            # for p should be updated with that in mind, so this check is silly.
            # elif gameState.getDistanceProb(trueDistance, noisyDistance)>0: #only do anything if there is any possibility of getting the given noisy distance from this true distance
            else:
                allPossible[p] = beliefs[p] * gameState.getDistanceProb(trueDistance, noisyDistance)

        allPossible.normalize()
        self.setDistrib(agentIndex, allPossible)

    # does inference based on where the agent could move to and updates opponents distributions
    def positionMoveInfer(self, agentIndex, gameState=None, beliefs=None):
        if not gameState:
            gameState = self.getCurrentObservation()
        if not beliefs:
            beliefs = self.getmDistribs(agentIndex)
        # myState=gameState.getAgentState(self.index)
        myPos = self.getMyPos(gameState)

        possiblePositions = util.Counter()

        for pos in self.legalPositions:
            # if the distance is less than SIGHT_RANGE, don't need to do inference on this position, bc we know the agent isn't there
            if beliefs[pos] > 0:
                newPosDist = self.getPositionDistribution(agentIndex, pos, gameState)
                for position, prob in newPosDist.items():
                    possiblePositions[position] += prob * beliefs[pos]

        possiblePositions.normalize()
        return possiblePositions

    # returns a probability distribution for the agents subsequent position, given that it is at curPos
    def getPositionDistribution(self, agentIndex, curPos, gameState=None):
        if not gameState:
            gameState = self.getCurrentObservation()
        neighbors = game.Actions.getLegalNeighbors(curPos, gameState.getWalls())
        probs = {}
        # currently assumes agressively moves towards closest "objective" (food or pacman) with probability .8
        objectives = self.data.defendFoodGrid[-1].asList()
        for i in self.getTeam(gameState):
            if gameState.getAgentState(i).isPacman:
                objectives.append(gameState.getAgentPosition(i))

        minDist = self.getMazeDistance(neighbors[0], objectives[0])
        bestNeighbor = neighbors[0]
        # find the neighbor that is closest to an objective
        for obj in objectives:
            for neighbor in neighbors:
                if self.getMazeDistance(obj, neighbor) < minDist:
                    bestNeighbor = neighbor
        defProb = .8
        otherProbs = (1 - defProb) / (len(neighbors) - 1)
        # set the probability we move to a neighbor that is not bestNeighbor to the correct value
        for n in neighbors:
            probs[n] = otherProbs
        probs[bestNeighbor] = defProb

        return probs

    # compares the most recent food log to the one before it, looking for any food that disappeared
    def checkFood(self):
        if len(self.data.defendFoodGrid) < 2:
            return False
        prevFood = self.data.defendFoodGrid[-2]
        currFood = self.data.defendFoodGrid[-1]
        halfway = currFood.width / 2
        # copied from halfgrid
        if self.red:
            xrange = range(halfway)
        else:
            xrange = range(halfway, currFood.width)
        # TODO: can check numCarrying of previous agent to see if it changed, only do this check if it ate food
        for y in range(currFood.height):
            for x in xrange:
                if prevFood[x][y] and not currFood[x][y]:
                    # food has been eaten in the past move
                    self._setKnownPosDist(self.getPrevPlayer(), (x, y))
                    return True

    # checks if we can see either of the opponents, if so, updates their belief state and doesn't do inference
    # if not, does inference
    def updatePosDist(self, gameState=None):
        if not gameState:
            gameState = self.getCurrentObservation()
        for i in self.getOpponents(gameState):
            if gameState.getAgentPosition(i):  # can observe the given agent

                self._setKnownPosDist(i, gameState.getAgentPosition(i))
            # Only do move infer on the agent right before the current agent, as both agents haven't moved since last call
            # (if this is agent 3, agent 2 just moved, but agent 4 has not moved since agent 1 did inference.
            elif self.getPrevPlayer() == i:  # i is the previous agent
                if self.index == 0 and self.getPreviousObservation() == None:  # this is the first move, don't do inference
                    pass
                else:
                    # check if any food was eaten. If so, don't do inference. if not, do inference
                    if not self.checkFood():
                        self.positionDistanceInfer(i)
                        # positionDistanceInfer returns the new distribution, so update the saved distribution
                        self.setDistrib(i, self.positionMoveInfer(i))
            else:
                # do inference based on distance
                self.positionDistanceInfer(i)