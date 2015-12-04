# myTeam.py
# ---------
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


from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game


#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='TestAgent', second='TestAgent'):
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


##########
# Agents #
##########

class DummyAgent(CaptureAgent):
    """
    A Dummy agent to serve as an example of the necessary agent structure.
    You should look at baselineTeam.py for more details about how to
    create an agent as this is the bare minimum.
    """

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

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)

        '''
        Your initialization code goes here, if you need any.
        '''

    def chooseAction(self, gameState):
        """
        Picks among actions randomly.
        """
        actions = gameState.getLegalActions(self.index)

        '''
        You should change this in your own agent.
        '''

        return random.choice(actions)


class TestAgent(CaptureAgent):
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

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)


    def chooseAction(self, gameState):
        actions = gameState.getLegalActions(self.index)
        myState = gameState.getAgentState(self.index)

        otherState=gameState.getAgentState(self.getOpponents(gameState)[0])
        pos = otherState.getPosition()
        print(pos)

        p2=gameState.data.layout.agentPositions[self.getOpponents(gameState)[0]][-1]
        g=5
        '''
        You should change this in your own agent.
        '''

        return random.choice(actions)


class RealAgent(CaptureAgent):
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

        '''
        Make sure you do not delete the following line. If you would like to
        use Manhattan distances instead of maze distances in order to save
        on initialization time, please take a look at
        CaptureAgent.registerInitialState in captureAgents.py.
        '''
        CaptureAgent.registerInitialState(self, gameState)
        #set up distribution list that will hold belief distributions for agents
        self.mDistribs=[]
        opps=self.getOpponents(gameState)
        for i in range(gameState.getNumAgents()):
            if i in opps:
                dist=util.Counter()
                oppPos=gameState.getInitialAgentPosition(i)
                dist[oppPos]=1.0
                self.mDistribs[i]=dist
            else:
                self.mDistribs[i]=None
        #should be all legal positions
        self.legalPositions = gameState.data.layout.walls.asList(key = False)
        #initialize belief distribution to be 0
        for p in self.legalPositions:
            for i in opps:
                if p not in self.mDistribs[i]:
                    self.mDistribs[i][p]=0.0

    #does inference based on noisy distance to agents and updates opponents distributions
    def positionDistanceInfer(self):
        gameState=self.getCurrentObservation()
        myState=gameState.getAgentState(self.index)

        # noisyDistance = observation
        # emissionModel = busters.getObservationDistribution(noisyDistance)
        myPos = myState.getPosition()

        for i in self.getOpponents(gameState):
            noisyDistance = gameState.getAgentDistances()[i]
            beliefs=self.mDistribs[i]
            allPossible = util.Counter()


            for p in self.legalPositions:
                trueDistance = util.manhattanDistance(p, myPos)
                if beliefs[p]==0:
                    #don't need to do anything
                    pass
                elif trueDistance==0:
                    #no probability of ghost here, bc is current position
                    allPossible[p]=0
                elif gameState.getDistanceProb(trueDistance, noisyDistance)>0: #only do anything if there is any possibility of getting the given noisy distance from this true distance
                    allPossible[p]=beliefs[p]*gameState.getDistanceProb(trueDistance, noisyDistance)
            allPossible.normalize()
            self.mDistribs[i]=allPossible

    #does inference based on where the agent could move to and updates opponents distributions
    def positionMoveInfer(self):
        gameState=self.getCurrentObservation()
        myState=gameState.getAgentState(self.index)
        myPos = myState.getPosition()


        for i in self.getOpponents(gameState):
            possiblePositions = util.Counter()
            beliefs=self.mDistribs[i]
            for pos in self.legalPositions:
                if beliefs[pos] > 0:
                    newPosDist = self.getPositionDistribution(i, pos)
                    for position, prob in newPosDist.items():
                        possiblePositions[position] += prob * beliefs[pos]

            possiblePositions.normalize()
            self.mDistribs[i]=possiblePositions

    #returns a probability distribution for the agents subsequent position, given that it is at curPos
    def getPositionDistribution(self, agentIndex, curPos):

        actions=self.getCurrentObservation().getLegalActions(agentIndex)
        probs={}
        #Currently VERY dumb impl, assumes agent moves randomly
        for action in actions:
            probs[game.Actions.getSuccessor(curPos, action)]=1/len(actions)

        return probs

