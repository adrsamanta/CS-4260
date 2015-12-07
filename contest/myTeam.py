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
from capture import GameState, SIGHT_RANGE
from collections import namedtuple
import time

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first='RealAgent', second='RealAgent'):
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





class TeamData:
    RedData=None
    BlueData=None
    def __init__(self, gameState, team, opps, agent):
        self.team=team
        self.mAgent=agent
        self.offensive = True
        self.mDistribs=[None]*gameState.getNumAgents()
        #opps=self.team[0].getOpponents(gameState)
        for i in range(gameState.getNumAgents()):
            if i in opps:
                dist=util.Counter()
                oppPos=gameState.getInitialAgentPosition(i)
                dist[oppPos]=1.0
                self.mDistribs[i]=dist
            else:
                self.mDistribs[i]=None
        #should be all legal positions
        self.legalPositions = gameState.data.layout.walls.asList(key = False) #TODO: NEEDS TO BE CHECKED
        self.defendFoodGrid=[]
        halfway = agent.getFood(gameState).width / 2
        self.borderPositions=[(halfway, y) for y in range(agent.getFood(gameState).height) if not gameState.hasWall(halfway, y)]

        self.borderDistances={}
        self.calcBorderDistances(gameState)

    def calcBorderDistances(self, gameState):
        grid = gameState.getWalls()
        halfway = grid.width / 2
        if not self.mAgent.red:
            xrange = range(halfway)
        else:
            xrange = range(halfway, grid.width)

        for x in xrange:
            for y in range(grid.height):
                if not grid[x][y]:
                    self.borderDistances[(x, y)]= min(self.mAgent.getMazeDistance((x, y), borderPos) for borderPos in self.borderPositions)

    def logFood(self, gameState):
        self.defendFoodGrid.append(self.mAgent.getFoodYouAreDefending(gameState))

    def getOffensive(self):
        self.offensive = not self.offensive
        return self.offensive

#TODO: scared timer stuff in agent states, look at line 573 in capture
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
        #TODO: figure out how to avoid doing mazeDistances for both agents
        #TODO: Look into stealing mazeDistances from other team if possible
        CaptureAgent.registerInitialState(self, gameState)
        #set up data repository
        if self.red:
            if not TeamData.RedData:
                TeamData.RedData=TeamData(gameState, self.getTeam(gameState), self.getOpponents(gameState), self)
            self.data=TeamData.RedData

        else:
            if not TeamData.BlueData:
                TeamData.BlueData=TeamData(gameState, self.getTeam(gameState), self.getOpponents(gameState), self)
            self.data=TeamData.BlueData

        self.legalPositions=self.data.legalPositions
        self.offensive = self.data.getOffensive()
        #set up distribution list that will hold belief distributions for agents

    def chooseAction(self, gameState):
        #print(pos)

        #get a list of actions
        #update belieft distribution about enemy positions
        #is anyone scared
        #food far away from know enemies
        #food close to enemies but near capsule
        #if theres one action:  update belief distribution and take that action
        #
        #else calculate utility function for each action
        #
        '''
        You should change this in your own agent.
        '''
        #startTime=time.time()
        self.knownEnemies={}
        self.data.logFood(gameState)
        self.updatePosDist(gameState)
        #print "infer time: ", time.time()-startTime
        self.displayDistributionsOverPositions(self.data.mDistribs)
        return self.actionSearch(self.index, gameState)
        # return random.choice(gameState.getLegalActions(self.index))
        #return self.offensiveReflex(gameState)

    def actionSearch(self, agentIndex, gameState):
        ##do a breadth first search until time runs out
        #dictionary to keep track of visited spots so we can look up their utility in constant time
        visited = dict()
        #set to keep track of
        visitedInSequence = set()
        #queue of action states to visit
        toVisit = util.Queue()
        actions = []
        #lower bound and upper bound set to arbitrary values for testing purposes
        upperBound = 999999
        lowerBound = -99999
        #start time so we can terminate before 1 second time limit
        start_time = time.time()
        debug = False
        #way to keep track of best action so far????
        bestActionSequence = gameState.getLegalActions(self.index)
        bestActionSequenceUtility = None
        #make sure this does a deep copy
        #enemy_belief_states = list(self.data.mDistribs)
        currGameStateFeatures = self.getFeatures(gameState)
        #named tuple for readability
        State = namedtuple('State', 'agentIndex actions visitedInActionSequence currGameState currGameStateFeatures positions')
        toVisit.push((State(agentIndex, actions, visitedInSequence, gameState, currGameStateFeatures, [self.getMyPos(gameState)]), 0))
        #using a constant of .75 seconds for now
        while time.time() - start_time < .75 and not toVisit.isEmpty():
            curr_state, curr_utility = toVisit.pop()

            for next_action in curr_state.currGameState.getLegalActions(self.index):
                if next_action=="Stop":
                    continue
                next_game_state = curr_state.currGameState.generateSuccessor(curr_state.agentIndex, next_action)

                if debug:
                    #print("curr state actions: ", curr_state.actions)
                    print("next action: ", next_action)
                if len(curr_state.actions) > 0 and isinstance(curr_state.actions[0], list):
                    print("curr state actions: ", curr_state.actions)
                    print("legal actions: ", curr_state.currGameState.getLegalActions(self.index))
                    print("next action: ", next_action)
                new_actions = [action for action in curr_state.actions]
                new_actions.append(next_action)

                if isinstance(next_action, list):
                    print("next_action: ", next_action)
                    print("legal actions: ", curr_state.currGameState.getLegalActions(self.index))

                for action in new_actions:
                    if isinstance(action, list):
                        print("curr state actions: ", curr_state.actions)
                        print("legal actions: ", curr_state.currGameState.getLegalActions(self.index))
                        print("next action: ", next_action)
                #update enemy belief states based on move
                #Array index out of bounds exception thrown in getPositionDistribution so using dummy array instead
                #enemy_belief_states = [self.getPositionDistribution(i, next_game_state.getAgentPosition(self.index), next_game_state) for i in self.getOpponents(next_game_state)]
                curr_state_features = curr_state.currGameStateFeatures
                #I dont think this dictionary works because all state objects will be different
                #Either need to define a new dictionary class that compares internal values of states
                #Or store more specific information in the dictionary - such as index positions
                next_state_features = self.getFeatures(next_game_state)


                if next_game_state in visited:
                    state_utility = visited[next_game_state]
                else:
                    state_utility = self.Utility(next_game_state, next_state_features)
                    visited[next_game_state] = state_utility
                if next_state_features["foodEatenBySelf"]:
                    print "hallejueah!"
                #do we want to do the bounds check on just the utility of that state, or the state's utility + past_utility
                #need a way to calculate upper and lower bound
                if self.estimatedUtilityWillIncrease(curr_state_features, next_state_features):
                    total_utility = .8**len(new_actions)*state_utility + curr_utility
                    if debug:
                        print("new actions: ", new_actions, " utility: ", total_utility)
                    if not bestActionSequenceUtility or total_utility/len(new_actions) > bestActionSequenceUtility:
                        bestActionSequenceUtility = total_utility/len(new_actions)
                        bestActionSequence = new_actions
                    toVisit.push((State(agentIndex, new_actions, visitedInSequence.add(next_action), next_game_state, next_state_features, total_utility), total_utility/len(new_actions)))
        #Currently first action in action sequence with the highest utility
        #Should we remember the entire sequence to make later computations faster
        print "bestActionSequence:", bestActionSequence
        return bestActionSequence[0]

    #Doing a recalculation of minimum distance to food here - we can get rid of this later
    def estimatedUtilityWillIncrease(self, curr_state_features, next_state_features):
        if self.offensive:
            return next_state_features["foodDist"] < curr_state_features["foodDist"] or next_state_features["foodEatenBySelf"] > next_state_features["foodEatenBySelf"]
        if next_state_features["numEnemyPacmen"] == 0:
            return True
        if curr_state_features["numEnemyPacmen"] == 0:
            return False
        for i in range(min(len(curr_state_features["distToEnemyPacman"]),len(next_state_features["distToEnemyPacman"]))):
            if next_state_features["distToEnemyPacman"][i] < curr_state_features["distToEnemyPacman"][i]:
                return True
        return False


    def Utility(self, gameState, features):

        #features = self.getFeatures(gameState)
        weights = self.getWeights(features, gameState)
        if len(features)!=len(weights):
            print("AWKO TACO")
        utility = 0
        for feature, feature_value in features.items():
            if not feature_value or not weights[feature]:
                continue
            if isinstance(feature_value,list):
                if feature=="distToEnemyPacman":
                    for i in range(len(feature_value)):
                        if not feature_value[i]:
                            print "featurevalue[i] was 0", feature, feature_value
                            continue
                        utility+=6./feature_value[i] * weights[feature]
                else:
                    for i in range(len(feature_value)):
                        utility += feature_value[i] * weights[feature]
            elif feature=="foodDist":
                utility+=5./feature_value*weights[feature]
            elif feature=="distToNearestCapsule":
                utility+=4./feature_value*weights[feature] #TODO: make this better
            else:
                utility += feature_value * weights[feature]

        return utility

    #weight on a -5 to 5 scale
    def getWeights(self, features, gameState):
        def getEnemyGhostDistanceDistrib(distance):

            if 0< distance <=2:
                return -5
            elif 2<distance<=4:
                return -1
            elif 4<distance<=6:
                return -.2
            else:
                return 0

        weights = util.Counter()
        weights["foodDist"] = 3 if self.offensive else 2
        weights["distToNearestCapsule"]= .5 #TODO: make this better
        weights["numEnemyPacmen"] = 0
        weights["distToEnemyPacman"] = 2 if self.offensive else 3 if features["distToEnemyPacman"] > features["scaredMovesRemaining"] else -3
        weights["numEnemyGhost"] = 0
        weights["distToEnemyGhosts"] = 0 if not gameState.getAgentState(self.index).isPacman else 1 \
                    if features["scaredEnemyMovesRemaining"] <= features["distToEnemyGhost"] else getEnemyGhostDistanceDistrib(features["distToEnemyGhosts"])
        weights["score"] = .5
        weights["movesRemaing"] = 0
        weights["scaredMovesRemaining"] = 0
        weights["scaredEnemyMovesRemaining"]=0
        weights["foodEatenBySelf"] = 5*weights["foodDist"]
        weights["enemyPacmanFood"] = 0
        weights["distToHome"] = max(features["foodEatenBySelf"]/-4., -5) if features["distToHome"] < features["movesRemaining"] else -5 #Tweak value later
        return weights



    def getFeatures(self, gameState):
        features=util.Counter()
        myPos= self.getMyPos(gameState)
        enemies=[gameState.getAgentState(i) for i in self.getOpponents(gameState)]


        foodList=self.getFood(gameState).asList()
        minDist=self.getMazeDistance(myPos, foodList[0])
        for pos in foodList:
            dist=self.getMazeDistance(myPos, pos)
            if dist<minDist:
                minDist=dist
                if dist==1:
                    break
        features["foodDist"]=minDist

        features["numEnemyPacmen"]=0
        features["distToEnemyPacman"]=[]
        features["numEnemyGhost"]=0
        features["distToEnemyGhost"]=0
        for i, enemy in zip(self.getOpponents(gameState), enemies):
            if enemy.isPacman:
                features["numEnemyPacmen"]+=1
                features["distToEnemyPacman"].append(self.getDistanceToEnemy(gameState, i))

            else:
                features["numEnemyGhost"]+=1
                if features["distToEnemyGhost"]:
                    features["distToEnemyGhost"]=min(self.getDistanceToEnemy(gameState, i), features["distToEnemyGhost"])

        features["score"]=self.getScore(gameState)
        features["movesRemaining"]=gameState.data.timeleft

        features["distToNearestCapsule"]=self.getDistToNearestCapsule(gameState)
        features["scaredMovesRemaining"]=self.getScaredMovesRemaining(gameState)
        features["scaredEnemyMovesRemaining"] = self.getEnemyAgentScaredMovesRemaining(gameState)
        features["foodEatenBySelf"]=self.getFoodEatenBySelf(gameState)
        features["enemyPacmanFood"]=[]
        for i in self.getOpponents(gameState):
            features["enemyPacmanFood"]=self.getFoodEatenByEnemyAgent(gameState, i)
        features["distToHome"]=self.getDistanceToHomeSide(gameState)

        return features

    def getDistToNearestCapsule(self, gameState):
        if gameState.isOnRedTeam(self.index):
            return min([self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), cap) for cap in gameState.getRedCapsules()])
        else:
            return min([self.getMazeDistance(gameState.getAgentState(self.index).getPosition(), cap) for cap in gameState.getBlueCapsules()])

    def getScaredMovesRemaining(self, gameState):
        return gameState.getAgentState(self.index).scaredTimer

    def getEnemyAgentScaredMovesRemaining(self, gameState):
        return gameState.getAgentState(self.getOpponents(gameState)[0]).scaredTimer

    def getFoodEatenByEnemyAgent(self, gameState, agentIndex):
        return gameState.getAgentState(agentIndex).numCarrying

    #food in our stomach
    def getFoodEatenBySelf(self, gameState):
        return gameState.getAgentState(self.index).numCarrying

    #gain of going to home side

    def getDistanceToHomeSide(self, gameState):
        myPos=self.getMyPos(gameState)
        return self.data.borderDistances[myPos] if myPos in self.data.borderDistances else 0



    def getMyPos(self, gameState):
        return gameState.getAgentPosition(self.index)

    def getDistanceToEnemy(self, gameState, enemyIndex):
        if enemyIndex in self.knownEnemies:
            return self.getMazeDistance(self.getMyPos(gameState), self.knownEnemies[enemyIndex])
        else:
            #list of all distances that have prob>=.5
            dists=[self.getMazeDistance(self.getMyPos(gameState), pos)
                   for pos, prob in self.getmDistribs(enemyIndex).items() if prob>=.5]
            if len(dists)==0:
                bestPos= max(self.getmDistribs(enemyIndex).items(),
                           key= lambda x : x[1])
                return self.getMazeDistance(self.getMyPos(gameState), bestPos)
            # dists=[]
            # maxProb=0
            # maxProbPos=None
            # for pos, prob in self.getmDistribs(enemyIndex).items():
            #     if prob>=.5:
            #         try:
            #             x,y=pos
            #
            #         except TypeError:
            #             print("pos not iterable")
            #             print pos
            #         try:
            #             x, y=self.getMyPos(gameState)
            #         except TypeError:
            #             print "myPos not iterable"
            #         dists.append(self.getMazeDistance(pos, self.getMyPos(gameState)))
            #     else:
            #         if prob>=maxProb:
            #             maxProbPos=pos
            # if len(dists)==0:
            #     print "no good estimate, returning:", maxProbPos
            #     if not maxProbPos:
            #         pass
            #     return maxProbPos
            return sum(dists)/len(dists)


    def _setKnownPosDist(self, agentIndex, knownPos):
        self.knownEnemies[agentIndex]=knownPos
        dist=self.getmDistribs(agentIndex)
        dist.clear()
        dist[knownPos]=1.0

    def _capturedAgent(self, agentIndex):
        self._setKnownPosDist(agentIndex, self.getCurrentObservation().getInitialPosition(agentIndex))

    def getPrevPlayer(self):
        return (self.index-1)%self.getCurrentObservation().getNumAgents()

    #if this causes a significant slowdown, can just add mDistribs attribute to this class, initialize as ref to data
    def getmDistribs(self, agentIndex):
        return self.data.mDistribs[agentIndex]

    def setDistrib(self, agentIndex, newDistrib):
        self.data.mDistribs[agentIndex]=newDistrib

    #checks which side of the board a given position is, returns true if its my side
    def onMySide(self, gameState, pos):
        halfway = self.getFood(gameState).width / 2
        #copied from halfgrid
        if self.red:
            return pos[0] < halfway
        else:
            return pos[0] > halfway


    #does inference based on noisy distance to agents and updates opponents distributions
    def positionDistanceInfer(self, agentIndex, gameState=None):
        if not gameState:
            gameState=self.getCurrentObservation()
        #myState=gameState.getAgentState(self.index)

        # noisyDistance = observation
        # emissionModel = busters.getObservationDistribution(noisyDistance)
        myPos = self.getMyPos(gameState)

        noisyDistance = gameState.getAgentDistances()[agentIndex]
        beliefs= self.getmDistribs(agentIndex)
        allPossible = util.Counter()


        for p in self.legalPositions:
            trueDistance = util.manhattanDistance(p, myPos)
            if beliefs[p]==0:
                #don't need to do anything
                pass
            elif trueDistance<=SIGHT_RANGE:
                #agent would be visible if it were here, so its not here
                allPossible[p]=0
            #if this position is not on the side of the board the agent is currently on, then the agent isn't at this location
            elif self.onMySide(gameState, p) != gameState.getAgentState(agentIndex).isPacman:
                allPossible[p]=0
            #NOTE: original code had the check below, but this isn't a good idea because if that prob is 0, the belief
            #for p should be updated with that in mind, so this check is silly.
            #elif gameState.getDistanceProb(trueDistance, noisyDistance)>0: #only do anything if there is any possibility of getting the given noisy distance from this true distance
            else:
                allPossible[p]=beliefs[p]*gameState.getDistanceProb(trueDistance, noisyDistance)

        allPossible.normalize()
        self.setDistrib(agentIndex, allPossible)

    #does inference based on where the agent could move to and updates opponents distributions
    def positionMoveInfer(self, agentIndex, gameState=None, beliefs=None):
        if not gameState:
            gameState=self.getCurrentObservation()
        if not beliefs:
            beliefs= self.getmDistribs(agentIndex)
        #myState=gameState.getAgentState(self.index)
        myPos= self.getMyPos(gameState)

        possiblePositions = util.Counter()

        for pos in self.legalPositions:
            #if the distance is less than SIGHT_RANGE, don't need to do inference on this position, bc we know the agent isn't there
            if beliefs[pos] > 0:
                newPosDist = self.getPositionDistribution(agentIndex, pos, gameState)
                for position, prob in newPosDist.items():
                    possiblePositions[position] += prob * beliefs[pos]

        possiblePositions.normalize()
        return possiblePositions


    #returns a probability distribution for the agents subsequent position, given that it is at curPos
    def getPositionDistribution(self, agentIndex, curPos, gameState=None):
        if not gameState:
            gameState=self.getCurrentObservation()
        neighbors = game.Actions.getLegalNeighbors(curPos, gameState.getWalls())
        probs={}
        #currently assumes agressively moves towards closest "objective" (food or pacman) with probability .8
        objectives=self.data.defendFoodGrid[-1].asList()
        for i in self.getTeam(gameState):
            if gameState.getAgentState(i).isPacman:
                objectives.append(gameState.getAgentPosition(i))

        minDist=self.getMazeDistance(neighbors[0], objectives[0])
        bestNeighbor=neighbors[0]
        #find the neighbor that is closest to an objective
        for obj in objectives:
            for neighbor in neighbors:
                if self.getMazeDistance(obj, neighbor)<minDist:
                    bestNeighbor=neighbor
        defProb=.8
        otherProbs=(1-defProb)/(len(neighbors)-1)
        #set the probability we move to a neighbor that is not bestNeighbor to the correct value
        for n in neighbors:
            probs[n]=otherProbs
        probs[bestNeighbor]=defProb

        return probs

    #compares the most recent food log to the one before it, looking for any food that disappeared
    def checkFood(self):
        if len(self.data.defendFoodGrid) < 2:
            return False
        prevFood=self.data.defendFoodGrid[-2]
        currFood=self.data.defendFoodGrid[-1]
        halfway = currFood.width / 2
        #copied from halfgrid
        if self.red:
            xrange = range(halfway)
        else:
            xrange = range(halfway, currFood.width)
        #TODO: can check numCarrying of previous agent to see if it changed, only do this check if it ate food
        for y in range(currFood.height):
            for x in xrange:
                if prevFood[x][y] and not currFood[x][y]:
                    #food has been eaten in the past move
                    self._setKnownPosDist(self.getPrevPlayer(), (x,y))
                    return True

    #checks if we can see either of the opponents, if so, updates their belief state and doesn't do inference
    #if not, does inference
    def updatePosDist(self, gameState=None):
        if not gameState:
            gameState=self.getCurrentObservation()
        for i in self.getOpponents(gameState):
            if gameState.getAgentPosition(i): #can observe the given agent
                self._setKnownPosDist(i, gameState.getAgentPosition(i))
            #Only do move infer on the agent right before the current agent, as both agents haven't moved since last call
            #(if this is agent 3, agent 2 just moved, but agent 4 has not moved since agent 1 did inference.
            elif self.getPrevPlayer()==i: #i is the previous agent
                if self.index==0 and self.getPreviousObservation()==None: #this is the first move, don't do inference
                    pass
                else:
                    #check if any food was eaten. If so, don't do inference. if not, do inference
                    if not self.checkFood():
                        self.positionMoveInfer(i)
                        #positionDistanceInfer returns the new distribution, so update the saved distribution
                        self.setDistrib(i, self.positionMoveInfer(i))
            else:
                #do inference based on distance
                self.positionDistanceInfer(i)



    ###### BEGIN OFFENSIVE CODE ##########
    # def offensiveReflex(self, gameState):
    #     actions = gameState.getLegalActions(self.index)
    #     values = [self.evaluate(gameState, a) for a in actions]
    #     # print 'eval time for agent %d: %.4f' % (self.index, time.time() - start)
    #
    #     maxValue = max(values)
    #     bestActions = [a for a, v in zip(actions, values) if v == maxValue]
    #
    #     foodLeft = len(self.getFood(gameState).asList())
    #
    #     if foodLeft <= 2:
    #       bestDist = 9999
    #       for action in actions:
    #         successor = self.getSuccessor(gameState, action)
    #         pos2 = successor.getAgentPosition(self.index)
    #         dist = self.getMazeDistance(self.start,pos2)
    #         if dist < bestDist:
    #           bestAction = action
    #           bestDist = dist
    #       return bestAction
    #
    #     return random.choice(bestActions)
    #
    # def getSuccessor(self, gameState, action):
    #     """
    #     Finds the next successor which is a grid position (location tuple).
    #     """
    #     successor = gameState.generateSuccessor(self.index, action)
    #     pos = successor.getAgentState(self.index).getPosition()
    #     if pos != util.nearestPoint(pos):
    #       # Only half a grid position was covered
    #       return successor.generateSuccessor(self.index, action)
    #     else:
    #       return successor
    #
    # def evaluate(self, gameState, action):
    #     """
    #     Computes a linear combination of features and feature weights
    #     """
    #     features = self.getFeatures(gameState, action)
    #     weights = self.getWeights(gameState, action)
    #     return features * weights
    #
    # def getFeatures(self, gameState, action):
    #     features = util.Counter()
    #     successor = self.getSuccessor(gameState, action)
    #     foodList = self.getFood(successor).asList()
    #     features['successorScore'] = -len(foodList)#self.getScore(successor)
    #
    #     # Compute distance to the nearest food
    #
    #     if len(foodList) > 0: # This should always be True,  but better safe than sorry
    #       myPos = successor.getAgentState(self.index).getPosition()
    #       minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
    #       features['distanceToFood'] = minDistance
    #     return features
    #
    # def getWeights(self, gameState, action):
    #     return {'successorScore': 100, 'distanceToFood': -1}
    #
    # ############END OFFENSIVE REFLEX CODE#################





    ############END OFFENSIVE REFLEX CODE#################