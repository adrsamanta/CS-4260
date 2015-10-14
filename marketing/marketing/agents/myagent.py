# You will implement this class
# At the minimum, you need to implement the selectNodes function
# If you override __init__ from the agent superclass, make sure that the interface remains identical as in agent; 
# otherwise your agent will fail

from agent import Agent
import itertools

class MyAgent(Agent):
    
    maxDegree = -1
    sortedNodes = []
    bestSolution = set()
    bestSolutionUtility = -1
    network=None
    degrees=[]

    def selectNodes(self, network, t):
        # select a subset of nodes (up to budget) to seed at current time step t
        # nodes in the network are selected *** BY THEIR INDEX ***
        selected = []
        self.network=network
        self.maxDegree = network.maxDegree()

        #sort nodes by number of neighbors so we can choose nodes on order of neighbors during the branch and bound search
        self.sortedNodes = self.initializeList(network)

        self.degrees=[network.degree(self.sortedNodes[x]) for x in range(2*self.budget)]
        #Best initial estimate to use for initial lower and upper bounds in Branch and Bound
        self.setBestSolution(set(self.sortedNodes[:self.budget]), network)
        
        self.branchAndBound2([],network,set(),(0,network.size()), 0)
        
        return list(self.bestSolution)
    
    def initializeList(self, network):
        return sorted(range(network.size()),key = lambda x: network.degree(x), reverse=True)
    
    #now takes length of neighbors, as opposed to neighbors, to speed calculation
    def calcBounds(self, curr_assignment, lenneighbors, ):
        numRemaining=self.budget-len(curr_assignment)
        max_num_poss=0
        #get a tighter upper bound by adding degrees of biggest non-included nodes 
        for degree, node in zip(self.degrees, self.sortedNodes): #loops is still pretty slow, try to speed up somehow
            if node not in curr_assignment:
                max_num_poss+=degree
                numRemaining-=1
            if numRemaining==0:
                break

        return lenneighbors, lenneighbors+max_num_poss
        
    #Calculate the utility of a given solution
    def calcUtility(self, solution):
        neighbors = set(solution)
        network=self.network
        for node in solution:
            neighbors = neighbors.union(set(network.getNeighbors(node)))
        return len(neighbors)

    #Set the best Solution and best Solution Utility values
    def setBestSolution(self, solution, network, utility=0):
        self.bestSolution = solution
        self.bestSolutionUtility = utility if utility else self.calcUtility(solution)
    
    #Branch and Bound search of the graph, prunes subrees when their upper bound is not greater than the best Solution Utility or the utility of their parent
    def branchAndBound2(self, curr_assignment, network, neighbors, bounds, nodeListI):
        nodeList=self.sortedNodes
        lenNeighbors=bounds[0]
        #Base case when we have an assignment the same size of the budget
        if len(curr_assignment) == self.budget:
            if lenNeighbors > self.bestSolutionUtility:
                self.setBestSolution(curr_assignment, network, lenNeighbors)
            return
        for i in range(nodeListI, len(nodeList)):
            node=nodeList[i]
            if node in curr_assignment:
                continue
            else:
                new_neighbors = neighbors.union(set(network.getNeighbors(node)))
                new_neighbors.add(node)
                new_assignment = curr_assignment + [node]
                node_bounds = self.calcBounds(new_assignment, len(new_neighbors))
                if node_bounds[1] > bounds[0] and node_bounds[1]>self.bestSolutionUtility:
                    self.branchAndBound2(new_assignment, network, new_neighbors, node_bounds, i+1)

    
    def display():
        print "Agent ID ", self.id

