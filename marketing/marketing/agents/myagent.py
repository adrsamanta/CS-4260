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
        #print "max degree", self.maxDegree
        self.sortedNodes = self.initializeList(network)
        #track the upper 2*budget degrees, to make finding these faster in calcBounds
        #track 2*budget because will never need more than that in bounds calc
        self.degrees=[network.degree(self.sortedNodes[x]) for x in range(2*self.budget)]
        self.setBestSolution(set(self.sortedNodes[:self.budget]), network)
        
        #print "Best Initial Solution:", self.bestSolution
        #print "best utility: ", self.bestSolutionUtility

        #self.branchAndBound([],network,set(),(0,network.size()), self.sortedNodes[:])
        self.branchAndBound2([],network,set(),(0,network.size()), 0)
        #self.exhaustive([],network,set(), 0)
        
        # your code goes here
        #print "Best Final Solution:", self.bestSolution
        #print "calc value for BFSU", self.calcUtility(self.bestSolution)
        #neighbors = set()
        #network=self.network
        #for node in self.bestSolution:
        #    neighbors = neighbors.union(set(network.getNeighbors(node)))
        #print "neighbors=", neighbors
        #print "best Final utility: ", self.bestSolutionUtility
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
        #return lenneighbors,lenneighbors+(self.budget - len(curr_assignment)) * self.maxDegree
        
    def calcUtility(self, solution):
        neighbors = set(solution)
        network=self.network
        for node in solution:
            neighbors = neighbors.union(set(network.getNeighbors(node)))
        return len(neighbors)
    
    def getUtility(self, neighbors, curr_assignment):
        return len(neighbors)

    def setBestSolution(self, solution, network, utility=0):
        self.bestSolution = solution
        self.bestSolutionUtility = utility if utility else self.calcUtility(solution)
    
    #NOTE: curr_assignment is now a list, because offered slight speedup
    #NOTE: currently has a bug, will not check all assignments. 
    #def branchAndBound(self, curr_assignment, network, neighbors, bounds, nodeList):
    #    #print "Entered B&B"
    #    lenNeighbors=bounds[0]
    #    if len(curr_assignment) == self.budget:
    #        #print "found assignment of 4:", curr_assignment
    #        #print "has utility", len(neighbors)
    #        if lenNeighbors > self.bestSolutionUtility:
    #            self.setBestSolution(curr_assignment, network, lenNeighbors)
    #        return
    #    for node in nodeList:
    #        #print "Checking node:", node
    #        if node in curr_assignment:
    #            continue
    #        else:
    #            nodeList.remove(node)
    #            new_neighbors = neighbors.union(set(network.getNeighbors(node)))
    #            new_assignment = curr_assignment + [node]
    #            node_bounds = self.calcBounds(new_assignment, len(new_neighbors))
    #            #print "checking assignment with bounds", node_bounds
    #            if node_bounds[1] > bounds[0] and node_bounds[1]>self.bestSolutionUtility:
    #                self.branchAndBound(new_assignment, network, new_neighbors, node_bounds, nodeList[:])
    
    #should properly check all combinations, now don't need to pass/copy the list
    #not optimal
    def branchAndBound2(self, curr_assignment, network, neighbors, bounds, nodeListI):
        #print "Entered B&B"
        nodeList=self.sortedNodes
        lenNeighbors=bounds[0]
        if len(curr_assignment) == self.budget:
            #print "found assignment of 4:", curr_assignment
            #print "has utility", len(neighbors)
            if lenNeighbors > self.bestSolutionUtility:
                self.setBestSolution(curr_assignment, network, lenNeighbors)
            return
        for i in range(nodeListI, len(nodeList)):
            node=nodeList[i]
            #print "Checking node:", node
            if node in curr_assignment:
                continue
            else:
                #nodeList.remove(node)
                new_neighbors = neighbors.union(set(network.getNeighbors(node)))
                new_neighbors.add(node)
                new_assignment = curr_assignment + [node]
                node_bounds = self.calcBounds(new_assignment, len(new_neighbors))
                #print "checking assignment with bounds", node_bounds
                if node_bounds[1] > bounds[0] and node_bounds[1]>self.bestSolutionUtility:
                    self.branchAndBound2(new_assignment, network, new_neighbors, node_bounds, i+1)
    
    #non optimal, but is supposed to check all combinations. 
    def exhaustive(self, curr_assignment, network, neighbors, nodeListI):
        nodeList=range(network.size())
        for assignment in itertools.combinations(nodeList, 4):
            if self.calcUtility(assignment)>self.bestSolutionUtility:
                self.setBestSolution(assignment, network)

    
    def display():
        print "Agent ID ", self.id

