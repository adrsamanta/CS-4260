# You will implement this class
# At the minimum, you need to implement the selectNodes function
# If you override __init__ from the agent superclass, make sure that the interface remains identical as in agent; 
# otherwise your agent will fail

from agent import Agent
    
class MyAgent(Agent):
    
    maxDegree = -1
    sortedNodes = []
    bestSolution = set()
    bestSolutionUtility = -1

    def selectNodes(self, network, t):
        # select a subset of nodes (up to budget) to seed at current time step t
        # nodes in the network are selected *** BY THEIR INDEX ***
        selected = []
        self.maxDegree = network.maxDegree()
        sortedNodes = self.initializeList(network)
        self.setBestSolution(set(sortedNodes[:self.budget]), network)
        self.branchAndBound(set(),network,set(),(0,network.size()))
        #find max degree
        #do branch and bound search

        # your code goes here
        
        return list(self.bestSolution)
    
    def initializeList(self, network):
        return sorted(range(network.size()),key = lambda x: network.degree(x), reverse=True)
    
    def calcBounds(self, curr_assignment, neighbors):
        return (len(neighbors) + len(curr_assignment),(self.budget - len(curr_assignment)) * self.maxDegree)
    
    def calcUtility(self, network, solution):
        neighbors = set()
        for node in solution:
            neighbors = neighbors.union(set(network.getNeighbors(node)))
        return len(neighbors) + len(solution)
    
    def setBestSolution(self, solution, network, utility=0):
        self.bestSolution = solution
        self.bestSolutionUtility = utility if utility else self.calcUtility(network, solution)

    def branchAndBound(self, curr_assignment, network, neighbors, bounds):
        if len(curr_assignment) == self.budget:
            if len(neighbors) + len(curr_assignment) > self.bestSolutionUtility:
                self.setBestSolution(curr_assignment, network)
        for node in self.sortedNodes:
            if node in curr_assignment:
                continue
            else:
                new_neighbors = neighbors.union(set(network.getNeighbors(node)))
                new_assignment = curr_assignment.union(set(node))
                node_bounds = self.calcBounds(new_assignment, new_neighbors)
                if node_bounds[1] > bounds[0] and node_bounds[1]>self.bestSolutionUtility:
                    branchAndBound(new_assignment, network, new_neighbors, node_bounds)

    
    
    def display():
        print "Agent ID ", self.id

