# input: set of agents to target [by id]; # of simulations

# check that |agents| <= K; otherwise, return 0 adopters

# output: average # of adopters

from network.network import Network

from agents.agent import Agent

import ConfigParser
import copy
import math
import time
import random

from util import ConfigSectionMap, EnforceSelectionFeasibility

Config = ConfigParser.ConfigParser()
Config.read("params.conf")

# number of time steps
timeSteps = int(ConfigSectionMap(Config, "SimulationParameters")["timesteps"])

# interest rate (at which remaining budget increases in every step)
interest = float(ConfigSectionMap(Config, "SimulationParameters")["interest"])

# name of cascade model to use
cascadeModuleName = ConfigSectionMap(Config, "SimulationParameters")["cascademodule"]

# name of agent module to use
agentModuleName = ConfigSectionMap(Config, "SimulationParameters")["agentmodule"]

import importlib
agentModule = importlib.import_module(agentModuleName)
cascadeModule = importlib.import_module(cascadeModuleName)

# initialize the network and cascade (diffusion) process model
cascade = cascadeModule.CascadeModel(Config)

avetime = 0
aveutility = 0
numSamples = 10

for s in range(numSamples):
    random.seed(s)

    nw = Network(Config,cascade,random)
    
    # initialize agent
    agent =  agentModule.MyAgent(0,Config)
    
    # run simulations
    budget = int(ConfigSectionMap(Config, "AgentParameters")["budget"])
    
    start = time.time()
    for t in range(0,timeSteps):
        selected = EnforceSelectionFeasibility(int(math.floor(budget)), agent.selectNodes(copy.deepcopy(nw),t))
        adopters = nw.update(selected)
        
        # subtract selection from remaining budget
        budget -= len(selected)
        budget *= 1+interest
        
    end = time.time()
    aveutility += float(adopters)
        
    avetime += end - start

avetime /= numSamples
aveutility /= numSamples

print "time = ",avetime,"; utility = ",aveutility
    


