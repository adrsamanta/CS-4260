This folder contains the code for a competitive version of pacman, which is described here: http://rawgit.com/adrsamanta/CS-4260/master/contest/contest.html

I have written several different agents to play this game, one for class, the rest as a personal project. 

The files I have written are:
* myTeam, which is my original attempt at an agent. It attempts to perform a branch and bound search using a utility function to rank the different states. This agent is far from perfect, and will often make confusing or sometimes outright stupid moves.

* newTeam, which is my second attempt at an agent. Instead of using pure utility, this agent picks a "High level action" for each state, and then picks the move it feels is most likely to achieve that high level action.

* UtilTeam, which is in the "SearchRewrite" branch. This is a return to the idea of a pure utility agent. It ranks the spaces it can move to according to their utility and the utility of the subsequent moves it can make. It is still a work in progress. 


* search.py is also mostly written by me (the search algorithms and heuristics are at least). 
