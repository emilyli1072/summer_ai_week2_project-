# search.py
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


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

from audioop import reverse
import util
import numpy as np
class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()
visited = {}
def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]
def depthFirstSearch(problem: SearchProblem):
    from game import Directions
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
    """
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors((25, 15)))
    visited = {}
    nodes = util.Stack()
    min = []
    nodes.push([problem.getStartState(), 0, []])
    visited[problem.getStartState()]=0
    while nodes.isEmpty() == False:
        cur = nodes.pop()
        if problem.isGoalState(cur[0]):
            if len(min)>cur[1] or min==[]:
                min = cur[2].copy()
        for i in problem.getSuccessors(cur[0]):
            if (not i[0] in visited or cur[1]+i[2]<visited[i[0]]):
                visited[i[0]] = cur[1]+i[2]
                cur[2].append(i[1])
                nodes.push([i[0], i[2]+cur[1], cur[2].copy()])
                cur[2].pop()
    ans = []
    for i in min:
        ans.append(i)
    print(ans)
    return ans

def backtracking(visited, cur_pos, ans):
    if visited[cur_pos][1] == cur_pos[1]+1:
        ans.append("South")
        return backtracking(visited, (visited[cur_pos][0], visited[cur_pos][1]), ans)
    elif visited[cur_pos][1] == cur_pos[1]-1:
        ans.append("North")
        return backtracking(visited, (visited[cur_pos][0], visited[cur_pos][1]), ans)
    elif visited[cur_pos][0] == cur_pos[0]+1:
        ans.append("West")
        return backtracking(visited, (visited[cur_pos][0], visited[cur_pos][1]), ans)
    elif visited[cur_pos][0] == cur_pos[0]-1:
        ans.append("East")
        return backtracking(visited, (visited[cur_pos][0], visited[cur_pos][1]), ans)
    else:
        i = len(ans)-1
        reversed_ans= []
        while i>-1:
            reversed_ans.append(ans[i])
            i-=1
        return reversed_ans
def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    nodes = util.Queue()
    nodes.push([problem.getStartState(), 0])
    visited[problem.getStartState()]=(-100,-100)
    while True:
        cur = nodes.pop()
        if problem.isGoalState(cur[0]):
            return backtracking(visited, cur[0], [])
        for i in problem.getSuccessors(cur[0]):
            if (not i[0] in visited):
                visited[i[0]] = cur[0]
                nodes.push([i[0], i[2]+cur[1], cur[0]])
            if problem.isGoalState(cur[0]):
                break


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors((problem.getStartState())))

def manhattanHeuristic(position, problem):
    "The Manhattan distance heuristic for a PositionSearchProblem"
    xy1 = position
    xy2 = problem.goal
    return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """

    return 0
#from searchAgents import manhattanHeuristic
def aStarSearch(problem: SearchProblem, heuristic=manhattanHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    nodes = util.PriorityQueue()
    h = manhattanHeuristic(problem.getStartState(), problem)
    visited={problem.getStartState():[-100,-100, h]}
    nodes.push(problem.getStartState(),0)
    while (not nodes.isEmpty()):
        cur = nodes.pop()
        for i in problem.getSuccessors(cur[1]):
            h = manhattanHeuristic(i[0], problem)
            if (not i[0] in visited or cur[0]+i[2]+h<visited[i[0]][2]):
                visited[i[0]] = [cur[1][0], cur[1][1], cur[0]+i[2]+h]
                nodes.push(i[0], cur[0]+i[2])
            if problem.isGoalState(i[0]):
                return backtracking(visited, i[0], [])
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
