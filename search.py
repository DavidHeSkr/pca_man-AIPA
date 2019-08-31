# search.py
# ---------
# Licensing Information:	You are free to use or extend these projects for
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

import util
import searchAgents

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


def tinyMazeSearch(problem):
		"""
		Returns a sequence of moves that solves tinyMaze.  For any other maze, the
		sequence of moves will be incorrect, so only use this for tinyMaze.
		"""
		from game import Directions
		s = Directions.SOUTH
		w = Directions.WEST
		return	[s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
		"""
		Search the deepest nodes in the search tree first.

		Your search algorithm needs to return a list of actions that reaches the
		goal. Make sure to implement a graph search algorithm.

		To get started, you might want to try some of these simple commands to
		understand the search problem that is being passed in:

		print("Start:", problem.getStartState())
		print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
		print("Start's successors:", problem.getSuccessors(problem.getStartState()))
		"""
		"*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"

		util.raiseNotDefined()

def breadthFirstSearch(problem):
		"""Search the shallowest nodes in the search tree first."""
		"*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"

		util.raiseNotDefined()

def uniformCostSearch(problem):
		"""Search the node of least total cost first."""
		"*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"


		util.raiseNotDefined()

def nullHeuristic(state, problem=None):
		"""
		A heuristic function estimates the cost from the current state to the nearest
		goal in the provided SearchProblem.  This heuristic is trivial.
		"""
		return 0

def aStarSearch(problem, heuristic=nullHeuristic):
		"""Search the node that has the lowest combined cost and heuristic first."""
		"*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
		util.raiseNotDefined()

def iterativeDeepeningSearch(problem):

		"""Search the deepest node in an iterative manner."""
		"*** YOUR CODE HERE FOR TASK 1 ***"
		steps = 0
		for i in range(0, 10000):
				j = i
				#print("I is : ", i)
				result = []
				visited = []

				stack = util.Stack()
				start = (problem.getStartState(), [])
				stack.push(start)

				while not stack.isEmpty():
						#print("33333")
						(state, path) = stack.pop()
						if problem.isGoalState(state):
								result = path
								return result

						for w in problem.getSuccessors(state):
								if (w[0] not in visited) and (len(path)+1 <= j):
										#steps = max(steps, len(path))
										#print("the path len is " + str(len(path)))
										visited.append(w[0])
										newPath = path + [w[1]]
											#print("the new path len is ", len(newPath))
										newState = (w[0], newPath)
										stack.push(newState)


		return result

		print(problem)
		util.raiseNotDefined()

def waStarSearch(problem, heuristic=nullHeuristic):
    #Search the node that has has the weighted (x 2) lowest combined cost and heuristic first.
    "*** YOUR CODE HERE FOR TASK 2 ***"

    #hardcoded weight as specified
    WEIGHT = 2
    current_state = problem.getStartState()
    openList = [([current_state], [], (heuristic(current_state, problem) + 0) * WEIGHT)]
    closedList = []
    best_g = {}

    if len(current_state) > 2:
        best_g[current_state[0]] = 0
    else:
        best_g[current_state] = 0

    while openList:
        openList = sorted(openList, key=lambda x:x[2])
        path, actions, f_current = openList.pop(0)

        last_state = path[-1]
        if len(last_state) <= 2:
            last_pos = last_state
        else:
            last_pos = last_state[0]
        g_current = f_current - heuristic(last_state, problem) * 2
        if last_state not in closedList or best_g[last_pos] > g_current:
            #closedList.append(path[-1])
            #best_g[path[-1]] = g_node
            if last_state not in closedList:
                closedList.append(last_state)
            best_g[last_pos] = g_current

            if problem.isGoalState(last_state):
                return actions
            for successorState, action, cost in problem.getSuccessors(path[-1]):
                if successorState not in path:
                    #if successor is not the goal and not visited, add to the expansion stack
                    newf = g_current + (cost + heuristic(successorState, problem)) * WEIGHT
                    openList.append((path + [successorState], actions + [action], newf))
    return actions



# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
wastar = waStarSearch


"""
def waStarSearch(problem, heuristic=nullHeuristic):
    #Search the node that has has the weighted (x 2) lowest combined cost and heuristic first.
    "*** YOUR CODE HERE FOR TASK 2 ***"

    #hardcoded weight as specified
    WEIGHT = 2
    currentState = problem.getStartState()
    openList = [([currentState], [], (heuristic(currentState, problem) + 0) * WEIGHT)]
    closedList = []
    best_g = {currentState: 0}
    while openList:
        openList = sorted(openList, key=lambda x:x[2])
        path, actions, oldCost = openList.pop(0)
        g_node = (oldCost - heuristic(path[-1], problem)) /2
        if path[-1] not in closedList or best_g[path[-1]] > g_node:
            closedList.append(path[-1])
            best_g[path[-1]] = g_node
            if problem.isGoalState(path[-1]):
                return actions
            for successorState, action, cost in problem.getSuccessors(path[-1]):
                if successorState not in path:
                    #if successor is not the goal and not visited, add to the expansion stack
                    newf = g_node + (cost + heuristic(successorState, problem)) * WEIGHT
                    openList.append((path + [successorState], actions + [action], newf))

					def waStarSearch(problem, heuristic=nullHeuristic):
						W = 0
						#Search the node that has has the weighted (x 2) lowest combined cost and heuristic first.
						"*** YOUR CODE HERE FOR TASK 2 ***"
						start_state = problem.getStartState()
						if isinstance(problem, searchAgents.CapsuleSearchProblem):
							print("yes!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
							# to update the visited points so that we can search next food
							visited = start_state[3]
						else:
							visited = []
						heuris_distance = util.PriorityQueue()
						#currentState/path/cost
						heuris_distance.push((start_state, []), 0)
						reopen = util.Counter()

						while (heuris_distance.isEmpty() == False):
							#curr_state : [curr_point(a,b), capsule_position([(a,b)]), foods(2_d list), visited(list)
							curr_state, actions = heuris_distance.pop()
							print("The actions are:------------", actions)
							if isinstance(problem, searchAgents.CapsuleSearchProblem):
								curr_point = curr_state[0]
							else:
								curr_point = curr_state
							#if curr_point in visited and problem.getCostOfActions(actions) >= reopen[curr_point]:
							if curr_state not in visited or problem.getCostOfActions(actions) < reopen[curr_point]:
								visited.append(curr_state)
								#print("The visited are:__________", visited)
								#print("the returned curr_state is :&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&", curr_state[2])
								reopen[curr_state[0]] = problem.getCostOfActions(actions)
								if problem.isGoalState(curr_state):
									return actions

								#curr_state[3] = visited
								# successors.append((nextState, action, cost))
								for next_state, direction, steps in problem.getSuccessors(curr_state):
									if next_state[0] not in visited:
										position = next_state[0]
										if isinstance(problem, searchAgents.CapsuleSearchProblem):
											#visited = next_state[3]
											print("the visited is::::::::::::::::::::::::::::::::::::::::::,", visited)
										heuris_distance.push((next_state, actions + [direction]) , steps + problem.getCostOfActions(actions) + 2 * heuristic(next_state, problem))
						#util.raiseNotDefined()
						print("I am about to return Noneeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee!!!!!!!")
"""







#python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=wastar,heuristic=manhattanHeuristic
