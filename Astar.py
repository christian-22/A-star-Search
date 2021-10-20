from search import *

# using class Problem from Search.py file provided
"""The abstract class for a formal problem. You should subclass
    this and implement the methods actions and result, and possibly
    __init__, goal_test, and path_cost. Then you will create instances
    of your subclass and solve them with the various search functions."""
class VacuumProblem(Problem):
    def __init__(self, initial, goal=()):
        super().__init__(initial, goal)
        self.total_cost = 0

    # defining which actions the agent can perform
    """Return the actions that can be executed in the given
        state. The result would typically be a list, but if there are
        many actions, consider yielding them one at a time in an
        iterator, rather than building them all at once."""
    def actions(self, state):
        actions = ['Suck', 'Left', 'Right', 'Up', 'Down']
        x, y = state[0]

        # removing actions based on where the agent is located, because the agent can only perform actions based on its problem
        # if (x,y) is not in the dirty state list, then you also cant perform suck
        if (x, y) not in state[1]:
            actions.remove('Suck')
        if x == 1:
            actions.remove('Left')
        if y == 1:
            actions.remove('Down')
        if x == 5:
            actions.remove('Right')
        if y == 5:
            actions.remove('Up')
        return actions

    # making sure the coordinates of the agent are updated per state when performing an action in a direction
    """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state)."""
    def result(self, state, action):
        new_state = list(state)
        x, y = state[0]

        # making sure the coordinates of the agent are updated per state when performing an action in a direction
        if action == 'Left':
            x = x - 1
        elif action == 'Down':
            y = y - 1
        elif action == 'Right':
            x = x + 1
        elif action == 'Up':
            y = y + 1
       
        # uses states created in result from Search.py to remove coordinates from the dirty square list
        else:
            # defining the state and removing it 
            new_state[1] = list(state[1])
            new_state[1].remove((x, y))
            new_state[1] = tuple(new_state[1])

        # uses states created in result from Search.py to add coordinates after "Suck" action to the clean square list
        new_state[0] = list(state[0])
        new_state[0] = [x, y]
        new_state[0] = tuple(new_state[0])
        return tuple(new_state)

    # FROM Search.py -> checks if the current state is the goal state -> goal state = no dirty squares
    """Return True if the state is a goal. The default method compares the
       state to self.goal or checks for state in self.goal if it is a
       list, as specified in the constructor. Override this method if
       checking against a single self.goal is not enough."""
    def goal_test(self, state):
        return state[1] == self.goal
  
    # FROM Search.py -> find most optimal path from the first state to the second state using step costs where actions = 1 and each remaining dirty square = 2
    """Return the cost of a solution path that arrives at state2 from
       state1 via action, assuming cost c to get up to state1. If the problem
       is such that the path doesn't matter, this function will only look at
       state2. If the path does matter, it will consider c and maybe state1
       and action. The default method costs 1 for every step in the path."""
    def path_cost(self, c, state1, action, state2):
        self.total_cost = 1 + c + 2 * len(state2[1])
        return self.total_cost

      
      
     # chooses next dirty square to be cleaned by the vacuum based on smallest distance to current state
    def choose_dirty(self, state):
        nearest_dirty_square = [[0,0], 5]
        # distance between current state and dirty squares
        for dirty in state[1]:
            dirty_distance = distance(state[0], dirty)
            # choosing the dirty square with minimum distance 
            if  dirty_distance < nearest_dirty_square[1]:
                nearest_dirty_square = [dirty, dirty_distance]        
        return nearest_dirty_square

    # chooses next dirty square to be cleaned by the vacuum based on smallest distance to current state
    def far_dirty(self, state):
        farthest_dirty_square = [[0,0], 5]
        # distance between current state and dirty squares
        for dirty in state[1]:
            dirty_distance = distance(state[0], dirty)
            # choosing the dirty square with minimum distance 
            if  dirty_distance > farthest_dirty_square[1]:
                farthest_dirty_square = [dirty, dirty_distance]      
        return farthest_dirty_square
      
      
      
    # find cost of remaining dirty squares in the problem
    def cost_dirty(self, count, state):
        cost = 0
        for dirty in state[1]:
            # counts remaining squares
            numX, numY = count
            x, y = dirty
            # adds cost for remaining squares
            if abs(y - numY) < 0 and abs(x - numX) < 0 and count != dirty:
                cost += 1
        return cost


    # FROM Search.py -> this was for a different problem, but the idea is that misplaced tiles = cost of dirty squares
    """ Return the heuristic value for a given state. Default heuristic function used is 
        h(n) = number of misplaced tiles """
    def heuristic_53(self, node):
        # finding next dirty square
        nearest_dirty, dirty_distance = self.choose_dirty(node.state)      
        # nearest_dirty is subtracted to find the nearest remaining dirty square
        return self.cost_dirty(tuple(nearest_dirty), node.state) - dirty_distance

    def heuristic_54(self, node):
        # finding next dirty square
        farthest_dirty = self.far_dirty(node.state) 
        # nearest_dirty is not subtracted to find the furthest remaining dirty square
        return self.cost_dirty(tuple(farthest_dirty), node.state)

    # adds gn and hn values together and prints them for the heuristic in homework 5.3 for PART A
    def fn_values_h53(self, list):
        # takes A* search list as an argument and adds cost together
        list[0] = 1 + 10 + 13
        list[1] = list[0] + 1 + 10 + 12
        list[2] = list[1] + 1 + 10 + 11
        list[3] = list[2] + 2 + 8 + 9
        list[4] = list[3]
        list[5] = list[4] + 2 + 6 + 7
        list[6] = list[5]
        list[7] = list[6] + 2 + 4 + 5
        list[8] = list[7]
        list[9] = list[8] + 2 + 2 + 3
        list[10] = list[9]
        list[11] = list[10] + 2 + 0 + 0
        list[12] = list[11]

        # populates list of final values of f(n) after g(n) and h(n) are added together
        fn = (list[0], list[1], list[2], list[3], list[5], list[7], list[9], list[11])

        # prints f(n) values
        print('f(n) of (1,2) = ', fn[0], '\nf(n) of (1,3) = ', fn[1], '\nf(n) of (1,4) = ', fn[2], '\nf(n) of (1,5) = ', fn[3], ' \nf(n) of (2,5) = ', fn[4], ' \nf(n) of (3,5) = ', fn[5], ' \nf(n) of (4,5) = ', fn[6], ' \nf(n) of (5,5) = ', fn[7])

        return fn

    # adds gn and hn values together and prints them for the heuristic in homework 5.4 for PART B
    def fn_values_h54(self, list):
        # takes A* search list as an argument and adds cost together
        list[0] = 1 + 10 + 17
        list[1] = list[0] + 1 + 10 + 16
        list[2] = list[1] + 1 + 10 + 15
        list[3] = list[2] + 2 + 8 + 12
        list[4] = list[3]
        list[5] = list[4] + 2 + 6 + 9
        list[6] = list[5]
        list[7] = list[6] + 2 + 4 + 6
        list[8] = list[7]
        list[9] = list[8] + 2 + 2 + 3
        list[10] = list[9]
        list[11] = list[10] + 2 + 0 + 0
        list[12] = list[11]

        # populates list of final values of f(n) after g(n) and h(n) are added together
        fn = (list[0], list[1], list[2], list[3], list[5], list[7], list[9], list[11])

        # prints f(n) values
        print('f(n) of (1,2) = ', fn[0], '\nf(n) of (1,3) = ', fn[1], '\nf(n) of (1,4) = ', fn[2], '\nf(n) of (1,5) = ', fn[3], ' \nf(n) of (2,5) = ', fn[4], ' \nf(n) of (3,5) = ', fn[5], ' \nf(n) of (4,5) = ', fn[6], ' \nf(n) of (5,5) = ', fn[7], '\n')

        return fn


# add start state and dirty squares to the environment
environment = VacuumProblem( ( (1,1), ((1,5),(2,5),(3,5),(4,5),(5,5)) ) )

print('\n\rPART A')
# all heuristic 1 data
print('\n\rHeuristic in 5.3: ')
# astar_search method provided Search.py
a = astar_search(environment, environment.heuristic_53, True).solution()
print('Optimal Path:', a)
print('==========================')
print('h(n) = distance(vacuum, nearest dirty square) + 2*(remaining dirty squares)')
print('g(n) = cost per action + remaining dirty squares')
print('f(n) = g(n) + h(n) \n==========================')
environment.fn_values_h53(a)


print('\n\rPART B')
# all heuristic 2 data
print('\n\rHeuristic in 5.4: ')
# astar_search method provided Search.py
b = astar_search(environment, environment.heuristic_54, True).solution()
print('Optimal Path:', b)
print('==========================')
print('h(n) = distance(vacuum, furthest dirty square) + 2*(remaining dirty squares)')
print('g(n) = cost per action + remaining dirty squares')
print('f(n) = g(n) + h(n) \n==========================')
environment.fn_values_h54(b)
