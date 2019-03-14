import time as t


print('Hey buddy! Hope you are having a great time! Are you in the mood to play some 8-Puzzle? :)\nYes??!\nWonderful!\nLets go then!\n')
print('Choose one of the following options for the difficulty level of the puzzle:')
print('1) EASY Mode\n2) MEDIUM Mode\n3) HARD Mode')
option = input()
if option == '1':
    print('Really?! Easy?! Don`t you like a little bit of challenge?? Okay then!' )
    s = [1,3,4,8,6,2,7,0,5]

elif option == '2':
    print('Not too hard and not too easy? It seems you had a really long day :D Okay, go ahead!')
    s = [2,8,1,0,4,3,7,6,5]

elif option == '3':
    print('Now we`re talking! Your computer is going to be crushed by this puzzle!')
    s = [5,6,7,4,0,8,3,2,1]




print('\nNow choose one of the following methods to solve the puzzle:')
print('1) Breadth First Search\n2) Depth First Search\n3) Greedy Best First Search\n4) Uniform Cost Search\n5) Iterative Deepening Search\n6) A* Search')
option = input()
if option == '1':
    search_algorithm = 'bfs'
    iterative_deep = False
    h_function = 'default'
elif option == '2':
    search_algorithm = 'dfs'
    iterative_deep = False
    h_function = 'default'
elif option == '3':
    search_algorithm = 'gbfs'
    iterative_deep = False
    h_function = 'default'
elif option == '4':
    search_algorithm = 'ucs'
    iterative_deep = False
    h_function = 'default'
elif option == '5':
    search_algorithm = 'ids'
    iterative_deep = True
    h_function = 'default'
elif option == '6':
    search_algorithm = 'astar'
    iterative_deep = False
    print('Cool! Which heuristics do you want to use for A*?')
    print('\n1) Number of Misplaced Tiles\n2) Manhattan Distance\n3) Euclidean Distance')
    answer = input()
    if answer == '1':
        h_function = 'misplacedTiles'
    elif answer == '2':
        h_function = 'Manhattan'
    elif answer == '3':
        h_function = 'Euclidean'
    else:
        print('Wrong number! Press 1, 2 or 3...')
else:
    print('Wroooooong! Wake up! The available options are 1, 2, 3, 4, 5, and 6...')

goal = [1,2,3,8,0,4,7,6,5]

class Thing:


    def HeuristicCost(state, goal, function):

        heuristic_cost = 0

        # the number of puzzle pieces out of place
        if function == 'misplacedTiles':
            for i in zip(state, goal):
                if i[0] != i[1]:
                    heuristic_cost += 1
                else:
                    continue

        # manhattan distance of the puzzle pieces to their goal state
        elif function == 'Manhattan':
            for i in goal:
                heuristic_cost += abs(goal.index(i) - state.index(i))

        # euclidean distance of the puzzle pieces to their goal state
        elif function == 'Euclidean':
            for i in goal:
                heuristic_cost += (goal.index(i) - state.index(i))**2

        elif function == 'default':
            heuristic_cost = 0


        return heuristic_cost

    def __init__(self, key, state, parent, g_n, depth, h_function, goal, move):

        self.key = key
        self.state = state
        self.parent = parent
        self.g_n = g_n
        self.depth = depth
        self.h_function = h_function
        self.goal = goal
        self.move = move
        self.h_n = Thing.HeuristicCost(self.state , self.goal , self.h_function)
        self.total_cost = self.g_n + self.h_n
        self.get_moves()

    def get_moves(self):

        self.moves = []

        # possible moves
        if self.state.index(0) == 0  :  self.moves.extend(('left','up'))
        elif self.state.index(0) == 1:  self.moves.extend(('left','right','up'))
        elif self.state.index(0) == 2:  self.moves.extend(('right','up'))
        elif self.state.index(0) == 3:  self.moves.extend(('up','down', 'left'))
        elif self.state.index(0) == 4:  self.moves.extend(('up','down','left','right',))
        elif self.state.index(0) == 5:  self.moves.extend(('right','up', 'down'))
        elif self.state.index(0) == 6:  self.moves.extend(('down','left'))
        elif self.state.index(0) == 7:  self.moves.extend(('left','right', 'down'))
        else:  self.moves.extend(('right','down'))

    def move_piece(self, move):

        new_node = self.state[:]
        zero_idx = new_node.index(0)

        if move == 'left':  rep_idx = zero_idx + 1
        elif move == 'right':  rep_idx = zero_idx - 1
        elif move == 'up':  rep_idx = zero_idx + 3
        else:  rep_idx = zero_idx - 3

        # rep_val also represents the cost since it's the value of the piece being moved
        rep_val = self.state[rep_idx]
        new_node[zero_idx] = rep_val
        new_node[rep_idx] = 0
        return new_node , rep_val

class queue:

    def __init__(self, search_algorithm, goal_state):

        self.search_algorithm = search_algorithm
        self.queue = []

    def return_node(self):

        if self.search_algorithm == 'bfs':  return self.queue[0] # FIFO
        elif self.search_algorithm == 'dfs':  return self.queue[-1] # LIFO
        elif self.search_algorithm == 'ucs':  return sorted(self.queue, key=lambda x: x.g_n)[0]  # return node with the lowest cost, g(n)
        elif self.search_algorithm == 'astar':  return sorted(self.queue, key=lambda x: x.total_cost)[0] # return node with the lowest total cost
        elif self.search_algorithm == 'gbfs':  return sorted(self.queue, key=lambda x: x.h_n)[0] # return node with the lowest heuristic cost, h(n)
        elif self.search_algorithm == 'ids': return sorted(self.queue, key=lambda x: x.depth)[0]

class PuzzleSolver:

    def __init__(self, node_init, goal_state, search_algorithm, iterative_deep):

        self.goal_state = node_init.goal
        self.current_node = node_init
        self.root = node_init
        self.search_algorithm = search_algorithm
        self.heuristic_function = node_init.h_function
        self.iterative_deep = iterative_deep
        self.key = 0
        self.move_counter = 0
        self.tree = {}
        self.queue = queue(self.search_algorithm, self.goal_state)
        self.queue.queue.append(self.root)
        self.visited_states = []
        self.depth_counter = 0
        self.limit = 0
        self.tree[0] = self.root
        self.solver()

    def solver(self):

        start_time = t.time()
        self.current_node = self.queue.return_node()

        while self.queue:

            # used to find the maximum length of the queue at the end
            que_len = []
            que_len.append(len(self.queue.queue))

            # check for goal convergence
            if self.current_node.state != self.goal_state:

                if self.iterative_deep:
                    if self.depth_counter > self.limit:
                        self.limit += 1
                        self.key = 0
                        self.move_counter = 0
                        self.tree = {}
                        self.queue = queue(self.search_algorithm, self.goal_state)
                        self.queue.queue.append(self.root)
                        self.visited_states = []
                        self.depth_counter = 0
                        self.current_node = self.root
                    else:  pass
                else:  pass

                # repeated state checking
                if self.current_node.state not in self.visited_states:
                    self.visited_states.append(self.current_node.state[:])
                    self.move_counter+=1

                    # for the current node, begin looping through possible nodes so we can make new nodes in the tree, we use the
                    # move piece method to iteratively return new states and the cost of the move
                    for move in self.current_node.moves:
                        self.key += 1
                        new_state , g_n = self.current_node.move_piece(move)
                        g_n += self.current_node.g_n
                        new_node = Thing(key=self.key,state=new_state,parent=self.current_node.key,g_n = g_n,depth=self.depth_counter+1,\
                                        h_function=self.heuristic_function,goal=self.goal_state,move=move)
                        self.tree[self.key] = new_node

                        # for searches that sort on cost, we have to search the queue to see if the states exists in the queue, if so
                        # we have to check and see if the cost of these existing nodes is more, if it isn't we leave the node in the queue
                        if self.search_algorithm in ['ucs', 'astar', 'gbfs']:
                            c = 0
                            if self.search_algorithm == 'ucs':  sort = 'g_n'
                            elif self.search_algorithm == 'astar':  sort = 'total_cost'
                            else:  sort = 'h_n'

                            for i in self.queue.queue:
                                    if i.state == new_node.state:
                                        if getattr(i,sort) > getattr(new_node,sort):
                                            del self.queue.queue[c]
                                        else:  c+=1
                                    else:  c += 1

                        else:  pass

                        self.queue.queue.append(new_node)

                    self.depth_counter+=1
                    self.current_node = self.queue.return_node()

                else:
                    if self.search_algorithm == 'dfs':  idx = -1
                    else:  idx = 0

                    # if we need to sort the queue we do, so we delete the proper node from the queue///// in the case of uniform cost,
                    # we sort by cost, best first we sort by heuristic cost and A* we sort by total cost
                    if self.search_algorithm == 'ucs':  self.queue.queue = sorted(self.queue.queue, key=lambda x: x.g_n)
                    elif self.search_algorithm == 'gbfs':  self.queue.queue = sorted(self.queue.queue, key=lambda x: x.h_n)
                    elif self.search_algorithm == 'astar':  self.queue.queue = sorted(self.queue.queue, key=lambda x: x.total_cost)
                    else:  pass

                    # we now delete an item from the queue based on the index defined above
                    del self.queue.queue[idx]
                    self.current_node = self.queue.return_node()

            else:
                # the puzzle has been solved so we can break the while loop
                break

        end_time = t.time()
        for i,j in self.tree.items():
            if j.state == self.goal_state:
                out = i
                break
            else:  continue

        # this will loop through the tree and find the path until we reach the root node
        # items are added in reverse order so we can print the path
        path = [out]
        while out != 0:
            path.insert(0, self.tree[out].parent)
            out = path[0]

        for i in path:
            print ('Move:', self.tree[i].move, '\n', 'Heuristic Cost:', self.tree[i].h_n, '\n', 'Total Cost:',self.tree[i].g_n,\
                '\n', self.tree[i].state[0:3], '\n', self.tree[i].state[3:6], '\n', self.tree[ i].state[6:], '\n')
        print ('Total Moves: ', len(path) - 1) # don't include the initial state as a move
        print ('Output:', '\n', 'Maximum Queue Length:', max(que_len), '\n', 'Nodes Popped:', self.move_counter, '\n',\
            'Time:', end_time - start_time)


Output = Thing(key=0,state=s,parent=0,g_n=0,depth=0,h_function=h_function,goal=goal,move='Initial State')
Run = PuzzleSolver(Output,goal_state=goal,search_algorithm=search_algorithm,iterative_deep=False)
