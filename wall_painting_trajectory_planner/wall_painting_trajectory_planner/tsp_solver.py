import math

###############################################################################

'''
Travelling Salesman Problem
Bround and Bound algorithm
'''
class TSPsolver:
    def __init__(self):
        self.maxsize = float('inf')
        self.score = None
        self.path = None
        self.n = None

    def copyToFinal(self, curr_path):
        self.path[:self.n] = curr_path[:]

    # Function to find the minimum edge cost
    # having an end at the vertex i
    def firstMin(self, adj, i):
        m = self.maxsize
        for k in range(self.n):
            if adj[i][k] < m and i != k:
                m = adj[i][k]
        return m

    # function to find the second minimum edge
    # cost having an end at the vertex i
    def secondMin(self, adj, i):
        first, second = self.maxsize, self.maxsize
        for j in range(self.n):
            if i == j:
                continue
            if adj[i][j] <= first:
                second = first
                first = adj[i][j]
            elif (adj[i][j] <= second and adj[i][j] != first):
                second = adj[i][j]
        return second

    def tsp(self, adj, curr_bound, curr_weight, level, curr_path, visited):
        # base case is when we have reached level N
        # which means we have covered all the nodes once
        if level == self.n:
            if curr_weight < self.score:
                self.copyToFinal(curr_path)
                self.score = curr_weight    
            return

        # for any other level iterate for all vertices
        # to build the search space tree recursively
        for i in range(self.n):
            # Consider next vertex if it is not same
            # (diagonal entry in adjacency matrix and
            # not visited already)
            if (adj[curr_path[level-1]][i] != 0 and
                                visited[i] == False):

                temp = curr_bound
                curr_weight += adj[curr_path[level - 1]][i]
            
                # different computation of curr_bound
                # for level 1 from the other levels
                if level == 1:
                    #curr_bound -= firstMin(adj, curr_path[level - 1])**2 + firstMin(adj, i)**2
                    curr_bound -= (self.firstMin(adj, curr_path[level - 1]) + self.firstMin(adj, i))/2
                else:
                    #curr_bound -= secondMin(adj, curr_path[level - 1])**2 + firstMin(adj, i)**2
                    curr_bound -= (self.secondMin(adj, curr_path[level - 1]) + self.firstMin(adj, i))/2
                
                # curr_bound + curr_weight is the actual lower bound
                # for the node that we have arrived on.
                # If current lower bound < final_res,
                # we need to explore the node further
                if curr_bound + curr_weight < self.score:
                    curr_path[level] = i
                    visited[i] = True

                    # call TSPRec for the next level
                    self.tsp(adj, curr_bound, curr_weight, level + 1, curr_path, visited)

                # Else we have to prune the node by resetting
                # all changes to curr_weight and curr_bound
                curr_weight -= adj[curr_path[level - 1]][i]
                curr_bound = temp

                # Also reset the visited array
                visited = [False] * len(visited)
                for j in range(level):
                    if curr_path[j] != -1:
                        visited[curr_path[j]] = True

    def solve(self, adj, verbose=False):
        self.n = len(adj)
        self.path = [None] * self.n
        self.score = self.maxsize

        score = float('inf')
        path = None

        for idx in range(self.n):
            # Calculate initial lower bound for the root node
            # using the formula 1/2 * (sum of first min +
            # second min) for all edges. Also initialize the
            # curr_path and visited array
            curr_bound = 0
            curr_path = [-1] * self.n
            visited = [False] * self.n

            # Compute initial bound
            for i in range(self.n):
                curr_bound += (self.firstMin(adj, i) + self.secondMin(adj, i))

            # Rounding off the lower bound to an integer
            curr_bound = math.ceil(curr_bound/2)

            # We start at vertex 1 so the first vertex
            # in curr_path[] is 0
            visited[idx] = True
            curr_path[0] = idx

            # Call to TSPRec for curr_weight
            # equal to 0 and level 1
            self.tsp(adj, curr_bound, 0, 1, curr_path, visited)

            if (self.score < score):
                score = self.score
                path = self.path

        if verbose:
            print('---')
            print('TSP score (cost) :', score)
            print('TSP solution (path) : ', end = ' ')
            for i in range(self.n):
                print(path[i], end = ' ')
        
        return path
