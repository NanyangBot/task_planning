import sys

###############################################################################

'''
Shortest Path Problem
Dijkstra algorithm
'''
class PathSolver():

    def __init__(self):
        self.n = None
        self.graph = None

    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):

        # Initialize minimum distance for next node
        min_dist = sys.maxsize

        # Search not nearest vertex not in the
        # shortest path tree
        for u in range(self.n):
            if dist[u] < min_dist and sptSet[u] == False:
                min_dist = dist[u]
                min_index = u

        return min_index, min_dist

    # Function that implements Dijkstra's single source
    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def solve(self, adj, idx, verbose=False):
        
        self.n = len(adj)
        self.graph = adj
        
        path = [None] * self.n
        cost = 0

        dist = [sys.maxsize] * self.n
        dist[idx] = 0
        sptSet = [False] * self.n

        for cout in range(self.n):

            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # x is always equal to src in first iteration
            x, cc = self.minDistance(dist, sptSet)

            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[x] = True
            path[cout] = x
            cost += cc

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shortest path tree
            for y in range(self.n):
                if self.graph[x][y] > 0 and sptSet[y] == False and \
                        dist[y] > dist[x] + self.graph[x][y]:
                    dist[y] = dist[x] + self.graph[x][y]

        if verbose:
            print('- Dijkstra algorithm -')
            print('Score (cost) :', cost)
            print('Solution (path) : ', end=' ')
            for i in range(self.n):
                print(path[i], end=' ')
            print('\n---')
        return path
