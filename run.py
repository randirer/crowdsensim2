import numpy as np
import random
import networkx as nx


# Still some tech debt to pay for here.
# 1. User locations don't update until pseudo-node removal in crowdsemsim2.py is removed, otherwise
#    a user's location no longer exists.
#    Will factor this in once we begin profiling, don't want to remove this until I see how it affects
#    the other models.
# 2. userused/routeused order still backwards, so users have static locations for now.
# 3. Djikstra takes awhile to run. Appears to cache after the first few iterations though.
# 4. Djikstra pathing targets removed until 1 and 2 figured out - they have nowhere to go until that
#    is figured out.

def normalizeList(lst):
    return [float(x)/sum(lst) for x in lst]

# Gerenate n random float normally distributed about mean with stddev
def nNormalRands(n, mean, stddev):
    return np.random.normal(mean, stddev, n)

def shuffleList(lst):
    length = len(lst)
    for index in range(length):
        current = lst[index]
        choose = random.randint(0,length-1)
        to = lst[choose]

        lst[index] = to
        lst[choose] = current


class SocialMobility():
    def __init__(self, users, minCoords, maxCoords, distance):
        self.initGrid(minCoords, maxCoords, distance)

        self.initUsers(users, connCoeff=0.5)

        

    # Distance: how large we want our grid to be
    def initGrid(self, minCoords, maxCoords, distance):
        # Some throat-clearing to handle python's inability to increment loops by a float
        # Initialize at start coordinates
        latCount = minCoords[1]
        longCount = minCoords[0]

        # Increment by fixed amount
        latInc = distance
        longInc = distance 

        # We actually overshoot the last grid sometimes, but that's okay it's just an index.
        latIters = int((maxCoords[1] - minCoords[1])/distance)+1
        longIters = int((maxCoords[0] - minCoords[0])/distance)+1

        
        grid = []
        for i in range(latIters):
            temp = []
            for j in range(longIters):
                temp.append((latCount, longCount))
                latCount += latInc

            latCount = minCoords[1]
            grid.append(temp)
            longCount += longInc



        self.grid = grid
        print(self.grid)


    # Locate the grid index belonging to the x and y (long and lat)
    def locateGrid(self, x, y):
        gridIndex = (-1, -1)
        lastRow = []
        yIndex = 0
        xIndex = 0

        # First loop to find the y coord, return correct row
        for row in self.grid:
            (longCoord, latCoord) = row[0]
            if (y > latCoord):
                yIndex = yIndex + 1
                continue

            else:
                lastRow = row
                break
            
        # Then loop through that row to get our x coord
        for (longCoord, latCoord) in lastRow:
            if (x > longCoord):
                xIndex = xIndex + 1
                continue
            else:
                break

        return (xIndex, yIndex)
        
    # Find the node id of the user
    def locateUser(self, userId):
        print("Locationg ", userId, "from ", self.userLocs)
        userOrigin = self.userLocs[userId][1]
        return userOrigin


    def userAttraction(self, uid):
        return self.userWeights[uid]

    def generateConnectionList(self, userCount, conns):
        # Generate uids to connect to, get their global weights
        indexes = [self.userAttraction(random.randint(0,userCount-1)) for _ in range(conns)]

        # Pad the matrix out with zeroes
        zeroes = [0 for _ in range(userCount - conns)]

        ret = indexes + zeroes
        shuffleList(ret)
        return ret
                
    def initUsers(self, count, mean=0.5, stddev=0.25, connCoeff=0.05):
        # Weights give a global "attraction factor" to each user indexed by uid
        self.userWeights = nNormalRands(count, mean, stddev)
        self.userWeights = list(map(abs, self.userWeights))

        
        # How many other users any one user is connected to.
        # Begin the search here if memory becomes an issue, lot a useless zeroes here.
        connections = 1+int(count*connCoeff)
        self.userConns = []
        for uid in range(count):
            self.userConns.append(self.generateConnectionList(count, connections))

    def usersInGrid(self, gridIndex):
        users = []
        for uid in range(len(self.userLocs)):
            ((x,y), _) = self.userLocs[uid]
            if (self.locateGrid(x,y) == gridIndex):
                users.append(uid)
                
        return users

    # Get some random node at that grid location.
    # There has to be users their for this to work.
    def randomUserPointInGrid(self, gridIndex):
        uid = random.choice(self.usersInGrid(gridIndex))
        ((_,_), nodeIndex) = self.userLocs[uid]
        return nodeIndex
        
    # Evaluate the weights that userId is connected to.
    # Normalize their weights to a probability, and pick one
    def pickDestination(self, userId):
        grids = dict()

        weights = self.userConns[userId]

        for uid in range(len(weights)):
            weight = weights[uid]
            ((x,y), nodeId) = self.userLocs[uid]

            # Figure out what grid index our current user is in
            grid = self.locateGrid(x,y)

            # Save a lot of zeroes this way
            if (weight != 0):
                if (grids.get(grid)):
                    grids[grid] = grids[grid] + weight
                else:
                    grids[grid] = weight

        # Interesting that there's a builtin for exactly for this purpose, but I'll take it...
        print(grids)
        gridIndex = random.choices(list(grids.keys()), weights=grids.values(), k=1)
        return gridIndex[0]

    # User has moved along their destination path, now update their location in the lookup array
    def updateUser(userId, lastNodeID ):
        self.userLocs[userId] = lastNodeID

    ### NOTE
    ### The following functions all have some dependency on the graphs of cities.
    ### I expect the worst of problems can be found from here on out
    def pathTo(self, originNode, node, cut, cityGraph):
        
        # Returns tuple of two dictionaries keyed by target nodes.
        # First stores distance to each target. Second stores path
        (length, path) = nx.single_source_dijkstra(cityGraph, originNode, target=None, weight='length')

        idr = 0 
        for l in length:
            if length[l] > cut:
                idr = l
                
        if idr==0:
            idr = max(length, key=length.get)

        # From here we can feed this into the rest of the user generation algorithm and continue
        return path[idr]
    
    def chooseRoute(self, userId, cut, cityGraph):
        dest = self.randomUserPointInGrid(self.pickDestination(userId))
        return self.pathTo(self.userLocs[userId][1], dest, cut, cityGraph)

    def initUserOrigins(self, osmidMax, osmidNew, minCoords, maxCoords, G_imp):
    #def initUserOrigins(self, osmidMax, osmidNew, minCoords, maxCoords):
        # Users loc entries have the following structure: ((x, y), nodeId)
        # Locs tell us where to find each particular user
        
        self.userLocs = []
        for _ in range(len(self.userWeights)):
            nodeId = random.randint(osmidMax+1, osmidNew-1)
            tempNode = G_imp.node[nodeId]
            self.userLocs.append(((tempNode['x'], tempNode['y']), nodeId))

        print(self.userLocs)

            #x = random.randint(minCoords[1], maxCoords[1])
            #y = random.randint(minCoords[0], maxCoords[0])
            # self.userLocs.append(((x,y), nodeId))


                      
    



# sm = SocialMobility(10, (0, 0), (4, 6), 1)
# sm.initUserOrigins(5, 10, (0,0), (4,6))
# print(sm.locateGrid(1,2))

# print("Picking destination for user 0")
# dest = sm.pickDestination(0)
# print(dest)

# print("Users currently there:")
# print(sm.usersInGrid(dest))


# print("Destination function:")
# print(sm.randomUserPointInGrid(dest))

# sm.chooseRoute(0, 10)
