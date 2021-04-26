import random
import math

import osmnx as ox
import networkx as nx


# Task evaluation framework for mobility models produced by crowdsensim

def getCityGraph(name_city, type_net="walk"):
    G = ox.graph_from_place(name_city, network_type=type_net,simplify=False)
    return G


def grabRandom(array):
    i = random.randint(0, len(array)-1)
    return array[i]

class TaskObj():
    def __init__(self, pt, start, duration, taskRadius, subTasks, compThresh):
        self.pt = pt
        self.start = start
        self.duration = duration
        self.radius = taskRadius
        self.subtasks = subTasks

        self.users = 0
        self.compThresh = compThresh
        self.visit = dict()

    ## ALGORITHM FUCNTIONS
    # We care about these for actually running the tests
        
    # Signifies that a user has visited this task
    # Time interval: how many minutes that user spent at this task. Default to 1 minute increments
    def makeVisit(self, uid, distance, timeInterval=1):
        timed = False
        seen = self.visit.get(uid, False)
        if not seen:
            self.users += 1
            self.visit[uid] = (distance, timeInterval)
            timed = True
            
            seen = self.visit[uid]

            

            
        if (seen[0] < distance):
            self.visit[uid] = (distance, self.visit[uid][1])

        if not timed:
            self.visit[uid] = (self.visit[uid][0], timeInterval+self.visit[uid][1])
            

    def hasSeen(self, uid):
        seen = self.visit.get(uid, False)
        return seen

    ## DIAGNOTSIC FUNCTIONS
    ## We care about these for results
    def completionRatio(self):
        rate = self.users / self.subtasks
        return 1.0 if (rate >= 1.0) else rate

    def isComplete(self):
        return self.completionRatio() >= self.compThresh

    # Average distance between every user that saw a task, and the task itself
    def averageDistance(self):
        dists = sum(list(map(lambda user: user[0], self.visit.values())))

        if (len(self.visit.keys()) == 0):
            return 0
        
        return dists/len(self.visit.keys())

    # Averages of how much time a user spent at a particular point
    def averageTime(self):
        times = sum(list(map(lambda x: x[1], self.visit.values())))

        if (len(self.visit.keys()) == 0):
            return 0

        return times/len(self.visit.keys())


class Tasks():
    # maxDuration in minutes, start/stop hours in 24h
    def __init__(self, startHour, stopHour, maxDuration, taskRadius, compRatio):
        self.points = []
        self.startHour = startHour

        self.stopHour = stopHour
        self.maxDuration = maxDuration
        self.compRatio = compRatio
        self.taskRadius = taskRadius
         

    def loadPoints(self, pointArray):
        self.points += pointArray

    def choosePoint(self):
        return grabRandom(self.points)


    def isEnoughTime(self, taskHour, taskMinute):
        # need at least self.maxDuration hours left in day for a task to be accepted
        hoursLeft = self.stopHour - taskHour
        minsLeft = (hoursLeft * 60) - taskMinute
        
        minsReq = self.maxDuration

        if (minsReq > minsLeft):
            return False

        return True
                  
    def chooseTime(self):
        selected = False
        hour = -1  
        minute = -1
        
        while (not(selected)):
            hour = random.randint(self.startHour, self.stopHour)
            minute = random.randint(0, 59)

            if (self.isEnoughTime(hour, minute)):
                selected = True
                
        return (hour, minute)
    
    def genTask(self, minSubs=3, maxSubs=5):
        pt = self.choosePoint()
        start = self.chooseTime()
        rad = self.taskRadius

        # Count number of subtasks this task is comprised of
        subTasks = random.randint(minSubs, maxSubs)
        
        return TaskObj(pt, start, self.maxDuration, rad, subTasks, self.compRatio)


# Take a graph, and generate possible (lat, long) pairs to place tasks at
def nodeLatLongs(G):
    pairs = list()
    g_und = G.to_undirected()
    
    for u,v,data in list(g_und.edges(data=True)):
        x1=g_und.node[u]['x']
        y1=g_und.node[u]['y']
        x2=g_und.node[v]['x']
        y2=g_und.node[v]['y']

        pairs.append((y1, x1))
        pairs.append((y2, x2))

    return pairs




def cartesianDistance(origin, dest):
    return math.sqrt(((origin[0]-dest[0])**2) +
                     ((origin[1]-dest[1])**2))


def haversine(originLatLong, destLatLong):
    lat1, lon1 = originLatLong
    lat2, lon2 = destLatLong
    radius = 6371 # km

    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius * c

    return d



# Testing harness for running experiments over tasks.
# API includes generation, indexing and evaluation
class TaskTest():

    # Here's some approximations according to the US Naval Academy. These change
    # as you travel away from the equator:
    # 0.001 = 111 m
    # 0.0001 = 11.1 m
    # 0.0005  = 55.5m
    def __init__(self, nTasks, compThreshold, minSubTasks, maxSubTasks, startHour, stopHour, cityGraph, maxDuration=90, radius=0.01):
        self.t = Tasks(startHour, stopHour, maxDuration, radius, compThreshold);
        
        self.t.loadPoints(nodeLatLongs(cityGraph))
        print("Successfully loaded task locations")

        self.radius = radius
        self.tasks = [self.t.genTask(minSubTasks, maxSubTasks) for _ in range(0, nTasks)]
        
        #for task in self.tasks:
            #print("Task located at", task.pt, "comprised of", task.subtasks, "subtasks")


    # minDistance: the minimum distance we require to consider a task "seen" by user
    def inRange(self, pt0, pt1, minDistance, rangeFn):
        if (rangeFn(pt0, pt1) <= minDistance):
            return True

        return False
         
    # This is the inner loop of runTest. Once we have a user in mind, figure out which
    # tasks they have seen, and increment the task's respective counters
    def evalUserTask(self, uid, userLoc, timeInterval=1, rangeFn=cartesianDistance):
        tasksSeen = 0
        
        for task in self.tasks:

            # Only increment the task info if the task and user are within the defined range of each other
            #print("RANGE FROM", userLoc, "TO", task.pt, "GIVEN RADIUS OF", self.radius)
            #print(rangeFn(userLoc, task.pt))
            if (self.inRange(userLoc, task.pt, self.radius, rangeFn)):
                distance = rangeFn(userLoc, task.pt)
                
                task.makeVisit(uid, distance, timeInterval)
                tasksSeen += 1
                
        return tasksSeen

    
    def minsElapsed(lastMinute, minute):
        lm = lastMinute
        m = minute
        if (lm > m):
            return (60 - lm) + m
        else:
            return m - lm
            
    def runTest(self, days):
        self.howManySeen = dict()
        
        for day in range(0, days):
            with open('./Inputs/Mobility/UserMovementsListEvents_' + str(day) +'.txt', 'r') as data:
                lastUser = -1
                cnt = 0
    
                for line in data:
                    # Skip the first line, it only contains human readable info
                    if cnt == 0:
                        cnt+=1
                        continue

        
                    p = line.split()
        
                    uid = p[0]
                    locX = p[1]
                    locY = p[2]
                    
                    hour = p[5]
                    minute = p[6]
                    
                    coords = (float(locX), float(locY))
                    

                    lastUser = uid
        
                    # Gotta iterate through each task for that user now
                    if self.howManySeen.get(uid, False):
                        self.howManySeen[uid] += self.evalUserTask(uid, coords)
                    else:
                        self.howManySeen[uid] = self.evalUserTask(uid, coords)

    def aggregateCompletion(self):
        res = sum(map(lambda task: task.completionRatio(), self.tasks))
        n = len(self.tasks)
        return res/n
        
    def aggregateTimes(self):
        res = sum(map(lambda task: task.averageTime(), self.tasks))
        n = len(self.tasks)
        return res/n

    def aggregateDistances(self):
        res = sum(map(lambda task: task.averageDistance(), self.tasks))
        n = len(self.tasks)
        return res/n

    
        
    
    # Dump the actual results of our task info
    def results(self):
        tasksCompleted = len(list(filter(lambda task: task.isComplete(), self.tasks)))

        l = len(self.howManySeen.keys())
        avgTasksSeen = 0
        if l != 0:
            avgTasksSeen = sum(self.howManySeen.values())/len(self.howManySeen.keys())
            
        completed = self.aggregateCompletion()
        time = self.aggregateTimes()
        dists = self.aggregateDistances()

        print("TASKS COMPLETED", tasksCompleted)
        print("TASKS SEEN:", avgTasksSeen)
        print("AVERAGE COMPLETION:", completed)
        print("AVERAGE TIME AT POINT:", time)
        print("AVERAGE DISTANCE TO POINT:", dists)
              
        
        
        

                


    
    
# Configurations made here
city = "Timmins, Ontario, Canada"
cityGraph = getCityGraph(city)
compThreshold = 0.5
random.seed(8586032)
tt = TaskTest(10, 0.5, 1, 5, 8, 20, cityGraph, compThreshold, radius=0.0003)

days = 4
tt.runTest(days)
tt.results()
    

# So here's some aproximations according to the USNA.
# (These vary by longitude though, don't rely on them)
# 0.001 = 111 m
# 0.0001 = 11.1 m
# 0.0005  = 55.5m
# So we'll be looking at increments of 0.0001 to 0.001 to determine whether or not


# at 40deg north: 1.0 = 85km
# at 0deg north: 1.0 = 111km
# 0.0001 = 8.5m
# 0.0010 = 85m
# 0.0100 = 850m
# 0.1000 = 85,00m
# 1.0000 = 85,000m


# 50m = 0.0002941176470588235
