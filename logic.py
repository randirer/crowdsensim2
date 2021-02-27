import random
import math

def getEventFiles(days):
    eventfile=[]
    for d in range(0,days):
        f = open('./Inputs/Mobility/UserMovementsListEvents_'+str(d)+'.txt', 'w')
        f.write("/ID-User/-/Lat/-/Long/-/Alt/-/Day/-/Hour/-/Minute/- \n")
        eventfile.append(f)
    return eventfile
        
def generateUsers(days, users, minSpeed=1.0, maxSpeed=1.5):
    eventfiles = getEventFiles()


    for day in range(0, days):
        

        for userId in range(0, users):
        
        # If we're dealing with the first instance of user:
        print (userId, point[0], point[1], 0, day, 0, day, hours, minutes, file=eventfiles[day])

        # If we're dealing with the laters instances of a user
        print (userId, point[0], point[1], 0, day, hours, minutes, count_minute, bearing1, distance, file=eventfiles[day])




# userused: Current user, now provided from loop
# point[0] and point[1]: Latitude and longitude
#    Found from calculateYX(speed, route_node, bearing, distance_last_node)
#    route_node:
#    bearing: Caluclated given route, index, G_old

(length, path) = nx.single_source_dijkstra(G_old, origin_node, target=None, cutoff=cutadded, weight='length')

print (userused, point[0], point[1], 0, fay,hours,minutes,count_minute,bearing1,n, file=eventfiles[day])
