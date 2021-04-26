import math

# Simply treat lat/long as points, not as lats/longs.
def cartesianDistance(origin, dest):
    return math.sqrt(((origin[0]-dest[0])**2) +
                     ((origin[1]-dest[1])**2))


def haversine(originLatLong, destLatLong):
    lat1, lon1 = originLatLong
    lat2, lon2 = destLatLong
    radius = 6371000 # m

    dlat = math.radians(lat2-lat1)
    dlon = math.radians(lon2-lon1)
    a = math.sin(dlat/2) * math.sin(dlat/2) + math.cos(math.radians(lat1)) \
        * math.cos(math.radians(lat2)) * math.sin(dlon/2) * math.sin(dlon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = radius * c

    return d

def inRange(origin, target, distance):
    if (cartesianDistance(origin, target) <= distance):
        return True

    return False

def runTest(days):
    for day in range(0, days):
        with open('./Inputs/Mobility/UserMovementsListEvents_' + str(day) +'.txt', 'r') as data:
            lastUser = -1
            cnt = 0
    
            for line in data:
                # Skip the first line, it only contains huma readable info
                if cnt == 0:
                    cnt+=1
                    continue

        
                p = line.split()
        
                uid = p[0]
                locX = p[1]
                locY = p[2]
                hour = p[5]
                minute = p[6]


                compPoint = (37.4861113, 127.0295553)
                pnt = (float(locX), float(locY))
                isInRange = inRange(pnt, compPoint, 0.01)
                rng = ""
                if isInRange:
                    rng = "TRUE"
                else:
                    rng = "FALSE"
                    
                print("USER:", uid)
                print("LOCATION:", (locX, locY))
                print("TIME:", hour + ':' + minute)
                print("IS IN RANGE:", isInRange)



                




def test(lm, m):
    if (lm > m):
        return (60 - lm) + m
    else:
        return m - lm


print(test(59, 1))
print(test(0, 5))

             

        
    
    
