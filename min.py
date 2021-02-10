import osmnx as ox
import pandas as pd
import networkx as nx
import numpy as np
import random
import math
import time
import shutil
import os,sys
import folium
from folium.features import DivIcon
import folium.plugins as plugins

import geopy
from geopy.distance import VincentyDistance


def getCityGraph(name_city, type_net="walk"):
    G = ox.graph_from_place(name_city, network_type=type_net,simplify=False)
    return G



# Get the maximum and minimum latitude and longitude bounds of a city graph
def get_bounds(G):
    maxlat = 0
    maxlong = 0
    minlat = 999999999
    minlong = 999999999

    for n,d in G.nodes(data=True):
        x=d['x']
        y=d['y']
        
        if(y<minlat):
            minlat=y
        if(y>maxlat):
            maxlat=y
        if(x<minlong):
             minlong=x
        if(x>maxlong):
            maxlong=x

    return [(maxlat, maxlong), (minlat, minlong)]


def calculateBearing(node_from, node_to):
    x1 = node_from['x']
    y1 = node_from['y']
    x2 = node_to['x']
    y2 = node_to['y']
        
    lat1 = y1 * math.pi / 180 
    lat2 = y2 * math.pi / 180 
    long1 = x1 * math.pi / 180 
    long2 = x2 * math.pi / 180 
    bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
    bearing = math.degrees(bearing)
    bearing = (bearing + 360) % 360
    return bearing

# SUMMARY OF THE BELOW:
# - Thank fucking god this now makes *some* sense
# - Neighbourhoods are now trivial to add, it'll go something along the lines of this too
# - I'll clean up the comments a bit in the future and document things a bit better
# - min_dist=3 still throwing me off... why 3? 1.5m/s is our max speed, so why 3m
#   distances?
# Add additional points to the city graph for... something
# Removed the notion of "groups" form this one, may serve as a good way of implementing
# neighbourhoods later on though
def add_points(G, min_dist=3):
    g_und = G.to_undirected()
    grid_distance=500
 
    dictdist={}
    G2 = nx.MultiDiGraph(name='G2', crs={'init':'epsg:4326'})

    for n,d in G.nodes(data=True):
        g_und.node[n]['group']=-1
            
    # Going to be adding our own nodes, we'll begin counting them from the most recent
    osmid_counter = max(g_und.nodes())+1

    # Used for an average calculation at the end
    totlen=0
    nedges=0
    
    d=float(float(min_dist)/1000)

    print("Elaborating city graph. This can take a minute...")
   
    for u,v,data in list(g_und.edges(data=True)):
        x1=g_und.node[u]['x']
        y1=g_und.node[u]['y']
        y2=g_und.node[v]['y']
        x2=g_und.node[v]['x']
        
        
        dista=ox.great_circle_vec(y1,x1,y2,x2)
        
        totlen= totlen + dista
        nedges=nedges+1

        # If the distance travelled by that node less than the min distance
        # we can travel, then no need to append new nodes
        if dista< min_dist or (y1==y2 and x1==x2):
            continue

        # rel: How many points lie between origin (x1,y1) to destination (x2,y2)
        rel=int(dista/min_dist)
        ty=y1
        tx=x1

        # Calculate our current bearing so we can point the additional nodes in the
        # right direction
        bearing = calculateBearing(g_und.node[u], g_und.node[v])

        # Origin (x1,y1) to destination (x2,y2), increment distance by k time the amount
        # we can travel, use this and bearing to calculate a destination node (x3,y3).
        # Mark distance between the last new node as "dist", distance from origin as "du",
        # and distance to destination as "dv".
        # No idea what the hell "dec" is supposed to be.
        for k in range (0,rel):
            distance= d*(k+1)
            origin = geopy.Point(y1, x1)
            destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
            y3, x3 = destination.latitude, destination.longitude            
            
            dist=ox.great_circle_vec(ty,tx,y3,x3)
            
            du=ox.great_circle_vec(y1,x1,y3,x3)
            dv=ox.great_circle_vec(y2,x2,y3,x3)
            dec=math.sqrt( (x3 - tx)**2 + (y3 - ty)**2 )

            # u,v:   Actual-node from and to
            # d:     distance since last node we created
            # x,y:   latitude and longitude of this new node
            # du,dv: distance from origin and to destination
            # group: gutted this, previously used to generate antennas though
            G2.add_node(osmid_counter,y=y3,x=x3,osmid=osmid_counter,u=u,v=v,d=dist,dec=dec,du=du,dv=dv,group="any")
                       
            tx=x3
            ty=y3

            osmid_counter=osmid_counter+1

    print ("Average Length of Edges : ",totlen/nedges)
    print ("Number of nodes before: ", len(g_und.nodes()))
    print ("Number of nodes after: ", len(G2.nodes()))
    return G2

print("Getting city graph...")            
add_points(getCityGraph("Aylmer, Quebec, Canada"))
print("Success")
