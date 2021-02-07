import osmnx as ox
import pandas as pd
import networkx as nx
import numpy as np
import random
import math
from tqdm import tqdm
from menucl import main_menu1,menu2
import time
import shutil
import os,sys
import folium
from folium.features import DivIcon
import folium.plugins as plugins


import geopy
from geopy.distance import VincentyDistance

name_city='luxembourg city'
ox.config(use_cache=True)
#name_city='morlupo'
ray=50
max_dist=1500
decision='y'
type_net='walk'
num_usr=2000
days=2
min_speed=1.0
max_speed=1.5
grid_distance=500
max_osmid=0
newosmid=0
max_walk_time=40
maxlat=0
maxlong=0
minlat=999999999
minlong=999999999
maxlen=0
cud=0
list_group=[]

def checkcontact(point,listpoints):
    count=0
    ngh=0
    for itm in listpoints:
        if(itm[2]!=point[2]):
            if(itm[2]!=ngh):
                dist=ox.great_circle_vec(point[0],point[1],itm[0],itm[1])
                if(dist<ray):
                    count+=1
                    ngh=itm[2]
                    
    
    
    return count

def add_points(G3,min_dist):

    global maxlen
    global maxlat
    global maxlong
    global minlat
    global minlong
    dictdist={}
    G2 = nx.MultiDiGraph(name='G2', crs={'init':'epsg:4326'})
    for  n,d  in G.nodes(data=True):
        x=d['x']
        y=d['y']
        G3.node[n]['group']=-1

        if(y<minlat):
            minlat=y
        if(y>maxlat):
            maxlat=y
        if(x<minlong):
            minlong=x
        if(x>maxlong):
            maxlong=x
            
    dist_or=ox.great_circle_vec(maxlat,minlong,maxlat,maxlong)
    dist_ver=ox.great_circle_vec(minlat,maxlong,maxlat,maxlong)

    grid_long=int(dist_or/grid_distance)
    grid_lat=int(dist_ver/grid_distance)
    

    if(grid_lat!=0):
        diff_ver=(maxlat-minlat)/grid_lat

    else:
        diff_ver=maxlat
        grid_lat=1
    
    if(grid_long !=0):
        diff_or=(maxlong-minlong)/grid_long 
    else:
        diff_or=maxlong
        grid_long=1
    
    ng=grid_lat*grid_long
    for i in range(0,(grid_long*grid_lat)):
        list_group.append(list())    

    global max_osmid,newosmid
    err=min_dist/2
    max_osmid=max(G3.nodes())+1
    newosmid=max_osmid
    ed_osmid=0
    r=0
    totlen=0
    nedges=0
    
        
    list_edges=list(G3.edges(data=True))
    
    pbar = tqdm(total=len(list_edges))
    d=float(float(min_dist)/1000)
   
    for u1,v1,dat in list_edges:
        
        if dat['length']> maxlen:
            maxlen=dat['length']
            
        pbar.update(1);
        
        flag=0
        r=r+1
        i=0
        
        R=6371009

        x1=G3.node[u1]['x']
        x2=G3.node[v1]['x']
        y1=G3.node[u1]['y']
        y2=G3.node[v1]['y']
        
        
        if(y1==maxlat):
            row=grid_lat-1
        else:
            row=int((y1-minlat)/diff_ver)

        if(x1==maxlong):
            col=grid_long-1
        else:
            col=int((x1-minlong)/diff_or)

        groupu=(grid_long*row)+col



        nodeu = {}
        nodeu['y'] = y1
        nodeu['x'] = x1
        nodeu['osmid'] =  G3.node[u1]['osmid']
        nodeu['u']=u1
        nodeu['v']=u1
        nodeu['d']=-1
        nodeu['dec']=0
        nodeu['du']=0
        nodeu['dv']=0        
        nodeu['group']=groupu
        
        lat1 = y1 * math.pi / 180 
        lat2 = y2 * math.pi / 180 
        
        long1 = x1 * math.pi / 180 
        long2 = x2 * math.pi / 180 
        
        dista=ox.great_circle_vec(y1,x1,y2,x2)
        
        totlen= totlen + dista
        nedges=nedges+1
        if dista< min_dist or (y1==y2 and x1==x2):
            continue
        
        rel=int(dista/min_dist)
        ty=y1
        tx=x1
        
        
        bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360        
        
        for k in range (0,rel):
            distance= d*(k+1)
            origin = geopy.Point(y1, x1)
            destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
            y3, x3 = destination.latitude, destination.longitude            
            
            
            
            dist=ox.great_circle_vec(ty,tx,y3,x3)
            
            du=ox.great_circle_vec(y1,x1,y3,x3)
            dv=ox.great_circle_vec(y2,x2,y3,x3)
            dec=math.sqrt( (x3 - tx)**2 + (y3 - ty)**2 )
            node = {}
            node['y'] = y3
            node['x'] = x3
            node['osmid'] = newosmid
            node['u']=u1
            node['v']=v1
            node['d']=dist
            node['dec']=dec
            node['du']=du
            node['dv']=dv
            node2 = {}
            if(y3==maxlat):
                row=grid_lat-1
            else:
                row=int((y3-minlat)/diff_ver)
            if(x3==maxlong):
                col=grid_long-1
            else:
                col=int((x3-minlong)/diff_or)
            group=(grid_long*row)+col

            node['group']=group
            list_group[group].append(newosmid)

            G2.add_node(newosmid,y=y3,x=x3,osmid=newosmid,u=u1,v=v1,d=dist,dec=dec,du=du,dv=dv,group=group)
            deglen=110.25
            x = ty - y3
            y = (tx - x3)*math.cos(math.radians(y3))
            
                       
            tx=x3
            ty=y3

            ed_osmid=ed_osmid+1
            i=i+1
            newosmid=newosmid+1

    pbar.close()  
    print ("Average Lenght of Edges : ",totlen/nedges)      
    return G2

# HONOURS: Bullshit file parsing here
with open('Setup.txt', 'r') as data:
    count=0
    for line in data:
        if count==1:
            p=line.split()
            days=int(p[4])

        if count==4:
            r=line.split()
            num_usr=int(r[4])

        if count==22:
            r=line.split()
            default=int(r[5])
            print ('\nDefault choice = ',default)

        if count==18:
            r=line.split()
            antenna_decision=int(r[5])
        if count==9:
            r=line.split()
            numhours=int(r[4])
        if count==12:
            r=line.split()
            endhour=int(r[4])
            
        count+=1

if default==0:
    shutil.copy2('./Inputs/saved/DefaultList/route_usr_1day_0.html','/var/www/html/CrowdSenSim/route_usr_1day_0.html')
    sys.exit(0)
    
saved=[]
with open('./Inputs/SavedList.txt', 'r') as data:
    for line in data:
        
        listsaved={}
        p=line.split()
        listsaved['id']=int(p[0])
        listsaved['name']=p[1]
        listsaved['us']=int(p[2])
        listsaved['days']=int(p[3])
        saved.append(listsaved)
 
try:
    saved,name_city,num_usr,days=main_menu1(saved,num_usr,days)
except SystemExit:
    print ('Exit from CrowdSenSim')
    sfile = open('./Inputs/SavedList.txt', 'w')
    for item1 in saved:
        sfile.write(" %s %s %s %s\n" % (item1['id'],item1['name'],item1['us'],item1['days']))
    
    sfile.close()    
    sys.exit(1)
    
    
aFile='Setup.txt'
shutil.move( aFile, aFile+"~" )

destination= open( aFile, "w" )
source= open( aFile+"~", "r" )
count=0
for line in source:
  
    if count ==4:
        destination.write("|Number of users| = "+str(num_usr) + "\n" )
    elif count==1:
        destination.write("|Days of simulation| = "+str(days)+ "\n" )
    else:
        destination.write( line )
    count+=1

source.close()
destination.close()  
    

if  name_city!='no' :
     
    while(True):
        
        print ('Antenna choice = ',antenna_decision )
        print ('Number of Days = ',days)
        print ('Number of Users = ',num_usr)
      
    
        try:    
            print ('Downloading map of ***',name_city,'*** ................')
            G = ox.graph_from_place(name_city,network_type=type_net,simplify=False)
            break
        except:
            print ('Wrong city name, please retry')
            time.sleep(2)
            name_city=menu2()
            
            
    while(True):    
        choice=input('\nDo you want to save it? (y/n)     ')
        ch = choice.lower()
        if ch!='y' and ch !='n':
            print ('Wrong selection, please retry')
        else :
            break
    if ch=='y':
        if len(saved)>0:
            index=int(saved[-1]['id'])
        else:
            index=-1
        index+=1
        try:
            os.makedirs('./Inputs/saved/'+str(index)+'list')
        except:
            pass
        list_eve_to_save={}
        list_eve_to_save['id']=index
        list_eve_to_save['name']=name_city.replace(" ","_")
        list_eve_to_save['us']=num_usr
        list_eve_to_save['days']=days
        saved.append(list_eve_to_save)
        
        setfile = open('./Inputs/saved/'+str(index)+'list/Setup.txt', 'w')
        setfile.write("%s %s " % (num_usr,days))   
        setfile.close()
        


    # HONOURS: Uncomment to see the graph of the city.
    # Broken af, relying on fucking ancient version of the OSMNX lib
    #ox.plot_graph(G)
    
    G_und = G.to_undirected()

    print ('Elaborating Map................')
    start = time.time()
    
   
    G_big=add_points(G_und,3)
    print ("Number nodes after algorithm: ",len(G_big.nodes()))
    
    end = time.time()
    print((end - start),'  <-----Algorithm  Time (seconds) '     )

    
    G_und=None
    if(antenna_decision==1):
        c=0
        afile = open('./Inputs/CoordinatesAntennas.txt', 'w')
        afile.write("/ID-Antenna/-/Lat/-/Long/\n")
        for i in range(0,len(list_group)):

            if len(list_group[i])!=0:
                c+=1
                antenna=random.choice(list_group[i])
                afile.write("%s %s %s\n" % (c,float(G_big.node[antenna]['y']), float(G_big.node[antenna]['x'])))

    afile.close()
    if ch=='y':
        shutil.copy2('./Inputs/CoordinatesAntennas.txt', './Inputs/saved/'+str(index)+'list/CoordinatesAntennas.txt')
        
    listhours=np.loadtxt("./Inputs/hours.txt")
    perc_hour=[]
    
    hr=12
    hr=hr+((days-1)*24)
    sumconts=0
    random0_or_tracce1=0
    
    if random0_or_tracce1==0:    
        for i in range(0,hr):
            listhours[i]=15
    
    
    for i in range(0,hr):
        sumconts+=listhours[i]
    
    perctot=0.0
    perc_hour.append(0)

    
    for i in range(0,hr):
        perc=float(listhours[i])/float(sumconts)
        perctot+=perc
        perc_hour.append(perc*100)
      
    
    perc_hour[0]=((float(perctot)/float(hr))*100)
    
    
    G_imp=G_big
    G_old=G
    eventfile=[]
    for d in range(0,days):
        f = open('./Inputs/Mobility/UserMovementsListEvents_'+str(d)+'.txt', 'w')
        f.write("/ID-User/-/Lat/-/Long/-/Alt/-/Day/-/Hour/-/Minute/- \n")
        eventfile.append(f)
        
        
    edgeid=0
    numexc=0
    countnext=0
    contactavg=0.0
    act_usr=1
    perc_rem=100
    perc_used=0
    userused=0
    userlimit=num_usr*0.9
    conts_expected=num_usr
    num_usr_init=num_usr
    countcont=0
    print ("\nCreating List of Event (list of movements of each user)")
    
    pbar = tqdm(total=num_usr)
    userus=0
    if random0_or_tracce1==0:
        flag_first=0
    else:
        flag_first=2
    fstat = open('./Outputs/stats_out.txt', 'w') 
    #nodiroutes = open('nodiroutes.txt', 'w')
    
    while True:
        
        num_usr=num_usr_init-userused
        
        day=0
        
        countnext=0
        contactavg=0.0
        perc_rem=100
        perc_used=0        
        userus=0
        for h in range(0,hr+2):
           
            if h==0:
                
                minutlist=[]
                for i in range(0,105):
                    minutlist.append(list())
            
            else:
                
                for i in range (0,105):
                    if(i<45):
                        minutlist[i]=minutlist[60+i]
                    else:
                        minutlist[i]=[]
                        
            if h==0 and flag_first==0:
                continue                
                
            if h==1 :
                if flag_first==2:
                    flag_first=1
                    userused=0
                if countcont==0 and perc_hour[0]!=0:
                    countcont=1                
                contfirst=countcont
                userus=0
                perc_used=0
                perc_rem=100
                countnext=0
                
            if h!=0:

                ##contactavg=float(countcont/usrtmp)
                ##print ("number of contacts:  ",countcont,"  HOUR : ",h-1)
                if(h==hr+1):
                    break
                conts_expected=(perc_hour[h]*contfirst)/perc_hour[0]
                
            ## HEATMAP start
            if h==2:
                listheat=[]
                for i in range(0,60):
                    listheat.append(list())
            
            if h==3:
                m = folium.Map([minlat,minlong], tiles='stamentoner', zoom_start=6)
                
                hm = plugins.HeatMapWithTime(
                    listheat,
                    auto_play=True
                )
                
                hm.add_to(m)
                
                bounds = [(maxlat,maxlong), (minlat,minlong)]
                m.fit_bounds(bounds)                
                
                filepath = '/var/www/html/CrowdSenSim/heat.html'
                m.save(filepath)                
            ## Heatmap END
            countcont=countnext
            countnext=0
            usrtmp=0
           
            num_eve=0
            
            perc_used+=perc_hour[h]
            perc_rem-=perc_hour[h]
            
            while True:
          
                if flag_first==1  and h>6:
                    remaining=((perc_rem)*userused)/perc_used
                    if (remaining+userus)>num_usr:
                        print ("terminated for users ",userus)
                        print ("Hour ",h," Users ",usrtmp," Contact ", countcont," next ",countnext,file=fstat )
                        break
                
                if flag_first==0 or flag_first==2:
                    if (usrtmp >= int(round((float(num_usr)/100)*perc_hour[h]))) or userused==num_usr_init:
                        print ("Hour ",h," Users ",usrtmp," Contact ", countcont," next ",countnext,file=fstat)
                        break                        
                    
                if flag_first==1:
                    if (countcont >= int(conts_expected)):
                        print ("terminated for Contact ",userus)
                        print ("Hour ",h," Users ",usrtmp," Contact ", countcont," next ",countnext,file=fstat)
                        break
               
                userused+=1
                usrtmp+=1
                userus+=1
                count_minute=0 
                totd=0
                    
                    
                flag=0
                if flag_first!=2:
                    pbar.update(1)
    
                origin_node=random.randint(max_osmid+1,newosmid-1)
                orig=G_imp.node[origin_node]
    
    
    
                if int(orig['u'])!=origin_node:
                    flag=1
                    node_or = {}
                    node_or['y'] = orig['y']
                    node_or['x'] = orig['x']
                    node_or['osmid'] = origin_node
                    G_old.add_node(origin_node,y=orig['y'],x=orig['x'],osmid=origin_node)
                    edgeid+=1
                    G_old.add_edge(u=int(orig['u']),v=origin_node,key=0,highway='unclassified',length=float(orig['du']),oneway=False,osmid=edgeid)
                    edgeid+=1
                    G_old.add_edge(u=origin_node,v=int(orig['v']),key=0,highway='unclassified',length=float(orig['dv']),oneway=False,osmid=edgeid)
    
    
                group_or=int(G_imp.node[origin_node]['group'])
    
                speed = random.uniform(min_speed,max_speed)
                
                mincamm=random.randint(20,40)                
                cut=mincamm*60*speed
                cutadded=cut+maxlen
                
                (length, path)= nx.single_source_dijkstra(G_old, origin_node, target=None, cutoff=cutadded, weight='length')
                idr=0
                for l in length:
                    if length[l]>cut:
                        idr=l
                    
                if idr==0:
                    idr=max(length, key=length.get)
                    
                ##print (cut-length[idr])
                ##route = nx.shortest_path(G_old, origin_node, destination_node)
                route=path[idr]
       
                if userused==1 :
                    avl=len(route)/2
                    graph_centroid = (G_old.node[route[int(avl)]]['y'],G_old.node[route[int(avl)]]['x'])
                    route_map = folium.Map(location=graph_centroid, zoom_start=13)
                    for idroute in range(1,len(route)):
                        location=[(G_old.node[route[idroute-1]]['y'],G_old.node[route[idroute-1]]['x']),(G_old.node[route[idroute]]['y'],G_old.node[route[idroute]]['x'])]
                        pl = folium.PolyLine(locations=location,color='red')
                        
                        pl.add_to(route_map)
                    #icon=DivIcon(icon_size=(150,36),html='<div style="font-size: 16pt;color:red">start</div>')
                    di= folium.map.Marker((G_old.node[route[1]]['y'],G_old.node[route[1]]['x']),popup='Start')
                    di.add_to(route_map)
                    
                    di= folium.map.Marker((G_old.node[route[idroute]]['y'],G_old.node[route[idroute]]['x']),popup='End')
                    di.add_to(route_map)

                    # HONOURS: Wouldn't count on this one working either >_>
                    #route_map = ox.plot_route_folium(G_old, route)
                    filepath = '/var/www/html/CrowdSenSim/route_usr_1day_0.html'
                    route_map.save(filepath)
                    if ch=='y':
                        shutil.copy2('/var/www/html/CrowdSenSim/route_usr_1day_0.html', './Inputs/saved/'+str(index)+'list/route_usr_1day_0.html')                
    
                len_edges=ox.get_route_edge_attributes(G_old,route, attribute='length', minimize_key='length')
               
                minutes=random.randint(0,59)
                tm=minutes
                hours = (h+11)%24
                if hours==23:
                    minutes=random.randint(0,40)                
                day=int((h+11)/24)
    
                seconds=0
                totsec=0
                if flag_first!=2:
                    print  (userused,G_old.node[route[0]]['y'],G_old.node[route[0]]['x'],0,day,hours,minutes,file=eventfile[day])
                num_eve+=1	
    
                ind=0
                avgbear=0
                for n in  len_edges:
                    
                    if n!=n:
                        continue
                    
                    lat1 = G_old.node[route[ind]]['y'] * math.pi / 180 
                    lat2 = G_old.node[route[ind+1]]['y'] * math.pi / 180 
                    long1 = G_old.node[route[ind]]['x'] * math.pi / 180 
                    long2 = G_old.node[route[ind+1]]['x'] * math.pi / 180                             
                    bearing1 = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
                    bearing1 = math.degrees(bearing1)
                    bearing1 = (bearing1 + 360) % 360                     
                    

                    # HONOURS: Looks like old debugging line
                    #print(userused,G_old.node[route[ind]]['y'],G_old.node[route[ind]]['x'],file=nodiroutes)
                    addseconds = int(n/speed)
                    totsec=addseconds+seconds
                    addmin=int(totsec/60)
                    count_minute=count_minute+addmin
                    seconds=totsec%60
                    
                    if count_minute>mincamm:
                     
                        lat1 = G_old.node[route[ind]]['y'] * math.pi / 180 
                        lat2 = G_old.node[route[ind+1]]['y'] * math.pi / 180 
                        long1 = G_old.node[route[ind]]['x'] * math.pi / 180 
                        long2 = G_old.node[route[ind+1]]['x'] * math.pi / 180                             
                        bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
                        bearing = math.degrees(bearing)
                        bearing = (bearing + 360) % 360 
                        nd=G_old.node[route[ind]]
                        distance=0
                        for m in range(1,mincamm-(count_minute-addmin)):
                            minutes+=1
                            tm+=1
                            if minutes >= 60:
                                hours=hours+1
                                minutes=minutes-60
                            
                                if hours==24:
                                    break                            
                           
                            distance= float(m*(speed*60))/1000
                            origin = geopy.Point((nd['y']), (nd['x']))
                            destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
                            y, x = destination.latitude, destination.longitude                               
                            
                            point=[y,x,userused]
                            
                            if flag_first!=0:
                                if tm>59:
                                    countnext+=checkcontact(point,minutlist[tm])
                                else:
                                    countcont+=checkcontact(point,minutlist[tm])
                                minutlist[tm].append(point)
                            if flag_first!=2:
                                print (userused,point[0],point[1],0,day,hours,minutes,count_minute,bearing1,distance,file=eventfile[day])
                        
                        avgbear= avgbear +  (bearing1*distance)   
                        totd=totd+distance
                        break
                    avgbear= avgbear +  (bearing1*n)
                    totd=totd+n
                    d=day
                    if addmin >1:
                        lat1 = G_old.node[route[ind]]['y'] * math.pi / 180 
                        lat2 = G_old.node[route[ind+1]]['y'] * math.pi / 180 
                        long1 = G_old.node[route[ind]]['x'] * math.pi / 180 
                        long2 = G_old.node[route[ind+1]]['x'] * math.pi / 180                             
                        bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
                        bearing = math.degrees(bearing)
                        bearing = (bearing + 360) % 360 
                        nd=G_old.node[route[ind]]                        
                        for k in range(1,addmin):
                            numexc+=1
                            minutes+=1
                            tm+=1
                            if minutes >= 60:
                                hours=hours+1
                                minutes=minutes-60
                            
                            if hours==24:
                                break                            
                            distance= float(k*(speed*60))/1000
                            origin = geopy.Point((nd['y']), (nd['x']))
                            destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
                            y, x = destination.latitude, destination.longitude                               
                            
                            point=[y,x,userused]
                                
                            if flag_first!=0:
                                if tm>59:
                                    countnext+=checkcontact(point,minutlist[tm])
                                else:
                                    countcont+=checkcontact(point,minutlist[tm])
                                minutlist[tm].append(point)
                            if flag_first!=2:
                                print (userused,point[0],point[1],0,day,hours,minutes,count_minute,bearing1,n,file=eventfile[day] )
                                num_eve+=1
                            
                    
                    if addmin > 0:
                        minutes+=1
                        tm+=1
                        if minutes >= 60:
                            hours=hours+1
                            minutes=minutes-60
                        
                            if hours==24:
                                break
                        nd=G_old.node[route[ind+1]]
                        point=[(nd['y']),(nd['x']),userused]
                        if flag_first!=0:
                            if tm>59:
                                countnext+=checkcontact(point,minutlist[tm])
                            else:
                                countcont+=checkcontact(point,minutlist[tm])
                            minutlist[tm].append(point)
                        if flag_first!=2:
                            print (userused,point[0],point[1],0,day,hours,minutes,count_minute,bearing1,n,file=eventfile[day])
                            if(h==2):
                                listheat[minutes].append([point[0],point[1]])
                                
                        num_eve+=1
                        
                    ind+=1	
    
    
                lat1 = G_old.node[route[0]]['y'] * math.pi / 180 
                lat2 = point[0] * math.pi / 180 
                long1 = G_old.node[route[0]]['x'] * math.pi / 180 
                long2 = point[1] * math.pi / 180                             
                bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
                bearing = math.degrees(bearing)
                bearing = (bearing + 360) % 360                 
                avgbear=avgbear/totd
                #print("BEARINGS",bearing,avgbear,point[0],point[1])
                if flag==1 or flag==3:
                    G_old.remove_edge(int(orig['u']),int(origin_node))
                    G_old.remove_edge(int(origin_node),int(orig['v']))
    
    
                    G_old.remove_node(origin_node)
    
    
        if(userused>=num_usr):
            if cud==1:
                userused=save_users
                p=float(100/float(len(perc_hour)-1))
                for q in range(0,len(perc_hour)):
                    perc_hour[q]=p
                
            else:
                #print ("\n user distibuted >> ",userused," user tot >> ",num_usr)
                break
            cud=0
            
        save_users=userused
        flag_first=0
        
        
        ############
        ############
###########LAST FILLING
        ############
        ############
    while (userused<num_usr_init):
       
       
        userused+=1
     
        count_minute=0                    
            
        flag=0

        origin_node=random.randint(max_osmid+1,newosmid-1)
        orig=G_imp.node[origin_node]


        if int(orig['u'])!=origin_node:
            flag=1
            node_or = {}
            node_or['y'] = orig['y']
            node_or['x'] = orig['x']
            node_or['osmid'] = origin_node
            G_old.add_node(origin_node,y=orig['y'],x=orig['x'],osmid=origin_node)
            edgeid+=1
            G_old.add_edge(u=int(orig['u']),v=origin_node,key=0,highway='unclassified',length=float(orig['du']),oneway=False,osmid=edgeid)
            edgeid+=1
            G_old.add_edge(u=origin_node,v=int(orig['v']),key=0,highway='unclassified',length=float(orig['dv']),oneway=False,osmid=edgeid)


        group_or=int(G_imp.node[origin_node]['group'])

        
        speed = random.uniform(min_speed,max_speed)
        
        mincamm=random.randint(20,40)                
        cut=mincamm*60*speed
        cutadded=cut+maxlen
        
        (length, path)= nx.single_source_dijkstra(G_old, origin_node, target=None, cutoff=cutadded, weight='length')
        idr=0
        for l in length:
            if length[l]>cut:
                idr=l
            
        if idr==0:
            idr=max(length, key=length.get)
            

        # HONOURS: Seems relevant for something? Old shortest path prior to (length, path) I guess
        ##route = nx.shortest_path(G_old, origin_node, destination_node)
        route=path[idr]


        len_edges=ox.get_route_edge_attributes(G_old,route, attribute='length', minimize_key='length')
        
        minutes=random.randint(0,59)
        tm=minutes
        hours = (hr+11)%24
        if hours==23:
            minutes=random.randint(0,40)
        day=int((hr+11)/24)
        
        seconds=0
        totsec=0
        print (userused,G_old.node[route[0]]['y'],G_old.node[route[0]]['x'],0,day,hours,minutes,file=eventfile[day])
        
        num_eve+=1	

        ind=0
        
        for n in  len_edges:
            if n!=n:
                continue
            
            
            addseconds = int(n/speed)
            totsec=addseconds+seconds
            addmin=int(totsec/60)
            count_minute=count_minute+addmin
            seconds=totsec%60
            
            if count_minute>mincamm:
        
                lat1 = G_old.node[route[ind]]['y'] * math.pi / 180 
                lat2 = G_old.node[route[ind+1]]['y'] * math.pi / 180 
                long1 = G_old.node[route[ind]]['x'] * math.pi / 180 
                long2 = G_old.node[route[ind+1]]['x'] * math.pi / 180                             
                bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
                bearing = math.degrees(bearing)
                bearing = (bearing + 360) % 360 
                nd=G_old.node[route[ind]]
        
                for m in range(1,mincamm-(count_minute-addmin)):
                    minutes+=1
                    tm+=1
                    if minutes >= 60:
                        hours=hours+1
                        minutes=minutes-60
        
                        if hours==24:
                            break                            
        
                    distance= float(m*(speed*60))/1000
                    origin = geopy.Point((nd['y']), (nd['x']))
                    destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
                    y, x = destination.latitude, destination.longitude                               
        
                    point=[y,x,userused]
        
        
        
                    if flag_first!=0:
                        if tm>59:
                            countnext+=checkcontact(point,minutlist[tm])
                        else:
                            countcont+=checkcontact(point,minutlist[tm])
                        minutlist[tm].append(point)
                    if flag_first!=2:
                        print (userused,point[0],point[1],0,day,hours,minutes,count_minute,file=eventfile[day])                            
                break
            d=day
            if addmin >1:
                lat1 = G_old.node[route[ind]]['y'] * math.pi / 180 
                lat2 = G_old.node[route[ind+1]]['y'] * math.pi / 180 
                long1 = G_old.node[route[ind]]['x'] * math.pi / 180 
                long2 = G_old.node[route[ind+1]]['x'] * math.pi / 180                             
                bearing = math.atan2(math.sin(long2-long1)*math.cos(lat2), math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(long2-long1))
                bearing = math.degrees(bearing)
                bearing = (bearing + 360) % 360 
                nd=G_old.node[route[ind]]                        
                for k in range(1,addmin):
                    numexc+=1
                    minutes+=1
                    tm+=1
                    if minutes >= 60:
                        hours=hours+1
                        minutes=minutes-60
        
                        if hours==24:
                            break                            
                    distance= float(k*(speed*60))/1000
                    origin = geopy.Point((nd['y']), (nd['x']))
                    destination = VincentyDistance(kilometers=distance).destination(origin, bearing)
                    y, x = destination.latitude, destination.longitude                               
        
                    point=[y,x,userused]
        
                    if flag_first!=0:
                        if tm>59:
                            countnext+=checkcontact(point,minutlist[tm])
                        else:
                            countcont+=checkcontact(point,minutlist[tm])
                        minutlist[tm].append(point)
                    if flag_first!=2:
                        print (userused,point[0],point[1],0,day,hours,minutes,count_minute,file=eventfile[day] )
                        num_eve+=1
                    
            
            if addmin > 0:
                minutes+=1
                tm+=1
                if minutes >= 60:
                    hours=hours+1
                    minutes=minutes-60
                
                    if hours==24:
                        break
                nd=G_old.node[route[ind+1]]
                point=[(nd['y']),(nd['x']),userused]


                print (userused,point[0],point[1],0,day,hours,minutes,file=eventfile[day] )
                num_eve+=1
                
            ind+=1	


        if flag==1 or flag==3:
            G_old.remove_edge(int(orig['u']),int(origin_node))
            G_old.remove_edge(int(origin_node),int(orig['v']))

            G_old.remove_node(origin_node)

        

    # HONOURS: Original author makes this call to let STDOUT dump to the user
    time.sleep(2)

    # HONOURS: Just do it properly instead:
    sys.stdout.flush()
    
    pbar.close	
    for d in range(0,days):
        eventfile[d].close()    
        if ch=='y':
            shutil.copy2('./Inputs/Mobility/UserMovementsListEvents_'+str(d)+'.txt', './Inputs/saved/'+str(index)+'list/UserMovementsListEvents_'+str(d)+'.txt')
    
    
else:
   
        
    print ('Event List loaded *****')

sfile = open('./Inputs/SavedList.txt', 'w')
for item1 in saved:
    sfile.write(" %s %s %s %s\n" % (item1['id'],item1['name'],item1['us'],item1['days']))

sfile.close()