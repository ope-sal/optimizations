import csv
import math
import time
import sys
import os
from pyomo.environ import *
from pyomo.opt import SolverFactory
import pyomo.environ
from xml.dom import minidom

#Global Variables
PC_CSV,PC_XML  = 'input/photocenters.csv','input/PhotoCenters.kml'
AP_CSV,AP_XML = 'input/accesspoints.csv','input/AccessPoints.kml'
OUTPUT_COORD = 'output/flight_path_'
DIST_UNIT = 'meters' #you can use meters or miles
MAX_DISTANCE = 800
photocenters = []
accesspoints = []
partitions = []
AVERAGE_NODE_DISTANCE = 23.3 #average distance between 2 photocenters
AVERAGE_DRONE_SPEED = 9 # * per second
AVERAGE_FLIGHT_TIME = 14 #average flight time(in minutes) for each full charge of the drones
MAX_AGENTS =  4 #Number of drones/agents
MAX_NODES = None #max nodes for each agent - enter None if there is no limit

max_pc =  int(((AVERAGE_DRONE_SPEED * AVERAGE_FLIGHT_TIME * 60) - MAX_DISTANCE)/AVERAGE_NODE_DISTANCE )#294
#the lower the photocenters (max_pc), the longer it'll take to solve the
#optimization problem. max_pc corresponds to the number photocenters that each agent
#can visit. For now the number is compute by:
# [(9m/s(speed of drone) * 14mins(time or flight)) - 800(max_distance from AP to Node)]/23m(~distance between photocenters)

def parse_PC(use_xml=False):
    """Extract the coord information from file of photocenters.
    use_xml=False(default) if the file is csv anfd True if file is xml"""
    if use_xml==True:
        photocenters_dom = minidom.parse(PC_XML)
        list_coord_dom = photocenters_dom.getElementsByTagName('coordinates')
        for coord in list_coord_dom:
            coord_list = coord.childNodes[0].data.split(',')
            #longitude, Latitude
            photocenters.append([float(coord_list[0]), float(coord_list[1])])
    else:
        with open(PC_CSV) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                photocenters.append([float(row['X']), float(row['Y'])])

def parse_AP(use_xml=False):
    """Extract the coord information from csv file of access points.
    use_xml=False(default) if the file is csv anfd True if file is xml"""
    if use_xml==True:
        accesspoints_dom = minidom.parse(AP_XML)
        list_coord_dom = accesspoints_dom.getElementsByTagName('coordinates')
        for coord in list_coord_dom:
            coord_list = coord.childNodes[0].data.split(',')
            #longitude, Latitude
            accesspoints.append([float(coord_list[0]), float(coord_list[1])])
    else:
        with open(AP_CSV) as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                accesspoints.append([float(row['X']), float(row['Y'])])

def compute_distance(pointA, pointB, long_lat=True):
    """Returns the distance between pointA(long,lat) and pointB t"""
    if long_lat == True:
        pointA=tuple(pointA)
        pointB=tuple(pointB)
        return geo_distance(pointA,pointB, DIST_UNIT)

    else:
        return math.sqrt((pointA[0] - pointB[0])**2 + (pointA[1] - pointB[1])**2)

def write_coord(list_coord, ap=0, agent=0):
    """Takes a list of geo coordinates and writes it to OUTPUT_COORD +'ap'+'agent'+'.csv'"""
    filepath = OUTPUT_COORD + '_ap' + str(ap) + '_agt' + str(agent) + '.csv'
    csvfile = open(filepath, 'a')
    if(os.path.getsize(filepath)==0): #if the file is empty
        csvfile.write("longitude,Latitude\n")
    for item in list_coord:
        csvfile.write(repr(item[0])+','+repr(item[1]) + '\n')
    csvfile.close()

def geo_distance(coord1, coord2, units='meters'):
    """computes the distace between 2 geo coordinates entered as long,lat"""
    long1,lat1 = coord1
    long2,lat2 = coord2

    # Convert latitude and longitude to spherical coordinates in radians.
    deg2rad = math.pi/180.0

    # phi = 90 - latitude
    phi1 = (90.0 - lat1)*deg2rad
    phi2 = (90.0 - lat2)*deg2rad

    # theta = longitude
    theta1 = long1*deg2rad
    theta2 = long2*deg2rad

    # Compute spherical distance from spherical coordinates.
    cos = (math.sin(phi1)*math.sin(phi2)*math.cos(theta1 - theta2) +
    math.cos(phi1)*math.cos(phi2))
    try:
        arc = math.acos(cos)
    except:
        arc = 0

    #radius of the earth in meters
    radius = 6371 * 1000
    if units == 'miles':
        radius = 0.621371 * radius

    distance = radius * arc
    return distance

#cost function --> return the distance between 2 Nodes
def cost(i,j, long_lat=True):
    """Returns the distance between pointA and pointB t"""
    if long_lat == True:
        return geo_distance((Nodes[j][0], Nodes[j][1]), (Nodes[i][0], Nodes[i][1]), DIST_UNIT)
    else:
        return math.sqrt((Nodes[j][0] - Nodes[i][0])**2 + (Nodes[j][1] - Nodes[i][1])**2)

def solveVRP(Nodes, max_agents,max_nodes):
    #this is a variation of the Vehicle Routing Problem intended to minimize waiting time
    n = len(Nodes)
    print "\tAgents available: ", max_agents, "\tNodes: ", n

    #Define the graph - This is a 3D graph [((node i, node j),agent k) ...]
    Points = [i for i in range(n)]
    list_of_agents = [i for i in range(max_agents)]
    graph = [(i,j,k) for i in range(n) for j in range(n) if i<j or j==0 for k in range(max_agents)]


    # Creation of a Concrete Model
    model = ConcreteModel()

    #ouse init feature of pyomo to init the costs
    model.use_edge = Var(graph, within=Binary)

    def waiting_time(node, vehicle):
        """Returns the waiting time at the specified node"""
        #remember to consider the actual distance between 2 nodes for this model
        return sum([model.use_edge[(i,j,vehicle)]  for i in range(1,node) for j in range(i+1,node) if (i+1)==j ])

    def objective_rule(model):
        #mtsp_prob += lpSum([cost(edge[0][0],edge[0][1])*waiting_time(edge[0][1],edge[1])  for edge in graph ])
        # minimize total waiting time
        return sum(model.use_edge[node]*waiting_time(node[1],node[2]) for node in graph)
    model.objective = Objective(rule=objective_rule, sense=minimize, doc='Define objective function')

    #We are assuming the node zero is the default starting Node
    #Ensure atmost  m agents depart from node 0
    def startingNode_rule(model, k):
        return sum(model.use_edge[0,j,k] for j in range(n) if j != 0 ) <= 1
      #use <= 1 if you want "at most m agents to depart from node 0" and == if you
      #want exactly m agents to depart from node 0
    model.starting = Constraint(list_of_agents, rule=startingNode_rule, doc='Ensure atmost m agents depart from node 0')

    #Ensure atmost m agents return to node 0
    def endingNode_rule(model, k):
        return sum(model.use_edge[i,0,k] for i in range(n) if i != 0) <= 1
      #use <= 1 if you want "at most m agents to depart from node 0" and == if you
      #want exactly m agents to depart from node 0
    model.ending = Constraint(list_of_agents, rule=endingNode_rule, doc='Ensure atmost m agents return to node 0')

    #Ensure that only one tour enters each nodes
    def singleTourEnter_rule(model, j):
        return sum(model.use_edge[(i,j,k)] for i in range(n) if (i+1)==j or j==0 or i==0 for k in range(max_agents)) == 1
    model.singleTourEnter = Constraint(Points[1:], rule=singleTourEnter_rule, doc='Ensure that only one tour enters each nodes')

    #Ensure that only one tour leaves each nodes
    def singleTourLeave_rule(model, j):
        return sum(model.use_edge[(i,j,k)] for j in range(n) if (i+1)==j or j==0 or i==0 for k in range(max_agents)) == 1
    model.singleTourLeave = Constraint(Points[1:], rule=singleTourLeave_rule, doc='Ensure that only one tour leaves each nodes')

    #Ensure that each agent visits at most "max_nodes" nodes
    if max_nodes != None:
        def maxNodes_rule(model, k):
            return sum(model.use_edge[(i,j,k)] for i in range(n) for j in range(n) if (i+1)==j or i==0 or j==0) <= max_nodes
        model.maxNodes = Constraint(list_of_agents, rule=maxNodes_rule, doc='Ensure that each agent visits at most "max_nodes" nodes')

    #Ensure that same vehicle arrives and departs from each node it serves
    def sameVehicle_rule(model, k, node):
        sum_entering = sum(model.use_edge[((i,node), k)] for i in range(n) if (i+1) == node or i==0 or node==0)
        sum_leaving = sum(model.use_edge[((node,j), k)] for j in range(n) if (node+1) == j or node==0 or j==0)
        return sum_entering - sum_leaving == 0
    model.sameVehicle = Constraint(list_of_agents, Points, rule=sameVehicle_rule, doc='Ensure that same vehicle arrives and departs from each node it serves')

    #subtour elimination
    #find u[i] - u[j] that satifies the constraints. Model U as a LP variable
    model.u = Var(Points, within=NonNegativeIntegers)
    ste_nodes = [(i,j,k) for k in range(max_agents) for i in range(n) for j in range(1,n) if (((i+1)==j) or (j==0) or i==0)]
    def subtourElimination_rule(model, i,j,k):
        return model.u[i] - model.u[j] + n*model.use_edge[(i,j,k)] <= n-1
    model.subtourElimination = Constraint(ste_nodes, rule=subtourElimination_rule, doc='subtour Elimination rule')

    #solve the problem using the gurobi solver. Gurobi solves mixed integer quadratic problems
    opt = SolverFactory("gurobi")
    results = opt.solve(model)
    #sends results to stdout
    #results.write()
    print("\nDisplaying Solution\n" + '-'*60)

    result = []
    #Print result - We are only concerned with Integer values of 1
    #also build the result return list
    print "\tResult: \n"
    for node in graph:
        #print value(use_edge[item]),
        if model.use_edge[node] == 1:
            print '\t', node#, value(use_edge[item])
            result.append(node)
    return result

def totalTime(result):
    '''Returns the total time for flight.'''
    max_num_node = max([len(result[node]) for node in result])
    return max_num_node * AVERAGE_NODE_DISTANCE/AVERAGE_DRONE_SPEED

begin_time = time.time()
#initialization
#delete the files in output folder
filelist = [ f for f in os.listdir("output") ]
for f in filelist:
    os.remove('output/' + f)

print'Parsing...'
#parse the input files and initialize arrays of photocenters and accesspoints
parse_PC(use_xml=True)
parse_AP(use_xml=True)

print 'parse complete!', len(accesspoints), len(photocenters)

#Break the nodes up into super nodes for easier MIQP solving
print('getting nodes')
continue_iterations = True
while(continue_iterations):
    max_points = 0
    max_ap = 0
    #find the access point that covers the most photocenters with MAX_DISTANCE radius
    for i in range(len(accesspoints)):
        counter = 0
        for pc in photocenters:
            if(compute_distance(accesspoints[i], pc) <= MAX_DISTANCE):
                counter += 1
        if counter > max_points:
            max_points = counter
            max_ap = i

    if max_points != 0: #make sure that there is atleast one point
        #record the access point and the photocenters it covers.
        partitions.append([accesspoints[max_ap], max_points, [pc0 for pc0 in photocenters if(compute_distance(accesspoints[max_ap], pc0) <= MAX_DISTANCE)]])
        #remove the photocenters from their corresponding list
        photocenters = [pc0 for pc0 in photocenters if(compute_distance(accesspoints[max_ap], pc0) > MAX_DISTANCE)]

    #remove the access point from corresponding list
    accesspoints.pop(max_ap)

    if(accesspoints == []): #if all the accesspoints have been used
        continue_iterations = False

print "\nAccess points used:"
for item in partitions:
    print '\t', item[0]

#if there are photocenters that couldn't be reach from any of the access Points
#report error
if len(photocenters):
    sys.exit("""Error: Solution not Feasible for this graph -
    Some Points couldn't be reach from any access point""")

#slice up the partitions into smaller super nodes for the VRP
for item in partitions:
    item.append([[item[0]]]) #add access point for the nodes
    j = 0
    for i in range(len(item[2])):
        if (i % max_pc) == 0:
            item[3].append([item[2][i]])
            j += 1
        else:
            item[3][j].append(item[2][i])

copy_partitions = partitions
# Add the starting point for each of the super node as a node for the VRP
NodesList = []
for partion in partitions:
    Nodes = []
    for item in partion[3]:
        Nodes.append(tuple(item[0]))
    NodesList.append(Nodes)

#Print some summary to the screen.
print "\nNodes: \n",
counter = 0
for partion in NodesList:
    print 'AP: {0}'.format(counter),
    counter += 1
    for node in partion:
        print '\t', node

#Solve the problem
result = dict()
ap_counter = 0
for Nodes in NodesList:
    print "\n\nSolution to AP: ", ap_counter
    path = solveVRP(Nodes, MAX_AGENTS,MAX_NODES)
    r_node = []
    for move in path:
        agentNum = move[2]
        nodeIndex = move[0]
        list_coords = copy_partitions[ap_counter][3][nodeIndex]
        write_coord(list_coords,ap_counter,agentNum)

        try:
            result[agentNum] =  result[agentNum] + list_coords
        except:
            result[agentNum] = list_coords
    ap_counter += 1
print "Total flight time is", totalTime(result)

print "Time taken:{0: .2f} seconds".format(time.time() - begin_time)
#print value(sum_leaving.varValue)
