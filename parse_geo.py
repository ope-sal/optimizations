import csv
import math
import time
import sys
from pulp import *
from xml.dom import minidom


PC_XML = 'input/PhotoCenters.kml'
AP_XML = 'input/AccessPoints.kml'
OUTPUT_COORD = 'output/flight_path_'
MAX_DISTANCE = 800 #in meters
photocenters = []
accesspoints = []
partitions = []



def parse_PC_csv(use_xml=False):
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
        with open('photocenters.csv') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                photocenters.append([float(row['X']), float(row['Y'])])

def parse_AP_csv(use_xml=False):
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
        with open('accesspoints.csv') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                accesspoints.append([float(row['X']), float(row['Y'])])

def compute_distance(pointA, pointB, long_lat=True):
    """Returns the distance between pointA(long,lat) and pointB t"""
    if long_lat == True:
        pointA=tuple(pointA)
        pointB=tuple(pointB)
        return distance(pointA,pointB)

    else:
        return math.sqrt((pointA[0] - pointB[0])**2 + (pointA[1] - pointB[1])**2)

def write_coord(list_coord, index=0):
    """Takes a list of geo coordinates and writes it to OUTPUT_COORD +'index'+'.csv'"""
    csvfile = open(OUTPUT_COORD + str(index) + '.csv', 'w')
    csvfile.write("longitude,Latitude")
    for item in list_coord:
        csvfile.write(item[0]+','+item[1])
    csvfile.close()

def distance(coord1, coord2, units='meters'):
    """coomputes the distace between 2 geo coordinates entered as long,lat"""
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
    arc = math.acos(cos)

    #radius of the earth in meters
    radius = 6371 * 1000
    if units == 'miles':
        radius = 0.621371 * radius

    distance = radius * arc
    return distance

begin_time = time.time()
parse_PC_csv(use_xml=True)
parse_AP_csv(use_xml=True)
print len(accesspoints), len(photocenters)
#sys.exit('break for now')

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

#print partitions
print "\nAccess points used:"
for item in partitions:
    print '\t', item[0]

#if there are photocenters that couldn't be reach from any of the access Points
#report error
if len(photocenters):
    sys.exit("""Error: Solution not Feasible for this graph -
    Some Points couldn't be reach from any access point""")

max_pc =  294 #the lower the photocenters (max_pc), the longer it'll take to solve the
#optimization problem. max_pc corresponds to the number photocenters that each agent
#can visit. For now the number is compute by:
# [(9m/s(speed of drone) * 14mins(time or flight)) - 800(max_distance from AP to Node)]/23m(~distance between photocenters)

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

# Add the startig point for each of the super node as a node for the VRP
NodesList = []
for partion in partitions:
    Nodes = []
    for item in partion[3]:
        #for x in item:
        #    print str(x[0])+','+str(x[1])
        Nodes.append(tuple(item[0]))
    NodesList.append(Nodes)
#sys.exit('')
#Print some summary to the screen.
print "\nNodes: \n",
counter = 0
for partion in NodesList:
    print 'AP: {0}'.format(counter),
    counter += 1
    for node in partion:
        print '\t', node


#cost function --> return the distance between 2 Nodes
def cost(i,j, long_lat=True):
    """Returns the distance between pointA and pointB t"""
    if long_lat == True:
        return distance((Nodes[j][0], Nodes[j][1]), (Nodes[i][0], Nodes[i][1]))
    else:
        return math.sqrt((Nodes[j][0] - Nodes[i][0])**2 + (Nodes[j][1] - Nodes[i][1])**2)


def solveVRP(Nodes):
    n = len(Nodes)
    #Number of drones/agents
    max_agents = 2
    print "\tAgents available: ", max_agents, "\tNodes: ", n

    #max nodes for each agent
    max_nodes = 5

    #Define the graph - This is a 3D graph [((node i, node j),agent k) ...]
    Points = [i for i in range(n)]
    graph = [(i,j) for i in range(n) for j in range(n) if (i+1)==j or j==0 or i==0]
    list_of_agents = [i for i in range(max_agents)]
    graph = [(i,j) for i in graph for j in list_of_agents ]
    #print graph

    #binary variable (xij) for ILP  - will take on the value of 1 if path ij is taken by vehicle k and 0 otherwise
    use_edge = LpVariable.dicts("UseEdge", graph, lowBound = 0,  upBound = 1, cat = pulp.LpInteger)

    # Setup the instance of the model for minimization problem
    mtsp_prob = LpProblem("MTSP-Model", LpMinimize)

    #objective
    #minimize cost
    mtsp_prob += lpSum([cost(edge[0][0], edge[0][1]) * use_edge[edge] for edge in graph]) #- lpSum([cost(0,j)*use_edge[((0,j),k)] for j in range(n) if j!=0 for k in range(max_agents)])

    #constraints
    #We are assuming the node zero is the default starting Node
    #Ensure that  m agents depart from node 0
    for k in range(max_agents):
        mtsp_prob += lpSum([use_edge[((0,j),k)] for j in range(n) if j != 0 ]) <= 1
        #use <= 1 if you want "at most m agents to depart from node 0" and == if you
        #want exactly m agents to depart from node 0

    #Ensure that m agents return to node 0
    for k in range(max_agents):
        mtsp_prob += lpSum([use_edge[((i,0),k)] for i in range(n) if i != 0 ]) <= 1
        #use <= 1 if you want "at most m agents to depart from node 0" and == if you
        #want exactly m agents to depart from node 0

    #Ensure that only one tour enters each nodes
    for j in range(1,n):
        mtsp_prob += lpSum([use_edge[((i,j), k)] for i in range(n) if (i+1)==j or i==0 or j==0 for k in range(max_agents)]) == 1

    #Ensure that only one tour leaves each nodes
    for i in range(1,n):
        mtsp_prob += lpSum([use_edge[((i,j), k)] for j in range(n) if (i+1)==j or i==0 or j==0 for k in range(max_agents)]) == 1

    #Ensure that each agent visits at most "max_nodes" nodes
    for k in range(max_agents):
        mtsp_prob += lpSum(use_edge[((i,j), k)] for i in range(n) for j in range(n) if (i+1)==j or i==0 or j==0) <= max_nodes

    #Ensure that same vehicle arrives and departs from each node it serves
    for k in range(max_agents):
        for node in range(n):
            sum_entering = lpSum([use_edge[((i,node), k)] for i in range(n) if (i+1) == node or i==0 or node==0])
            sum_leaving = lpSum([use_edge[((node,j), k)] for j in range(n) if (node+1) == j or node==0 or j==0])
            mtsp_prob +=  sum_entering - sum_leaving == 0

    #subtour elimination
    #find u[i] - u[j] that satifies the constraints. Model U as a LP variable
    u = LpVariable.dicts("U - Subtour Elimination", Points, 0, None, LpInteger)
    for k in range(max_agents):
        for i in range(n):
            for j in range(1, n): #constraint not defined for j = 0
                if (((i+1)==j) or (j==0) or i==0 ):
                    mtsp_prob += u[i] - u[j] + n*use_edge[((i,j),k)] <= n-1

    print "\tSolving ...\n"

    status = mtsp_prob.solve()


    print '\tSolution is ', LpStatus[status]
    result = []
    #Print result - We are only concerned with Integer values of 1
    print "\tResult: \n"
    for item in graph:
        #print value(use_edge[item]),
        if value(use_edge[item]) == 1:
            print '\t', item#, value(use_edge[item])
            result.append(item)
    return result


counter = 0
for Nodes in NodesList:
    print "\n\nSolution to AP: ", counter
    counter += 1

    solveVRP(Nodes)

print "Time taken:{0: .2f} seconds".format(time.time() - begin_time)
#print "Number of agent", max_agents
#print value(sum_leaving.varValue)
