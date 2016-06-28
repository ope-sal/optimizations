#Solution to the Vehicle Routing Problem / Multiple Travelling Salesman Problem
from pulp import *
import time
import math

Nodes = [(0,0),(1,1),(2,0),(1,-1),(-1,-1),(-1,0),(-2,1),(-1,2)]
n = len(Nodes)
print "Number of nodes: ", n

#Number of drones/agents
max_agents = 3
print "Number of agents available: ", max_agents

#max nodes for each agent
max_nodes = 6

#cost function --> return the distance between 2 Nodes
def cost(i,j):
    "Returns the cost or distance between two nodes i and j"
    #for now, we will have a simple function that returns the diffence between the nodes
    #distance = sqrt((x2 - x1)^2 + (y2 - y1)^2)
    return math.sqrt((Nodes[j][0] - Nodes[i][0])**2 + (Nodes[j][1] - Nodes[i][1])**2)

#Define the graph - This is a 3D graph [((node i, node j),agent k) ...]
Points = [i for i in range(n)]
graph = [i for i in permutation(Points, 2)]
list_of_agents = [i for i in range(max_agents)]
graph = [(i,j) for i in graph for j in list_of_agents ]
#print graph

#binary variable (xij) for ILP  - will take on the value of 1 if path ij is taken by vehicle k and 0 otherwise
use_edge = LpVariable.dicts("UseEdge", graph, lowBound = 0,  upBound = 1, cat = pulp.LpInteger)

# Setup the instance of the model for minimization problem
mtsp_prob = LpProblem("MTSP-Model", LpMinimize)

#objective
#minimize cost
mtsp_prob += lpSum([cost(edge[0][0], edge[0][1]) * use_edge[edge] for edge in graph])

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
    mtsp_prob += lpSum([use_edge[((i,j), k)] for i in range(n) if i != j for k in range(max_agents)]) == 1

#Ensure that only one tour leaves each nodes
for i in range(1,n):
    mtsp_prob += lpSum([use_edge[((i,j), k)] for j in range(n) if i != j for k in range(max_agents)]) == 1

#Ensure that each agent visits at most "max_nodes" nodes
for k in range(max_agents):
    mtsp_prob += lpSum(use_edge[((i,j), k)] for i in range(n) for j in range(n) if i != j) <= max_nodes

#Ensure that same vehicle arrives and departs from each node it serves
sum_leaving = LpVariable("Sum Leaving Node", 0, None, LpInteger)
sum_entering = LpVariable("Sum Entering Node", 0, None, LpInteger)
for k in range(max_agents):
    for node in range(n):
        sum_entering = lpSum([use_edge[((i,node), k)] for i in range(n) if i != node ])
        sum_leaving = lpSum([use_edge[((node,j), k)] for j in range(n) if node != j ])
        mtsp_prob +=  sum_entering - sum_leaving == 0

#subtour elimination
#find u[i] - u[j] that satifies the constraints. Model U as a LP variable
u = LpVariable.dicts("U - Subtour Elimination", Points, 0, None, LpInteger)
for k in range(max_agents):
    for i in range(n):
        for j in range(1, n): #constraint not defined for j = 0
            if (i!=j):
                mtsp_prob += u[i] - u[j] + n*use_edge[((i,j),k)] <= n-1

print "Solving ...\n"

begin_time = time.time()
status = mtsp_prob.solve()


print LpStatus[status]

#Print result - We are only concerned with Integer values of 1
print "The result of the Integer Linear Program is: \n"
for item in graph:
    #print value(use_edge[item]),
    if value(use_edge[item]) == 1:
        print item#, value(use_edge[item])


print "Time taken:{0: .2f} seconds".format(time.time() - begin_time)
#print "Number of agent", max_agents
#print value(sum_leaving.varValue)
