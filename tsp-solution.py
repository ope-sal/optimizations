#Solution to the travelling salesman problem
from pulp import *

#Nodes = [0,1,2,3,9,8,6,7,4,5]
Nodes = [0,1,2,3,4,5,6,7,8,9,10]
n = len(Nodes)
def cost(i,j):
    "Returns the cost or distance between two nodes i and j"
    #for now, we will have a simple function that returns the diffence between the nodes
    return abs(i - j)

#Define the graph
#graph = [(i,j) for i in A for j in B if i > j]
graph = [i for i in permutation(Nodes, 2)]
print graph

#binary variable (xij) for ILP  - will take on the value of 1 if path is taken and 0 otherwise
use_edge = LpVariable.dicts("UseEdge", graph, lowBound = 0,  upBound = 1, cat = pulp.LpInteger)

# Setup the instance of the model for minimization problem
mtsp_prob = LpProblem("MTSP-Model", LpMinimize)

#objective
mtsp_prob += lpSum([cost(edge[0], edge[1]) * use_edge[edge] for edge in graph] )

#constraints
#Ensure that only one tour enters each nodes
for j in range(n):
    mtsp_prob += lpSum([use_edge[(i,j)] for i in range(len(Nodes)) if i != j]) == 1

#Ensure that only one tour leaves each nodes
for i in range(n):
    mtsp_prob += lpSum([use_edge[(i,j)] for j in range(len(Nodes)) if i != j]) == 1

#subtour elimination
#find u[i] - u[j] that satifies the constraints. Model U as a LP variable
u = LpVariable.dicts("U", Nodes, 0, None, LpInteger)

for i in range(n):
    for j in range(1, n): #constraint not defined for j = 0
        if (i!=j):
            mtsp_prob += u[i] - u[j] + n*use_edge[(i,j)] <= n-1

status = mtsp_prob.solve()

print LpStatus[status]
for item in graph:
    if value(use_edge[item]) == 1:
        print item, value(use_edge[item])
