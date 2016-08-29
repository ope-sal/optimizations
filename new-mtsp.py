from pyomo.environ import *
import time
import math
from pyomo.opt import SolverFactory
import pyomo.environ

Nodes = [(0,0),(0,1),(0,2),(0,3),(0,4),(0,5),(0,6),(0,7),(0,8), (0,9)]

def mtsp_solver(Nodes):
    n = len(Nodes)
    print "Number of nodes: ", n
    max_agents = 3

    max_nodes = None

    def cost(i,j):
        "Returns the cost or distance between two nodes i and j"
        #for now, we will have a simple function that returns the diffence between the nodes
        #distance = sqrt((x2 - x1)^2 + (y2 - y1)^2)
        return math.sqrt((Nodes[j][0] - Nodes[i][0])**2 + (Nodes[j][1] - Nodes[i][1])**2)

    # Creation of a Concrete Model
    model = ConcreteModel()


    #Define the graph - This is a 3D graph [((node i, node j),agent k) ...]
    Points = [i for i in range(n)]
    list_of_agents = [i for i in range(max_agents)]
    #graph = [i for i in permutation(Points, 2)]
    graph = [(i,j,k) for i in range(n) for j in range(n) if i<j or j==0 for k in range(max_agents)]

    #model.j = Set(Points)
    #model.k = Set(list_of_agents)
    model.use_edge = Var(graph, within=Binary)

    def waiting_time(node, vehicle):
        """Returns the waiting time at the specified node"""
        #return lpSum([cost(i,j)*use_edge[((i,j), vehicle)] for i in range(1,node) for j in range(1,node) if (i+1)==j ])
        return sum([model.use_edge[(i,j,vehicle)]  for i in range(1,node) for j in range(i+1,node) if (i+1)==j ])

    def objective_rule(model):
        #mtsp_prob += lpSum([cost(edge[0][0],edge[0][1])*waiting_time(edge[0][1],edge[1])  for edge in graph ])
        #return sum(model.use_edge[node] for node in graph)
        return sum(model.use_edge[node]*waiting_time(node[1],node[2]) for node in graph)
    model.objective = Objective(rule=objective_rule, sense=minimize, doc='Define objective function')



    #We are assuming the node zero is the default starting Node
    #Ensure atmost  m agents depart from node 0
    def startingNode_rule(model, k):
        return sum(model.use_edge[0,j,k] for j in range(n) if j != 0 ) == 1
      #use <= 1 if you want "at most m agents to depart from node 0" and == if you
      #want exactly m agents to depart from node 0
    model.starting = Constraint(list_of_agents, rule=startingNode_rule, doc='Ensure atmost m agents depart from node 0')

    #Ensure atmost m agents return to node 0
    def endingNode_rule(model, k):
        return sum(model.use_edge[i,0,k] for i in range(n) if i != 0) == 1
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

    def pyomo_postprocess(options=None, instance=None, results=None):
        for node in graph:
            if model.use_edge[node] == 1:
                print node


    opt = SolverFactory("gurobi")
    results = opt.solve(model)
    #sends results to stdout
    results.write()
    print("\nDisplaying Solution\n" + '-'*60)
    pyomo_postprocess(None, None, results)


if __name__ == '__main__':
    # This emulates what the pyomo command-line tools does
    mtsp_solver(Nodes)
