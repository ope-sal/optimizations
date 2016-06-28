from pulp import *

REQUIRE = {
1 : 7,
2 : 5,
3 : 3,
4 : 2,
5 : 2
}
PRODUCTS = [1, 2, 3, 4, 5]
LOCATIONS = [1, 2, 3, 4, 5]
CAPACITY = 8


prob = LpProblem("facility.txt", LpMinimize)
use_vars = LpVariable.dicts("UseLocation", LOCATIONS, 0, 1, LpBinary)
waste_vars = LpVariable.dicts("Waste", LOCATIONS, 0, CAPACITY)
assign_vars = LpVariable.dicts("AtLocation",
    [(i, j) for i in LOCATIONS
        for j in PRODUCTS],
    0, 1, LpBinary)
prob += lpSum(waste_vars[i] for i in LOCATIONS)
for j in PRODUCTS:
    prob += lpSum(assign_vars[(i, j)] for i in LOCATIONS) == 1
for i in LOCATIONS:
    prob += lpSum(assign_vars[(i, j)] * REQUIRE[j] for j in PRODUCTS) + waste_vars[i] == CAPACITY * use_vars[i]

prob.solve()
TOL = 0.00001
for i in LOCATIONS:
    if use_vars[i].varValue > TOL:
        print "Location ", i, " produces ", [j for j in PRODUCTS if assign_vars[(i, j)].varValue > TOL]
