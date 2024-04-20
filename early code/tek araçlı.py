import numpy as np
import gurobipy as gp
from gurobipy import GRB

# Parameters
CAPACITY = 6  # Vehicle capacity
M = 99999 # Sufficiently big number

nodes = np.array([
    [0, 0, 0, 0.0, 24.0, 0],  # Node 0 (depot): x, y, demand, opening time, closing time
    [1, 3, 1, 5.0, 8.0, 0.5],  # Node 1: x, y, demand, opening time, closing time
    [2, 1, 2, 0.0, 4.0, 0.25],  # Node 2: x, y, demand, opening time, closing time
    [3, 5, 3, 10.0, 13.0, 0.5]   # Node 3: x, y, demand, opening time, closing time
])

#calculate distances
distances = np.linalg.norm(nodes[:, None, :2] - nodes[:, :2], axis=-1)

# Create a model
model = gp.Model("TSP")

# Variables
x = model.addVars(distances.shape[0], distances.shape[1],vtype=GRB.BINARY, name="x")
#subtour elimination variable
u = model.addVars(distances.shape[0],vtype=GRB.CONTINUOUS, name="u")
#supply carried to node i by single vehicle
s = model.addVars(distances.shape[0],vtype=GRB.CONTINUOUS, name="a")
# Time variable for service start
t = model.addVars(distances.shape[0], vtype=GRB.CONTINUOUS, name="t")
# If node i is visited
y = model.addVars(distances.shape[0], vtype =GRB.BINARY, name="y")
# Supply carried on each arcs
w = model.addVars(distances.shape[0], vtype= GRB.CONTINUOUS, name="z")


model.update()
# Objective function: minimize total distance
model.setObjective(gp.quicksum(distances[i][j] * x[i, j] for i in range(distances.shape[0]) for j in range(distances.shape[0]) if i != j)
                   ,GRB.MINIMIZE)

# Constraints
# Each node is visited exactly once
for i in range(distances.shape[0]):
    model.addConstr(gp.quicksum(x[i, j] for j in range(distances.shape[1]) if i != j) == y[i] )
    
# Constraints: each city is left exactly once by each vehicle
for j in range(distances.shape[1]):
    model.addConstr(gp.quicksum(x[i, j] for i in range(distances.shape[0]) if i != j) == y[j] )

# Sub-tour elimination
for i in range(1, distances.shape[0]):  # Exclude the depot (city 0)
    for j in range(1, distances.shape[1]):  # Exclude the depot (city 0)
        if i != j:
            model.addConstr(u[i] - u[j] + (distances.shape[0] - 1) * x[i, j] <= distances.shape[0] - 2)

# Time window constraints
for i in range(distances.shape[0]):
    model.addConstr(t[i] >= nodes[i, 3])  # Opening time constraint
    model.addConstr(t[i] <= nodes[i, 4])  # Closing time constraint
    
# Time windows constraints distance traveled
for i in range(1, distances.shape[0]):
    for j in range(1, distances.shape[1]):
        if i != j:
            model.addConstr(t[j] >= t[i] + distances[i][j] + nodes[i][5] - M * (1 - x[i, j])) 

#Supply carried should be <= capacity of vehicle
model.addConstr(gp.quicksum(s[i] for i in range(distances.shape[0])) <= CAPACITY)

#Supply carried should be >= demand
for i in range(1, distances.shape[0]):    
    model.addConstr( s[i] >= nodes[i][2])
    
#if supply is 0 to node the node should not be visited 
for i in range(1, distances.shape[0]):
    model.addConstr(s[i] <= M * y[i])

# weight calculation at other nodes
for i in range(1, distances.shape[0]):
    for j in range(distances.shape[1]):
        if i != j:
                model.addConstr(w[j] <= w[i] - s[i] + M* (1-x[i,j]))

# Solve
model.optimize()

# Print solution
# Print the optimal tours for each vehicle
if model.status == GRB.OPTIMAL:
    tour = []
    for i in range(distances.shape[0]):
        for j in range(distances.shape[1]):
            if x[i, j].x > 0.5:
                tour.append((i, j))
    tour.sort(key=lambda x: x[0])
    for i, j in tour:
        if(i != j):
            print(f"City {i} -> City {j}: Distance {distances[i, j]}")
    print()
    
    for i in w.keys():
        for j in w.keys():
            if x[i,j].X >0:
                if w[j].X >= 0:
                    print(f"While driving to node {i} from to node {j}, the vehicle weighted {w[j].X}.")
    
    print()

    for i in s.keys():
        if s[i].X >= 0:
            print(f"Supply delivered to node: {i} is {s[i].X}")
else:
    print("No solution found.")

