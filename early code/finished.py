import numpy as np
import gurobipy as gp
from gurobipy import GRB

# Parameters
M = 99999 # Sufficiently big number

nodes = np.array([
    [0, 0, 0, 0.0, 24.0, 0],  # Node 0 (depot): x, y, demand, opening time, closing time,delay
    [1, 3, 18, 5.0, 8.0, 0.5],  # Node 1: x, y, demand, opening time, closing time
    [2, 1, 9, 0.0, 15, 0.25],  # Node 2: x, y, demand, opening time, closing time
    [3, 5, 21, 10.0, 13.0, 0.5]   # Node 3: x, y, demand, opening time, closing time
])


vehicles = np.array([
    [111,16], #Vehicle 1: id, capacity
    [222,16], #Vehicle 2: id, capacity
    [333,16]  #Vehicle 2: id, capacity
    ])


#calculate distances
distances = np.linalg.norm(nodes[:, None, :2] - nodes[:, :2], axis=-1)

# Create a model
model = gp.Model("TSP")

# Variables
x = model.addVars(distances.shape[0], distances.shape[1], vehicles.shape[0],vtype=GRB.BINARY, name="x")
#subtour elimination variable
u = model.addVars(distances.shape[0],vehicles.shape[0],vtype=GRB.CONTINUOUS, name="u")
#supply carried to node i by  vehicle k 
s = model.addVars(distances.shape[0],vehicles.shape[0],vtype=GRB.CONTINUOUS, name="a")
# Time variable for service start for vehicle k
t = model.addVars(distances.shape[0],vehicles.shape[0] ,vtype=GRB.CONTINUOUS, name="t")  # Time variable for service start
# If node i is visited by vehicle k
y = model.addVars(distances.shape[0],vehicles.shape[0], vtype =GRB.BINARY, name="y")
# Supply carried on each arcs by vehicle k
w = model.addVars(distances.shape[0], vehicles.shape[0], vtype= GRB.CONTINUOUS, name="w")

model.update()
# Objective function: minimize total distance
model.setObjective(gp.quicksum(distances[i][j] * x[i, j, k] for i in range(distances.shape[0]) for j in range(distances.shape[0]) for k in range(vehicles.shape[0]) if i != j)
                   ,GRB.MINIMIZE)

# Constraints
# Each node is visited exactly once
for i in range(distances.shape[0]):
    for k in range(vehicles.shape[0]):
        model.addConstr(gp.quicksum(x[i, j, k] for j in range(distances.shape[1]) if i != j) == y[i,k] )
    
# Constraints: each city is left exactly once by each vehicle
for j in range(distances.shape[1]):
    for k in range(vehicles.shape[0]):
        model.addConstr(gp.quicksum(x[i, j, k] for i in range(distances.shape[0]) if i != j) == y[j, k] )

# Sub-tour elimination
for i in range(1, distances.shape[0]):  # Exclude the depot (city 0)
    for j in range(1, distances.shape[1]): # Exclude the depot (city 0)
        for k in range(vehicles.shape[0]): 
            if i != j:
                model.addConstr(u[i,k] - u[j,k] + (distances.shape[0] - 1) * x[i, j,k] <= distances.shape[0] - 2)

# Time window constraints
for i in range(distances.shape[0]):
    for k in range(vehicles.shape[0]): 
        model.addConstr(t[i,k] >= nodes[i, 3]  )  # Opening time constraint
        model.addConstr(t[i,k] +  nodes[i][5] <= nodes[i, 4]  )  # Closing time constraint

        
# Time windows constraints distance traveled
for i in range(1, distances.shape[0]):
    for j in range(distances.shape[1]):
        for k in range(vehicles.shape[0]): 
            if i != j:
                model.addConstr(t[j,k] >= t[i,k] + distances[i][j] + nodes[i][5] - M * (1 - x[i, j,k])) 


#Supply carried should be <= capacity of vehicle
for k in range(vehicles.shape[0]): 
    model.addConstr(gp.quicksum(s[i,k] for i in range(distances.shape[0])) <= vehicles[k,1])

#Supply carried should be >= demand
for i in range(1, distances.shape[0]):    
    model.addConstr(gp.quicksum(s[i,k] for k in range(vehicles.shape[0]))  >= nodes[i][2])
    
#if supply is 0 to node the node should not be visited 
for i in range(1, distances.shape[0]):
    for k in range(vehicles.shape[0]): 
        model.addConstr(s[i,k] <= M * y[i,k])

# weight calculation at other nodes
for i in range(1, distances.shape[0]):
    for j in range(distances.shape[1]):
        for k in range(vehicles.shape[0]): 
            if i != j:
                    model.addConstr(w[j,k] <= w[i,k] - s[i,k] + M* (1-x[i,j,k]))
                    
# total weight should not exceed vehicle capacity
for i in range(1, distances.shape[0]):
    for k in range(vehicles.shape[0]): 
        model.addConstr(w[i,k] <= vehicles[k,1])


# Solve
model.optimize()

# Print solution
# Print the optimal tours for each vehicle
if model.status == GRB.OPTIMAL:
    tour = []
    for i in range(distances.shape[0]):
        for j in range(distances.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].x > 0.5:
                    tour.append((i, j, k))
    tour.sort(key=lambda x: x[2])
    for i, j,k in tour:
        if(i != j):
            print(f"City {i} -> City {j}: Distance {distances[i, j]} by Vehicle:{vehicles[k,0]}")
    print()
    
    for k in range(vehicles.shape[0]):
        for i in range(distances.shape[0]):
            for j in range(distances.shape[1]):
                if x[i,j,k].x > 0.5:
                    if w[j,k].X >= 0:
                        print(f"While driving to node {i} from to node {j}, the vehicle {vehicles[k,0]} weighted {w[j,k].X}.") 
    print()
    for k in range(vehicles.shape[0]): 
        for i in range(distances.shape[0]):
            if s[i,k].x > 0:
                print(f"Supply delivered to node: {i} is {s[i,k].X} by vehicle {vehicles[k,0]}")
    print()    
    
    for k in range(vehicles.shape[0]):
        for i in range(distances.shape[0]):
            for j in range(distances.shape[0]):
                if x[i,j,k].x > 0.5:
                    if t[i,k].x >=0:
                        print(f"Arrival time of vehicle {vehicles[k,0]} to node {i} is {t[i,k].x}")

else:
    print("No solution found.")

