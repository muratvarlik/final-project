import numpy as np
import pandas as pd
import gurobipy as gp
from gurobipy import GRB


# Parameters
df = pd.read_csv("18.01/18.01.csv")
timelimit = 3600


distance= pd.read_csv("18.01/distance_matrix_18.01.csv", header=None)
distance= np.array(distance)

#opening time
opening_t= np.array(df[["Açılış"]])
#closing time
closing_t=np.array(df[["Kapanış"]])

M = 99999 # Sufficiently big number

# Demand
demand = np.array(df[["Demand"]])

delay = 0.5


vehicles = np.array([
    [0,19,0.000000036717348927875200000], #Vehicle: id, capacity, co2 emitted per meter per kg (in kg)
    [1,19,0.000000036717348927875200000],     
    [2,18,0.000000038757201646090500000],
    [3,18,0.000000038757201646090500000],
    [4,18,0.000000038757201646090500000],
    [5,18,0.000000038757201646090500000],
    [6,15,0.000000046508641975308600000],
    [7,15,0.000000046508641975308600000],
    [8,15,0.000000046508641975308600000],
    [9,10,0.000000069762962962963000000],
    [10,10,0.000000069762962962963000000],
    [11,9,0.000000077514403292181100000],
    [12,8,0.000000087203703703703700000]
    ])

# Create a model
model = gp.Model("TSP")

# Decision Variables
x = model.addVars(distance.shape[0], distance.shape[1], vehicles.shape[0],vtype=GRB.BINARY, name="x") 
#subtour elimination variable
u = model.addVars(distance.shape[0],vehicles.shape[0],vtype=GRB.CONTINUOUS, name="u")
# supply carried to node i by  vehicle k 
s = model.addVars(distance.shape[0],vehicles.shape[0],vtype=GRB.CONTINUOUS, name="a")
# Time variable for service start for vehicle k
t = model.addVars(distance.shape[0],vehicles.shape[0] ,vtype=GRB.CONTINUOUS, name="t")  # Time variable for service start
# If node i is visited by vehicle k
y = model.addVars(distance.shape[0],vehicles.shape[0], vtype =GRB.BINARY, name="y")
# Supply carried on each arcs by vehicle k
w = model.addVars(distance.shape[0], vehicles.shape[0], vtype= GRB.CONTINUOUS, name="w")

model.update()
# Objective function: minimize total distance
model.setObjective(gp.quicksum(distance[i][j] * w[j,k] * vehicles[k,2]
                               for i in range(distance.shape[0]) for j in range(distance.shape[0]) 
                               for k in range(vehicles.shape[0]) if i != j)
                   ,GRB.MINIMIZE)

# Constraints
# Each node is visited
for i in range(distance.shape[0]):
    for k in range(vehicles.shape[0]):
        model.addConstr(gp.quicksum(x[i, j, k] for j in range(distance.shape[1]) if i != j) == y[i,k] ) #to leave one way
    
# Constraints: each city is left exactly once by each vehicle
for j in range(distance.shape[0]):
    for k in range(vehicles.shape[0]):
        model.addConstr(gp.quicksum(x[i, j, k] for i in range(distance.shape[0]) if i != j) == y[j, k] ) #to arrive one way

# Sub-tour elimination
for i in range(1, distance.shape[0]):  # Exclude the depot (city 0)
    for j in range(1, distance.shape[0]): # Exclude the depot (city 0)
        for k in range(vehicles.shape[0]): 
            if i != j:
                model.addConstr(u[i,k] - u[j,k] + (distance.shape[0] - 1) * x[i, j,k] <= distance.shape[0] - 2)

# Time window constraints3
for i in range(1, distance.shape[0]):
    for k in range(vehicles.shape[0]): 
        model.addConstr(t[i,k] >= opening_t[i]  )  # Opening time constraint
        model.addConstr(t[i,k] + delay <= closing_t[i]  )  # Closing time constraint

      
# Time windows constraints distance traveled i -> j
for i in range(1, distance.shape[0]):
    for j in range(distance.shape[0]):
        for k in range(vehicles.shape[0]): 
            if i != j: 
                model.addConstr(t[j,k] >= t[i,k] + delay + distance[i][j]/50000 - M * (1 - x[i, j,k]))

#Supply carried should be <= capacity of vehicle
for k in range(vehicles.shape[0]): 
    model.addConstr(gp.quicksum(s[i,k] for i in range(distance.shape[0])) <= vehicles[k,1])

#Supply carried should be >= demand
for i in range(1, distance.shape[0]):    
    model.addConstr(gp.quicksum(s[i,k] for k in range(vehicles.shape[0]))  >= demand[i])

#vehicle priority
model.addConstr(y[0,0]>= y[0,1])
model.addConstr(y[0,2]>= y[0,3])
model.addConstr(y[0,3]>= y[0,4])
model.addConstr(y[0,4]>= y[0,5])
model.addConstr(y[0,6]>= y[0,7])
model.addConstr(y[0,7]>= y[0,8])
model.addConstr(y[0,9]>= y[0,10])

#if supply is 0 to node the node should not be visited
for i in range(1,distance.shape[0]):
    for k in range(vehicles.shape[0]): 
        model.addConstr(s[i,k] <= M * y[i,k])

# weight calculation at other nodes
for i in range(1, distance.shape[0]):
    for j in range(distance.shape[0]):
        for k in range(vehicles.shape[0]): 
            if i != j:
                model.addConstr(w[j,k] <= w[i,k] - 135 *  s[i,k] + M* (1-x[i,j,k]))

# empty vehicle weight calculations
for i in range(1, distance.shape[0]):
    for k in range(vehicles.shape[0]): 
            model.addConstr(3600*x[i,0,k] <= w[0,k])

# if the total present vehicle weight is 0 at node i the node should not be visited
for i in range(distance.shape[0]):
    for k in range(vehicles.shape[0]):
        model.addConstr( y[i,k]*3600<= w[i,k])

# Solve and set parameters
model.setParam('TimeLimit', timelimit)  
model.optimize()



# Print solution
# Print the optimal tours for each vehicle
if model.status == GRB.TIME_LIMIT:
    print("Time limit reached")
    tour = []
    
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].X > 0.5:
                    tour.append((i, j, k))
    tour.sort(key=lambda x: x[2])
    for i, j,k in tour:
        if(i != j):
            print(f"City {i} -> City {j}: Distance {distance[i][j]} by Vehicle:{vehicles[k,0]}")
    print()
    
    for k in range(vehicles.shape[0]):
        for i in range(distance.shape[0]):
            for j in range(distance.shape[1]):
                if x[i,j,k].x > 0.5:
                    if w[j,k].X >= 0:
                        print(f"While driving to node {i} from to node {j}, the vehicle {vehicles[k,0]} weighted {w[j,k].X}.") 
    print()
    for k in range(vehicles.shape[0]): 
        for i in range(distance.shape[0]):
            if s[i,k].x > 0:
                print(f"Supply delivered to node: {i} is {s[i,k].X} by vehicle {vehicles[k,0]}")
    print()    

    sum =0 
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].X > 0.5:
                    sum += distance[i, j]
    print(f"Total distance traveled is {sum} meters")
    print()
    c = 0
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].X > 0.5:
                    c += distance[i][j] * w[j,k].X * vehicles[k,2]
    print(f"Total CO2 emitted is {c} in kgs")
           

else:
    print("Optimization completed within the time limit")

    tour = []
    
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].X > 0.5:
                    tour.append((i, j, k))
    tour.sort(key=lambda x: x[2])
    for i, j,k in tour:
        if(i != j):
            print(f"City {i} -> City {j}: Distance {distance[i][j]} by Vehicle:{vehicles[k,0]}")
    print()
    
    for k in range(vehicles.shape[0]):
        for i in range(distance.shape[0]):
            for j in range(distance.shape[1]):
                if x[i,j,k].x > 0.5:
                    if w[j,k].X >= 0:
                        print(f"While driving to node {i} from to node {j}, the vehicle {vehicles[k,0]} weighted {w[j,k].X}.") 
    print()
    for k in range(vehicles.shape[0]): 
        for i in range(distance.shape[0]):
            if s[i,k].x > 0:
                print(f"Supply delivered to node: {i} is {s[i,k].X} by vehicle {vehicles[k,0]}")
    print()    

    sum =0 
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].X > 0.5:
                    sum += distance[i, j]
    print(f"Total distance traveled is {sum} meters")
    print()
    c = 0
    for i in range(distance.shape[0]):
        for j in range(distance.shape[1]):
            for k in range(vehicles.shape[0]): 
                if x[i, j,k].X > 0.5:
                    c += distance[i][j] * w[j,k].X * vehicles[k,2]
    print(f"Total CO2 emitted is {c} in kgs")
