using Pkg
Pkg.add("GLPK")

using JuMP
using GLPK

# Example data
const NUM_CUSTOMERS = 10
const DEMANDS = [0, 1, 2, 3, 4, 5, 1, 2, 3, 4, 5]  # Including depot demand (0)
const VEHICLE_CAPACITY = 10
const DEPOT = 1

# Distance matrix
const DISTANCES = [
    0 10 20 30 40 50 60 70 80 90 100;
    10 0 10 20 30 40 50 60 70 80 90;
    20 10 0 10 20 30 40 50 60 70 80;
    30 20 10 0 10 20 30 40 50 60 70;
    40 30 20 10 0 10 20 30 40 50 60;
    50 40 30 20 10 0 10 20 30 40 50;
    60 50 40 30 20 10 0 10 20 30 40;
    70 60 50 40 30 20 10 0 10 20 30;
    80 70 60 50 40 30 20 10 0 10 20;
    90 80 70 60 50 40 30 20 10 0 10;
    100 90 80 70 60 50 40 30 20 10 0
]

# Create model
model = Model(GLPK.Optimizer)

# Variables
@variable(model, x[1:NUM_CUSTOMERS, 1:NUM_CUSTOMERS, 1:NUM_CUSTOMERS], Bin)
@variable(model, u[2:NUM_CUSTOMERS] >= 0)

# Objective: minimize total distance
@objective(model, Min, sum(DISTANCES[i,j] * x[i,j,k] for i in 1:NUM_CUSTOMERS, j in 1:NUM_CUSTOMERS, k in 1:NUM_CUSTOMERS if i != j))

# Constraints
@constraint(model, [j in 2:NUM_CUSTOMERS], sum(x[DEPOT, j, k] for k in 1:NUM_CUSTOMERS) == 1)
@constraint(model, [i in 2:NUM_CUSTOMERS], sum(x[i, DEPOT, k] for k in 1:NUM_CUSTOMERS) == 1)

@constraint(model, [i in 2:NUM_CUSTOMERS], sum(x[i, j, k] for j in 1:NUM_CUSTOMERS, k in 1:NUM_CUSTOMERS if i != j) == 1)
@constraint(model, [j in 2:NUM_CUSTOMERS], sum(x[i, j, k] for i in 1:NUM_CUSTOMERS, k in 1:NUM_CUSTOMERS if i != j) == 1)

@constraint(model, [i in 2:NUM_CUSTOMERS, j in 2:NUM_CUSTOMERS, i != j], u[i] - u[j] + VEHICLE_CAPACITY * sum(x[i,j,k] for k in 1:NUM_CUSTOMERS) <= VEHICLE_CAPACITY - DEMANDS[j])

@constraint(model, sum(u[i] for i in 2:NUM_CUSTOMERS) == sum(DEMANDS[i] for i in 2:NUM_CUSTOMERS))

# Capacity constraints
@constraint(model, [k in 1:NUM_CUSTOMERS], sum(DEMANDS[j] * x[i,j,k] for i in 1:NUM_CUSTOMERS, j in 2:NUM_CUSTOMERS if i != j) <= VEHICLE_CAPACITY)

# Optimize
optimize!(model)

# Print solution
println("Objective value: ", objective_value(model))
for i in 1:NUM_CUSTOMERS, j in 1:NUM_CUSTOMERS, k in 1:NUM_CUSTOMERS
    if value(x[i,j,k]) > 0.5
        println("Vehicle ", k, " travels from ", i, " to ", j)
    end
end
