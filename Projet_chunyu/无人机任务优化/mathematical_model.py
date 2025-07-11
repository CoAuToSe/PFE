from gurobipy import Model, GRB, quicksum
import numpy as np

from extraction import select_grid_areas 
from visualization import visualize_solution

# 将实际参数转化为模型参数
def build_model(wall_width, wall_height, camera_parameter, wall_distance, drone_speed, e_profiling, image_path):

    photo_width, photo_height = camera_parameter[0] * wall_distance, camera_parameter[1] * wall_distance
    M, N = (np.ceil(wall_width / photo_width)).astype(int), (np.ceil(wall_height / photo_height)).astype(int)
    C_stations, S, F = select_grid_areas(image_path, M, N)

    # energy and time consumption for each movement
    e_movement = {}
    t_movement = {}
    for (i, j) in F:
        e_movement[(i, j, i, j)] = 0
        t_movement[(i, j, i, j)] = 0
        # Right movement
        if (i + 1, j) in F:
            e_movement[(i, j, i + 1, j)], t_movement[(i, j, i + 1, j)] = photo_width * e_profiling[5], photo_width / drone_speed
        # Left movement
        if (i - 1, j) in F:
            e_movement[(i, j, i - 1, j)], t_movement[(i, j, i - 1, j)] = photo_width * e_profiling[6], photo_width / drone_speed
        # Up movement
        if (i, j + 1) in F:
            e_movement[(i, j, i, j + 1)], t_movement[(i, j, i, j + 1)] = photo_height * e_profiling[3], photo_height / drone_speed
        # Down movement
        if (i, j - 1) in F:
            e_movement[(i, j, i, j - 1)], t_movement[(i, j, i, j - 1)] = photo_height * e_profiling[4], photo_height / drone_speed

    return M, N, F, S, C_stations, e_movement, t_movement
            
def solve_model(K, M, N, F, S, C_stations, e_movement, t_movement, pos_initial, pos_end, E_max, e_profiling, t_scan, t_charge, t_takeoff_landing, c_time, c_charge):

    # Problem Parameters
    i_initial, j_initial = pos_initial[0], pos_initial[1]
    i_end, j_end = pos_end[0], pos_end[1]
    E_scan = t_scan * e_profiling[1]

    # Initialize the MILP model
    model = Model("UAV_Path_Planning_Hierarchical")

    # Define Variables
    p = model.addVars([(i, j, k) for (i, j) in F for k in range(K)], vtype=GRB.BINARY, name="p")
    x = model.addVars([(k, i, j, m, n) for k in range(K-1) for (i, j) in F for (m, n) in F if abs(i - m) + abs(j - n) <= 1], vtype=GRB.BINARY, name="x")
    y = model.addVars([(i, j, k) for (i, j) in S for k in range(K)], vtype=GRB.BINARY, name="y")
    y_charge = model.addVars(K, vtype=GRB.BINARY, name="y_charge")
    e = model.addVars(K, vtype=GRB.CONTINUOUS, name="e")
    u = model.addVars(K, vtype=GRB.CONTINUOUS, name="u")
    w = model.addVars(K, vtype=GRB.CONTINUOUS, name="w")

    # Initial Constraints
    model.addConstr(e[0] == E_max - e_profiling[2], "InitialBattery")
    model.addConstr(p[i_initial, j_initial, 0] == 1, "InitialPosition")
    model.addConstr(p[i_end, j_end, K-1] == 1, "EndPosition")

    # Path Continuity Constraints
    for k in range(K - 1):
        for (i, j) in F:
            model.addConstr(quicksum(x[(k, i, j, m, n)] for (m, n) in F if abs(i - m) + abs(j - n) <= 1) == p[i, j, k], f"PathCont_{k}_{i}_{j}")

    for k in range(K):
        model.addConstr(quicksum(p[i, j, k] for (i, j) in F) <= 1, f"PositionCont_{k}")

    # Movement Relationship Constraints
    for k in range(K - 1):
        for (i, j) in F:
            for (m, n) in F:
                if abs(i - m) + abs(j - n) <= 1:
                    model.addConstr(p[m, n, k + 1] >= x[(k, i, j, m, n)], f"MoveRel_{k}_{i}_{j}_{m}_{n}")

    # Scanning Constraints
    for (i, j) in S:
        model.addConstr(quicksum(y[i, j, k] for k in range(K)) == 1, f"ScanOnce_{i}_{j}")
    
    for k in range(K):
        for (i, j) in S:
            model.addConstr(y[i, j, k] <= p[i, j, k], f"ScanPresence_{k}_{i}_{j}")

    # Battery Dynamics Constraints
    for k in range(K - 1):
        model.addConstr(
            e[k + 1] == e[k] + u[k] - quicksum(e_movement[(i, j, m, n)] * x[(k, i, j, m, n)] for (i, j) in F for (m, n) in F if abs(i - m) + abs(j - n) == 1) - E_scan * quicksum(y[i, j, k] for (i, j) in S),
            f"EnergyDynamics_{k}"
        )

    # Energy Recharge Constraints
    for k in range(K):
        model.addConstr(w[k] <= e[k], f"w_ub1_{k}")
        model.addConstr(w[k] >= e[k] - E_max * (1 - y_charge[k]), f"w_lb1_{k}")
        model.addConstr(w[k] <= y_charge[k] * E_max, f"w_ub2_{k}")
        model.addConstr(w[k] >= 0, f"w_lb2_{k}")
        model.addConstr(u[k] == E_max * y_charge[k] - w[k], f"RechargeAmount_{k}")

    # Charging Station Constraints
    for k in range(K):
        model.addConstr(y_charge[k] <= quicksum(p[i, j, k] for (i, j) in C_stations), f"ChargeAtStation_{k}")

    # Define Objective Functions for Hierarchical Optimization
    total_cost = quicksum(
        c_time * (quicksum(t_movement[(i, j, m, n)] * x[(k, i, j, m, n)] for (i, j) in F for (m, n) in F if abs(i - m) + abs(j - n) == 1) + t_scan * quicksum(y[i, j, k] for (i, j) in S) + u[k] * t_charge / E_max)
        + c_charge * y_charge[k]
        for k in range(K-1)
    )
    total_time = quicksum(
        quicksum(t_movement[(i, j, m, n)] * x[(k, i, j, m, n)] for (i, j) in F for (m, n) in F if abs(i - m) + abs(j - n) == 1) + t_scan * quicksum(y[i, j, k] for (i, j) in S) + u[k] * t_charge / E_max +(y_charge[k] + 1)*t_takeoff_landing
        for k in range(K-1)
    )
    total_energy = e[0] - e[K-1] + quicksum(u[k] for k in range(K-1))

    # Set Hierarchical Objectives with Priorities
    model.setObjectiveN(total_time, index=0, priority=5, name='MinimizeTotalTime')
    model.setObjectiveN(total_energy, index=1, priority=1, name='MinimizeTotalEnergy')
    model.setObjectiveN(total_cost, index=2, priority=10, name='MinimizeTotalCost')

    # Optimize the model
    model.optimize()

    # Check if an optimal solution is found
    if model.status == GRB.OPTIMAL:
        print("Optimal solution found with hierarchical optimization!")
        visualize_solution(p, y, y_charge, M, N, K, S, F, C_stations)
    else:
        print("No feasible solution found.")
