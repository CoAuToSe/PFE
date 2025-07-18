from mathematical_model import build_model, solve_model

pos_initial, pos_end= [0, 0], [3, 0]

e_profiling = [0.0287, 0.118, 0.0482, 1.44, 1.23, 0.375, 0.375]
E_max = 100 - e_profiling[2]

t_scan = 2
t_charge = 1.5*60*60
t_takeoff_landing = 9.248

drone_life, drone_cost = 40*60*60, 115
battery_life, battery_cost = 250, 30

image_path = "D:\Python程序\无人机任务优化\\target.png"
wall_width, wall_height= 7, 12.19
camera_parameter = [1, 0.75]
wall_distance = 2
drone_speed = 0.4
K = 30
c_time, c_charge = drone_cost / drone_life, battery_cost / battery_life
M, N, F, S, C_stations, e_movement, t_movement = build_model(wall_width, wall_height, camera_parameter, wall_distance, drone_speed, e_profiling, image_path)
solve_model(K, M, N, F, S, C_stations, e_movement, t_movement, pos_initial, pos_end, E_max, e_profiling, t_scan, t_charge, t_takeoff_landing, c_time, c_charge)
