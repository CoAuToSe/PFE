import math
import random
import numpy as np
from deap import base, creator, tools, algorithms
import matplotlib.pyplot as plt

preset_take_off_pos=np.array([0, 0])
preset_landing_pos=np.array([3, 0.5])
preset_speed=0.4
preset_wall_distance=1
preset_photo_time=3.0
preset_station_positions=np.array([[0, 0], [6, 0]])
battery_cap=100
energy_rates=np.array([0.2, 0.16, 0.18, 0.12, 0.024, 0.024])
camera_params=np.array([0.6, 0.4])
land_height=0.5
takeoff_height=0.5
land_time=3.2
takeoff_time=6.0
charge_time=13 * 60.0
lifespan=1000.0 * 60 * 60
drone_cost=199.0
battery_lifespan=500.0
battery_cost=35.69
wall_width = 3
wall_height = 2



class Drone:
    def __init__(self, preset_take_off_pos, preset_landing_pos, preset_speed, preset_wall_distance, preset_photo_time, preset_station_positions, battery_cap, energy_rates, camera_params, land_height, takeoff_height, 
                 land_time, takeoff_time, charge_time, lifespan, drone_cost, battery_lifespan, battery_cost):
        
        # 任务设定参数
        self.preset_take_off_pos = preset_take_off_pos
        self.preset_landing_pos = preset_landing_pos
        self.preset_speed = preset_speed
        self.preset_wall_distance = preset_wall_distance
        self.preset_photo_time = preset_photo_time
        self.preset_station_positions = preset_station_positions

        # 二级参数
        self.energy_rates = energy_rates

        # 无人机固定参数
        self.camera_params = camera_params
        self.battery_cap = battery_cap
        self.land_height = land_height
        self.takeoff_height = takeoff_height
        self.land_time = land_time
        self.takeoff_time = takeoff_time
        self.charge_time = charge_time
        self.lifespan = lifespan
        self.drone_cost = drone_cost
        self.battery_lifespan = battery_lifespan
        self.battery_cost = battery_cost

        # 过程参数
        self.reset()

    def reset(self):
        self.battery_level = self.battery_cap
        self.position = self.preset_take_off_pos.copy()
        self.energy_used = 0
        self.total_work_time = 0
        self.charge_count = 0

    def takeoff(self, station_pos):
        self.energy_used += self.energy_rates[4]
        self.total_work_time += self.takeoff_time
        self.battery_level -= self.energy_rates[4]
        self.position = self.get_takeoff_pos(station_pos)

    def land(self, station_pos):
        self.energy_used += self.energy_rates[5]
        self.total_work_time += self.land_time
        self.battery_level -= self.energy_rates[5]
        self.position = station_pos

    def charge(self):
        self.charge_count += 1
        self.total_work_time += self.charge_time * (1 - self.battery_level / self.battery_cap)
        self.battery_level = self.battery_cap

    def fly(self, current_pos, target_pos):
        distance = np.linalg.norm(target_pos - current_pos)
        energy_needed = self.get_energy_needed(current_pos, target_pos)
        self.total_work_time += distance / self.preset_speed
        self.energy_used += energy_needed
        self.battery_level -= energy_needed
        self.position = np.array(target_pos)

    def take_photo(self):
        self.total_work_time += self.preset_photo_time
        energy_needed = self.preset_photo_time * self.energy_rates[3]
        self.energy_used += energy_needed
        self.battery_level -= energy_needed

    def get_landing_pos(self, station_pos):
        return np.array([station_pos[0], station_pos[1] + self.land_height])
        
    def get_takeoff_pos(self, station_pos):
        return np.array([station_pos[0], station_pos[1] + self.takeoff_height])
    
    def get_energy_needed(self, current_pos, target_pos):
        vertical_distance = target_pos[1] - current_pos[1]
        horizontal_distance = np.abs(target_pos[0] - current_pos[0])
        if vertical_distance > 0:
            return vertical_distance * self.energy_rates[0] + horizontal_distance * self.energy_rates[2]
        else:
            return -vertical_distance * self.energy_rates[1] + horizontal_distance * self.energy_rates[2]
        
    def get_closest_station(self, current_pos):
        current_pos = np.array(current_pos)
        closest_station = self.preset_station_positions[0]
        min_distance = np.linalg.norm(current_pos - self.get_landing_pos(closest_station))
        for station_pos in self.preset_station_positions:
            test_landing_pos = self.get_landing_pos(station_pos)
            distance = np.linalg.norm(current_pos - test_landing_pos)
            if distance < min_distance:
                closest_station = station_pos
                min_distance = distance
        return closest_station

class Wall:
    def __init__(self, width, height):
        self.width = width
        self.height = height

def evaluate(path):
    # 创建新的无人机对象
    drone = Drone(
        preset_take_off_pos=preset_take_off_pos,
        preset_landing_pos=preset_landing_pos,
        preset_speed=preset_speed,
        preset_wall_distance=preset_wall_distance,
        preset_photo_time=preset_photo_time,
        preset_station_positions=preset_station_positions,
        battery_cap=battery_cap,
        energy_rates=energy_rates,
        camera_params=camera_params,
        land_height=land_height,
        takeoff_height=takeoff_height,
        land_time=land_time,
        takeoff_time=takeoff_time,
        charge_time=charge_time,
        lifespan=lifespan,
        drone_cost=drone_cost,
        battery_lifespan=battery_lifespan,
        battery_cost=battery_cost
    )

    drone.reset()  # 重置无人机状态
    infeasible = False  # 标记解是否可行

    total_time = 0
    total_energy = 0
    total_cost = 0

    drone.takeoff(drone.preset_take_off_pos)
    current_closest_station = drone.get_closest_station(drone.position)
    path = path.copy()
    path.append(drone.preset_landing_pos)

    photo_energy = drone.preset_photo_time * drone.energy_rates[3]

    for waypoint in path:
        # 将航点转换为 NumPy 数组
        waypoint_np = np.array(waypoint)

        to_charge_energy_needed = drone.get_energy_needed(drone.position, drone.get_landing_pos(current_closest_station)) + drone.energy_rates[5]
        if drone.battery_level >= photo_energy + to_charge_energy_needed:
            drone.take_photo()
        else:
            return_position = drone.position
            drone.fly(drone.position, drone.get_landing_pos(current_closest_station))
            drone.land(current_closest_station)
            drone.charge()
            drone.takeoff(drone.position)
            drone.fly(drone.position, return_position)
            drone.take_photo()
        
        next_closest_station = drone.get_closest_station(waypoint_np)
        move_energy_needed = drone.get_energy_needed(drone.position, waypoint_np)
        to_charge_energy_needed = drone.get_energy_needed(waypoint_np, drone.get_landing_pos(next_closest_station)) + drone.energy_rates[5]

        if drone.battery_level >= move_energy_needed + to_charge_energy_needed:
            drone.fly(drone.position, waypoint_np)
        else:
            drone.fly(drone.position, drone.get_landing_pos(current_closest_station))
            drone.land(current_closest_station)
            drone.charge()
            drone.takeoff(drone.position)
            drone.fly(drone.position, waypoint_np)

        current_closest_station = next_closest_station


    drone.land(drone.position)

    total_time = drone.total_work_time
    total_energy = drone.energy_used
    total_cost = total_time / drone.lifespan * drone.drone_cost + drone.charge_count / drone.battery_lifespan * drone.battery_cost

    return total_time, total_energy, total_cost

def generate_initial_path(wall, region_size):
    x_total = math.ceil(wall.width / region_size[0])
    y_total = math.ceil(wall.height / region_size[1])
    waypoints = []
    
    for y in range(y_total):
        for x in range(x_total):
            waypoints.append(((x + 0.5) * region_size[0], (y + 0.5) * region_size[1]))  # 使用元组表示航点
    
    # 确保初始路径是一个有效的排列
    return random.sample(waypoints, len(waypoints))

def cx_ordered(ind1, ind2):
    size = len(ind1)
    a, b = sorted(random.sample(range(size), 2))
    
    # 保存父代的片段
    temp1 = ind1[a:b]
    temp2 = ind2[a:b]
    
    # 创建空的子代
    child1 = [None]*size
    child2 = [None]*size
    
    # 将片段复制到子代
    child1[a:b] = temp1
    child2[a:b] = temp2
    
    # 定义填充函数
    def fill_child(child, parent, start, end):
        idx = end % size
        for i in range(size):
            gene = parent[(end + i) % size]
            if gene not in child:
                child[idx % size] = gene
                idx += 1
        return child
    
    # 填充子代的剩余部分
    child1 = fill_child(child1, ind2, a, b)
    child2 = fill_child(child2, ind1, a, b)
    
    # 将子代的基因赋值回原始个体，保持类型不变
    ind1[:] = child1
    ind2[:] = child2
    
    return ind1, ind2

def mutate_swap(individual):
    size = len(individual)
    a, b = random.sample(range(size), 2)
    individual[a], individual[b] = individual[b], individual[a]
    return individual,

# 可视化帕累托前沿解
def plot_pareto_front(pareto_front):
    # 提取每个个体的适应度值
    total_time = [ind.fitness.values[0] for ind in pareto_front]
    total_energy = [ind.fitness.values[1] for ind in pareto_front]
    total_cost = [ind.fitness.values[2] for ind in pareto_front]

    # 创建 3D 图
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制散点图
    scatter = ax.scatter(total_time, total_energy, total_cost, c='r', marker='o')
    ax.set_xlabel('Total Time')
    ax.set_ylabel('Total Energy')
    ax.set_zlabel('Total Cost')
    ax.set_title('Pareto Front Visualization')

    # 显示图形
    plt.show()

# 新增函数：可视化最优路径
def plot_best_path(best_individual, wall):
    # 提取路径
    path = best_individual

    # 提取 x 和 y 坐标，并进行偏移
    x_coords = [point[0] for point in path]
    y_coords = [point[1] for point in path]

    # 在路径的开头添加起点坐标（无人机的起飞位置）
    x_coords = [preset_take_off_pos[0]] + [preset_take_off_pos[0]] + x_coords + [preset_landing_pos[0]] + [preset_landing_pos[0]]
    y_coords = [preset_take_off_pos[1]] + [preset_take_off_pos[1] + takeoff_height] + y_coords + [preset_landing_pos[1]] + [preset_landing_pos[1] - land_height]

    # 绘制路径
    plt.figure(figsize=(8, 6))
    plt.plot(x_coords, y_coords, marker='o', linestyle='-', color='b')

    # 添加箭头，确保每段路径末端都有箭头
    for i in range(len(x_coords) - 1):
        plt.arrow(x_coords[i], y_coords[i],
                  x_coords[i + 1] - x_coords[i],
                  y_coords[i + 1] - y_coords[i],
                  head_width=0.1, head_length=0.15, fc='r', ec='r', length_includes_head=True)

    plt.title('Best Path Visualization')
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.xlim(0, wall.width)
    plt.ylim(0, wall.height)
    plt.grid(True)
    plt.show()

wall = Wall(wall_width, wall_height)
# Registering functions in toolbox
creator.create("FitnessMulti", base.Fitness, weights=(-1.0, -1.0, -1.0))
creator.create("Individual", list, fitness=creator.FitnessMulti)

toolbox = base.Toolbox()
toolbox.register("individual", tools.initIterate, creator.Individual, 
                 lambda: generate_initial_path(wall, (camera_params[0] * preset_wall_distance, camera_params[1] * preset_wall_distance)))  # 调整墙壁尺寸和区域大小

toolbox.register("population", tools.initRepeat, list, toolbox.individual)

toolbox.register("evaluate", evaluate)
toolbox.register("mate", cx_ordered)
toolbox.register("mutate", mutate_swap)
toolbox.register("select", tools.selNSGA2)

# 在主函数中调用可视化函数
def main():
    # 初始化种群
    population = toolbox.population(n=50)
    
    # 定义帕累托前沿，用于存储所有非支配解
    pareto_front = tools.ParetoFront()
    
    # 定义精英保留策略的记录器（存放最优解）
    halloffame = tools.HallOfFame(1)
    
    # 定义统计信息收集器
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean, axis=0)
    stats.register("std", np.std, axis=0)
    stats.register("min", np.min, axis=0)
    stats.register("max", np.max, axis=0)

    # 执行多目标优化
    population, logbook = algorithms.eaMuPlusLambda(
        population, toolbox, mu=50, lambda_=100, cxpb=0.7, mutpb=0.2, ngen=40,
        stats=stats, halloffame=halloffame, verbose=True
    )

    # 更新帕累托前沿解
    pareto_front.update(population)

    # 打印帕累托前沿上的解
    print("Pareto Front Solutions:")
    for ind in pareto_front:
        print(f"Path: {ind} -> Fitness: {ind.fitness.values}")

    # 可视化帕累托前沿
    plot_pareto_front(pareto_front)

    # 可视化最优路径（使用 Hall of Fame 中的最佳个体）
    #print("Best Individual:")
    #print(f"Path: {halloffame[0]} -> Fitness: {halloffame[0].fitness.values}")
    plot_best_path(halloffame[0], wall)

    return population, stats, halloffame, pareto_front

if __name__ == "__main__":
    main()