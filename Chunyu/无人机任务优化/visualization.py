import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


# 提取优化结果并可视化
def visualize_solution(p, y, y_charge, M, N, K, S, F, C_stations):


    # 提取 UAV 的移动路径、充电点、扫描点等信息
    uav_positions = {}  # 记录每个时间步 UAV 的位置
    scan_positions = []  # 记录 UAV 执行扫描操作的格
    charge_positions = []  # 记录 UAV 执行充电操作的格点

    print(K)

    for k in range(K):
        for (i, j) in F:
            if p[i, j, k].X > 0.5:  # 提取 UAV 在时间步 k 的位置
                uav_positions[k] = (i, j)

        for (i, j) in S:
            if y[i, j, k].X > 0.5:  # 提取 UAV 在时间步 k 执行扫描操作的点
                scan_positions.append((i, j))

        if y_charge[k].X > 0.5:  # 提取 UAV 在时间步 k 执行充电操作的点
            charge_positions.append(uav_positions[k])

    # 创建网格地图
    fig, ax = plt.subplots(figsize=(M + 2, N + 2))
    ax.set_xticks(np.arange(0, M, 1))
    ax.set_yticks(np.arange(0, N, 1))
    ax.set_xticklabels(np.arange(0, M, 1))
    ax.set_yticklabels(np.arange(0, N, 1))
    ax.grid(True)

    print(uav_positions)

    # 绘制网格上的扫描点
    for (i, j) in S:
        ax.add_patch(Rectangle((i - 0.5, j - 0.5), 1, 1, fill=True, color="lightgreen", alpha=0.5, label='Scanning Point'))
    # 绘制网格上的充电点
    for (i, j) in C_stations:
        ax.add_patch(Rectangle((i - 0.5, j - 0.5), 1, 1, fill=True, color="lightblue", alpha=0.5, label='Charging Station'))

    # 绘制 UAV 的移动路径
    prev_pos = None
    for k in range(K):
        if k in uav_positions:
            (i, j) = uav_positions[k]
            # 标注 UAV 在时间步 k 的位置
            ax.text(i, j, str(k), color="red", fontsize=12, ha="center", va="center", weight="bold")
            
            # 绘制路径线
            if prev_pos is not None:
                prev_i, prev_j = prev_pos
                plt.plot([prev_i, i], [prev_j, j], 'r--', lw=2, label='Path')
            prev_pos = (i, j)

    # 绘制扫描点和充电点的标识
    for (i, j) in scan_positions:
        ax.text(i, j, 'S', color="green", fontsize=16, ha="center", va="center", weight="bold")
    for (i, j) in charge_positions:
        ax.text(i, j, 'C', color="blue", fontsize=16, ha="center", va="center", weight="bold")

    # 设置图例和标题
    # ax.legend(["UAV Path", "Scanning Point", "Charging Station"], loc="upper right")
    plt.title("UAV Path Visualization")
    plt.xlim(-0.5, M - 0.5)
    plt.ylim(N - 0.5, -0.5)
    # plt.gca().set_aspect('equal', adjustable='box')
    plt.gca().set_aspect(0.75, adjustable='box')
    plt.gca().invert_yaxis()
    plt.show()