# fame_simulation
The `fame_simulation` package initializes a Gazebo simulation with two ground vehicles and an obstacle.

## Description
The proposed application scenario consists of two cooperative ground vehicles with different capabilities, but with a common mission, i.e. identify a specific target and destroy it. REX (Robot EXplorer) is in charge of performing the exploration of the area and identifying the target that should be destroyed. Whereas, DINGO (DestroyING rObot) starts its execution when it receives the coordinates of the target, so that it is able to reach and destroy it. The destruction of the target determines the ending of the system execution.

## System launch

**Package initialization**:
```bash
cd fame_simulation
colcon build
source install/setup.bash
```

**Node execution**:

```bash
ros2 launch fame_simulation multi_launch.py
```
