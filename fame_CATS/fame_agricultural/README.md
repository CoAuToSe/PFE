# fame_agricultural
The `fame_agricultural` package initializes a Gazebo simulation with two ground vehicles and a drone, deployed in an agricultural scenario.

## Description
The proposed application scenario consists of two (or possibly more) tractors and a drone which cooperate in order to  identify  and remove weeds in a farmland, and thus increase the farm productivity. 
Both the drone and the tractors are equipped with a controller, enabling computations and communications, a battery, and a several sensors and actuators. Specifically, the drone has four propellers, a laser sensor for the obstacle avoidance, and a camera for the image recognition. While the tractors have wheels moved by engines, a rotating blade for cutting weeds, and the laser sensor.  
At the system start-up, the drone is the only robot that starts its behavior, it receives the boundaries of the field to inspect, and starts the exploration. During the overflight of the field, the drone use the camera sensor to recognize weeds and, when found, it sends to the tractors the coordinates.
This enacts the tractors which store the weed coordinates and send back to the drone their distance to the weed. 
Therefore the drone can elect the closest tractor and notify it. At this point, the tractor elected by the drone starts moving towards the field, avoiding possible obstacles. Once it reaches the weed it activates the blade, cuts the weed, and stops its process until it receives a new position from the drone.

## System launch

**Package initialization**:
```bash
cd fame_agricultural
colcon build
source install/setup.bash
```

**Node execution**:

```bash
ros2 launch fame_agricultural multi_launch.py
```

