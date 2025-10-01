# my_FaMe
 changes to FaMe
 
 
 
 
## ReadMe FaMe
The FaMe framework allows the modeling and execution robotic systems via BPMN collaborations.

### Requirements
- Ubuntu 20.04 or later
- [ROS2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Installation.html)
- [Gazebo simulator](https://gazebosim.org/docs/all/getstarted) (not required in real robot execution)
- [Camunda Modeler](https://camunda.com/download/modeler/)

### ROS Packages
- `fame_engine`: integrates the [bpmn-engine](https://github.com/paed01/bpmn-engine) in a ROS node.
- `fame_agricultural`: initializes a Gazebo simulation with two ground vehicles and a drone, deployed in an agricultural scenario.
- `fame_simulation`: initializes a Gazebo simulation with two ground vehicles and an obstacle.
