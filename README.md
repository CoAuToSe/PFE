# PFE
 
## Summarie

1. [Default usage](#default-usage)
1. [Exploiting the tello](#exploiting-the-tello)
1. [Using Husky A300](#using-husky-a300)
1. [FaMe](#fame)
1. [Links](#links)


## Default usage

To install the package use:

```bash
make
```

### Ubuntu 24.04

to install dependecies use:
```bash
make 
```


### Ubuntu 20.04 (Old)

From a freshly installed Ubuntu 20.04:

[Warn] there might be still some issues currently as it has not been completely tested

```bash
make install_all
```

Afterward use one of the `launch` function in the Makefile to see the result. Example:

```bash
make launch_example
```
## Exploiting the tello

### Setup

```bash
make setup_ros2_shared
make setup_tello_msgs
make setup_tello
```

### Launch

#### Tello ROS nodes

```bash
make launch_tello_controller
```

#### Tello control via joystick

To controll a Tello with a controller use:

```bash
python3 ../PFE/tello_SDK/control_manette.py
```

or

```bash
python3 ../PFE/tello_SDK/control_manette_multiple.py
```

#### Gazebo

## Using Husky A300

### Husky workspace

### Reinstall from scratch

## FaMe

### FaMe modeler

### FaMe engine

#### FaMe agricultural

does not work as explained

#### FaMe simulation

was not used

## Links

### link for FaMe

[https://bitbucket.org/proslabteam/fame/src/master/](https://bitbucket.org/proslabteam/fame/src/master/)