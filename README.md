# PFE

## Summary

The Makefile of this project automates the installation and execution of **ROS2**, **Gazebo**, **Tello**, **Husky**, and **FaMe** environments.  
It handles cloning, setup, build, and launch of all related projects.


1. [Default usage](#default-usage)
2. [Tello](#tello)
3. [Husky A300](#husky-a300)
4. [FaMe](#fame)
5. [Links](#links)

---

## Default usage

To install the whole environment (ROS, Gazebo, dependencies, packages):

```bash
make
```

> This initializes `~/PFE`, clones submodules, and installs required dependencies.

For more detailed explanations, see [README_eng.md](../PFE/README_eng.md) or [README_fr.md](../PFE/README_fr.md). But beware that everything might not work completely as everything was made from scratch and had some modifications at some points.

### Ubuntu 24.04

Use ROS2 **Jazzy** and Gazebo **Harmonic**:

```bash
make apt_install
```

### Ubuntu 20.04 (Legacy)

Uses ROS2 **Foxy** and Gazebo Classic:

```bash
make install_all
```


---

## Tello

### Setup

Prepare dependencies and build:

```bash
make setup_ros2_shared
make setup_tello_msgs
make setup_tello
```

### Launch

#### ROS2 Tello nodes

```bash
make launch_tello_controller
```

#### Control via joystick (SDK)

```bash
python3 ../PFE/tello_SDK/control_manette.py
# or
python3 ../PFE/tello_SDK/control_manette_multiple.py
```

#### Gazebo simulation

Tello-related simulation can be launched through FaMe simulation (see below).

---

## Husky A300

### Setup

Clone and build the Husky workspace:

```bash
make clone_husky_2004
make setup_husky
```

### Reinstall from scratch

```bash
make clear_husky
make setup_husky
```

### Launch example

```bash
make FaMe_husky
```

---

## FaMe

FaMe is the core multi-robot and agricultural simulation framework.

### Setup

```bash
make setup_FaMe
make setup_FaMe_engine
make setup_FaMe_agricultural
make setup_FaMe_simulation
```

### Launch

#### Example

```bash
make launch_example
```

#### Engine or Simulation

```bash
make FaMe_engine_example
make FaMe_simulation_multi
```

#### Agricultural

```bash
make FaMe_agricultural_multi
make launch_comportement_agri
```

---

## Prerequisites

* Ubuntu **20.04** or **24.04**
* `sudo` privileges
* Internet connection
* ~15 GB free disk space
* Recommended hardware: 4+ cores, 16 GB RAM, GPU for Gazebo

---

## Links

* **FaMe repository:** [https://bitbucket.org/proslabteam/fame/src/master/](https://bitbucket.org/proslabteam/fame/src/master/)
* **Tello ROS messages:** [https://github.com/clydemcqueen/tello_ros](https://github.com/clydemcqueen/tello_ros)
* **ROS2 Shared:** [https://github.com/ptrmu/ros2_shared](https://github.com/ptrmu/ros2_shared)

```

