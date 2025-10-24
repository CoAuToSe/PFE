# PFE — Install & Launch Makefile

This repository includes a **Makefile** to install dependencies, clone sub-projects (ROS2, Tello, FaMe, Husky), set up the environment, build packages, and launch various demos/simulations.

> Primarily tested on **Ubuntu 20.04 (Focal)** and **Ubuntu 24.04 (Noble)**.  
> Default paths assume `~/PFE`. The Makefile tries to detect your Ubuntu version and install the right packages.

---

## Table of Contents

1. [Overview](#overview)
2. [Prerequisites](#prerequisites)
3. [Quickstart](#quickstart)
4. [Ubuntu Version Handling](#ubuntu-version-handling)
5. [Key Targets](#key-targets)
6. [Structure & Important Variables](#structure--important-variables)
7. [Tello (usage)](#tello-usage)
8. [Husky A300](#husky-a300)
9. [FaMe (modeler, engine, simulation)](#fame-modeler-engine-simulation)
10. [GitHub Integration (backup/restore)](#github-integration-backuprestore)
11. [Maintenance & Cleaning](#maintenance--cleaning)
12. [Tips & Troubleshooting](#tips--troubleshooting)
13. [Links](#links)

---

## Overview

- **Goal:** Automate installation (ROS2, Gazebo, Node/NVM, VS Code, etc.), workspace prep, `colcon` builds, and launch of **Tello**, **Husky**, and **FaMe** scenarios (agri/sim).
- **Approach:**
  - `install_*` targets to install software.
  - `clone_*` / `setup_*` to fetch/build ROS projects.
  - `launch_*` to run ROS2/Gazebo scenarios.
  - `copy_*_to/from_github` to back up/restore local files and folders.

---

## Prerequisites

- OS: **Ubuntu 20.04** or **Ubuntu 24.04** (others not supported by default).
- `sudo` access.
- Internet connection.
- Sufficient disk space (ROS2 + Gazebo + workspaces).

---

## Quickstart

### Default install

```bash
# In the directory containing the Makefile
make
````

This runs `git_init_PFE` (clones PFE into `~/PFE`, initializes submodules) then `apt_install` according to your Ubuntu version.

### Run an example

After install and builds:

```bash
make launch_example
```

This orchestrates a FaMe simulation + engine with a configurable delay (`DELAY`, default 20s).

---

## Ubuntu Version Handling

The Makefile auto-detects Ubuntu:

* **Ubuntu 24.04**

  * Dependencies via `make apt_install_24.04`
  * ROS 2 **Jazzy**, Gazebo **Harmonic**
  * Helper: `make setup_2404` (upgrade, GitHub Desktop, deps, software)

* **Ubuntu 20.04** (legacy)

  * Dependencies via `make apt_install_20.04`
  * ROS 2 **Foxy**, Gazebo Classic
  * **Note:** Parts are older and may need adjustments.

---

## Key Targets

> Full list: `make list`

### Base install

* `make apt_install` → calls `apt_install_24.04` or `apt_install_20.04`.
* `make install_software` → Terminator, Discord (snap), FaMe Modeler (deprecated), etc.
* `make install_node` → installs **NVM**, **Node.js LTS (Gallium)** and sets default version.
* `make install_python_3_10` → installs Python 3.10, pip, and a few libs.
* **24.04**:

  * `make install_ros2_jazzy` (or `install_ros2_jazzy_bis`)
  * `make install_gazebo_2404`
  * `make install_vscode_2404`
* **20.04**:

  * `make install_ros2_foxy`
  * `make install_gazebo_2004`
  * `make install_vscode_2004` and `make correct_vscode_2004`

### Clone & build ROS projects

* Clone: `make clone_ros2_shared`, `make clone_tello_msgs`, `make clone_FaMe_bitbucket`, `make clone_husky_2004`
* Setup/build:

  * `make setup_ros2_shared`, `make setup_tello_msgs`, `make setup_husky`, `make setup_tello`,
    `make setup_FaMe`, `make setup_FaMe_engine`, `make setup_FaMe_agricultural`, `make setup_FaMe_simulation`
  * These targets **source** required environments (`install/setup.bash`, `ROS2_SETUP`), run `nvm use` if needed, then build with `colcon` (often `--symlink-install`; some use `build` or `npm` + `colcon`).
* Clean a package: `make clear_<pkg>` (removes `build/ install/ log/` and sometimes `node_modules`).

### Launch (ROS2)

* **FaMe**:

  * `make launch_FaMe_engine_example`
  * `make launch_FaMe_simulation_multi`
  * `make launch_comportement_agri` (runs simulation then engine after `DELAY`)
  * Also: `make FaMe_CATS`, `make FaMe_agricultural_multi`, etc.
* **Tello**:

  * `make launch_tello_controller` (ROS2 Tello)
* **Husky**:

  * `make FaMe_husky`, `make FaMe_husky_tello`

### Shell env configuration

* `make setup_bashrc` adds a **“CATS Custom commands”** block to `~/.bashrc`:

  * `source /opt/ros/$ROS_DISTRO/setup.bash`
  * Handy aliases: `ros-build`, `ros-build-sym`, `ros-sc`, etc.
  * Function `this-sc <path>` to source the target’s `install/setup.bash`.

---

## Structure & Important Variables

* **Project root:** `PFE := $(HOME)/PFE`
* **Workspaces:**

  * `ROS2_SHARED := $(PFE)/ros2_shared`
  * `TELLO_MSGS := $(PFE)/tello_msgs`
  * `FAME := $(PFE)/my_FaMe`
  * `FAME_ENGINE := $(FAME)/fame_engine`
  * `FAME_AGRI := $(FAME)/fame_agricultural`
  * `FAME_SIMU := $(FAME)/fame_simulation`
  * `HUSKY_WS := $(HOME)/husky_ws`
* **Gazebo:** `GZ_MODEL_DIR := ~/.gazebo/models`
* **ROS:** `ROS2_SETUP := /opt/ros/$ROS_DISTRO/setup.bash`
* **Node/NVM:**

  * `NVM_SCRIPT := $HOME/.nvm/nvm.sh`
  * `NODE_VERSION := 16` (ABI 93, Gallium)
  * `NPM_VERSION := 16`
* **Orchestration delay:** `DELAY ?= 20` (seconds) for chained launches.

---

## Tello (usage)

### Setup

```bash
make setup_ros2_shared
make setup_tello_msgs
make setup_tello
```

> Depending on Ubuntu version, `make correct_git_clone` installs the proper `ros-$ROS_DISTRO-*` packages and drops `COLCON_IGNORE` in unused Tello packages.

### Launch

#### ROS Tello nodes

```bash
make launch_tello_controller
```

#### Control Tello via joystick (SDK)

```bash
python3 ../PFE/tello_SDK/control_manette.py
# or
python3 ../PFE/tello_SDK/control_manette_multiple.py
```

#### Gazebo

Use FaMe launch targets (below) or your own ROS2/Gazebo launches based on built packages.

---

## Husky A300

### Husky workspace

* Clone/build: `make clone_husky_2004` then `make setup_husky`
* Helper: `make setup_husky_launch` (copies a custom launch into the Husky ws)

### Reinstall from scratch

```bash
make clear_husky
make setup_husky
```

> Some Husky packages rely on `xacro`, `ros2-control`, etc., which are installed via `apt_install_*`.

---

## FaMe (modeler, engine, simulation)

### FaMe modeler

* (Deprecated) `make install_FaMe_modeler`: clone + `npm install` + start.
* Linux modeler upgrade: `make upgrade_linux_for_FaMe-modeler` (updates Node, `inotify`, `lib` symlink, etc.)

### FaMe engine

* Setup: `make setup_FaMe_engine`
* Launch examples:

  ```bash
  make launch_FaMe_CATS
  make launch_FaMe_engine_example
  ```

### FaMe agricultural

* Setup: `make setup_FaMe_agricultural`
* Launch:

  ```bash
  make FaMe_agricultural_multi
  make launch_comportement_agri
  ```

### FaMe simulation

* Setup: `make setup_FaMe_simulation`
* Launch:

  ```bash
  make FaMe_simulation_multi
  make launch_example
  ```

> Some targets also source `/usr/share/gazebo/setup.bash` and/or export `NODE_OPTIONS="--unhandled-rejections=strict"`.

---

## GitHub Integration (backup/restore)

Generic targets copy files/folders between your machine and `$(PFE)` (useful for versioning via Git):

* **Backup to PFE** (under `$(PFE)/…`):

  * `make copy_to_github`

    * includes: `copy_simu_gazebo_to_github`, `copy_makefile_to_github`, `copy_bashrc_to_github`,
      `copy_code_setup_to_github`, `copy_gazebo_models_to_github`, `copy_FaMe_to_github`, etc.

* **Restore from PFE** (**WARNING:** multiple confirmations):

  * `make copy_from_github`

    * includes: `copy_simu_gazebo_from_github`, `copy_code_setup_from_github`,
      `copy_gazebo_models_from_github`, `copy_FaMe_from_github`, `copy_makefile_from_github`, etc.

> Folder copies use `rsync -a --delete` (full sync).
> `clean_<name>` targets **remove** the destination (guarded by confirmations).

---

## Maintenance & Cleaning

* **Clean a ROS package** (removes `build/ install/ log/`):

  ```bash
  make clear_<pkg>
  # e.g.:
  make clear_FaMe_engine
  ```
* **Kill Gazebo processes**:

  ```bash
  make kill_all
  ```
* **Refresh bash environment**:

  ```bash
  make setup_bashrc
  source ~/.bashrc
  ```

---

## Tips & Troubleshooting

* **List all available targets:**

  ```bash
  make list
  ```

* **Force the right ROS in current shell:**

  ```bash
  export ROS_DISTRO=jazzy   # or foxy
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```

* **NVM/Node missing during build/launch:**

  * Run `make install_nvm` then `make install_node`.
  * Open a **new terminal** (or `source ~/.bashrc`).
  * Check: `command -v nvm && node -v && npm -v`.

* **Permission/ownership errors during GitHub copy:**

  * Targets use `sudo rsync` / `sudo install`. Ensure sudo password and correct paths.

* **Gazebo/ROS version mix-ups:**

  * 20.04 → ROS2 Foxy + Gazebo Classic.
  * 24.04 → ROS2 Jazzy + gz-harmonic.
    Use the proper `apt_install_*` and `install_ros2_*` targets.

* **Change orchestration delay (e.g., `launch_example`):**

  ```bash
  make launch_example DELAY=10
  ```

---

## Links

* **FaMe**: [https://bitbucket.org/proslabteam/fame/src/master/](https://bitbucket.org/proslabteam/fame/src/master/)

