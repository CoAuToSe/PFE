# Makefile — Automated setup for development environment @04Jun2025
# ----------------------------------------------------
# Usage examples:
#   make              # install everything (default)
#   make install_vscode
#   make test_ros2
#   make versions
# ----------------------------------------------------

SHELL := /bin/bash

.PHONY: all install_all install_terminator install_vscode correct_vscode install_ros2_foxy install_gazebo install_python_3_10 \
        test_ros2 test_gazebo versions install_FaMe_modeler run-FaMe install_nvm install_node install_cmake \
		install_discord-snap install_deps clone_ros2_shared setup_ros2_shared clone_tello_msgs setup_tello_msgs install_examples \
		setup_FaMe_agri copy_models_FaMe_agri setup_gazebo launch_gazebo install_FaMe_engine launch_comportement setup_FaMe_simulation \
		install_github_desktop_2004 min_install_2004 install_github_desktop_2404 min_install_2404

DELAY ?= 20

# Default target --------------------------------------------------------------
all: min_install_2004
clean:
	make -i try_clean

try_clean: 						\
	clear_ros2_shared 			\
	clear_tello_msgs 			\
	clear_fame_agri				\
	clear_ros2_FaMe_engine		\
	clear_pfe_simulation_gazebo	\

min_install_2004: 				\
	sudo_upgrade				\
	install_github_desktop_2004	\
	install_python_3_10			\
	install_software			\
	install_software_2004
	@echo "After clonning you need to execute 'make copy_from_github'"

min_install_2404: 				\
	sudo_upgrade				\
	install_github_desktop_2404	\
	install_deps_2404			\
	install_software

install_deps_2404:	\
	install_git			

# Install targets -------------------------------------------------------------
install_software:			\
	sudo_upgrade			\
	install_terminator		\
	install_cmake			\
	install_discord-snap	\
	install_vscode			\
	correct_vscode			\
	install_FaMe_modeler	\
	install_deps

install_software_2004:	\
	install_gazebo		\
	install_ros2_foxy		

# Aggregate install target ----------------------------------------------------
install_all: 			\
	install_software	\
	install_all2

install_all2: 					\
	clone_ros2_shared 			\
	setup_ros2_shared			\
	clone_tello_msgs 			\
	setup_tello_msgs			\
	install_examples 			\
	setup_FaMe_agri 			\
	copy_models_FaMe_agri		\
	setup_gazebo 				\
	install_FaMe_engine 		\
	setup_FaMe_simulation 		\
	install_github_desktop_2004 \
	setup_bashrc 				\
	setup_pfe_simulation_gazebo

update: sudo_upgrade


define _clear_ros
	if [ -d "build" ] && [ -d "install" ] && [ -d "log" ]; then 					\
		echo "Tous les dossiers sont présents. Suppression..."; 					\
		rm -rf "build" "install" "log"; 											\
	else 																			\
		echo "Un ou plusieurs dossiers sont manquants. Aucun dossier supprimé."; 	\
	fi;	
endef

# missing dependencies
install_git:
	sudo apt install -y git

# 0-Update System
sudo_upgrade:
	sudo apt update
	sudo apt upgrade -y

update_source:
	source ~/.bashrc

refresh-env:
	@echo "Sourcing .bashrc..."
	source ~/.bashrc 
	@echo "Environment refreshed"

# install_FaMe: 
# 	git clone https://github.com/SaraPettinari/fame-modeler.git -q
# 	cd fame-modeler/
# 	pwd
# 	npm install
# 	@npm run start

install_snap:
	sudo apt update
	sudo apt install snapd

install_cmake: install_snap
	sudo snap install cmake --classic

install_FaMe_modeler: update_source install_node update_source
	@if [ ! -d "fame-modeler" ]; then \
		git clone https://github.com/SaraPettinari/fame-modeler.git; \
	else \
		echo "Directory 'fame-modeler' already exists. Skipping clone."; \
	fi
	cd fame-modeler && . $$HOME/.nvm/nvm.sh && npm install

run-FaMe: install_FaMe_modeler update_source
	cd fame-modeler && . $$HOME/.nvm/nvm.sh && npm run start &


install_nvm: update_source
	curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
	@echo "NVM installation complete."

NPM_VERSION := 16

install_node: install_nvm update_source
#TODO change install depending on version to have only lst versions of npm
	@echo "Setting up Node.js version ${NPM_VERSION}..."
	@export NVM_DIR="$$HOME/.nvm" && \
	. $$NVM_DIR/nvm.sh && \
	nvm install --lts=gallium && \
	nvm use ${NPM_VERSION} && \
	nvm alias default ${NPM_VERSION}
	@echo "Node.js version ${NPM_VERSION} is now active."


# install_node: install_nvmtest
# 	nvm install 18
# 	nvm use 18
# 	nvm alias default 18

# install_nvm:
# 	curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
# 	export NVM_DIR="$$HOME/.nvm"
# 	[ -s "$$NVM_DIR/nvm.sh" ] && \. "$$NVM_DIR/nvm.sh"  # This loads nvm
# 	[ -s "$$NVM_DIR/bash_completion" ] && \. "$$NVM_DIR/bash_completion"  # This loads nvm bash_completion
	
install_nvmtest: install_nvm refresh-env
#command -v nvm
	source ~/.bashrc && nvm -v


# 1 — Terminator --------------------------------------------------------------
install_terminator:
	sudo add-apt-repository -y ppa:mattrose/terminator
	sudo apt update
	sudo apt install -y terminator

# 2 — Visual Studio Code -------------------------------------------------------
install_vscode:
	sudo apt-get update
	sudo apt-get install -y wget gpg apt-transport-https
	wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
	sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
	echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
	rm -f packages.microsoft.gpg
	sudo apt update
	sudo apt install -y code  # or code-insiders

correct_vscode:
	sudo rm -f /etc/apt/sources.list.d/vscode.list
	sudo rm -f /etc/apt/sources.list.d/vscode.sources
	sudo rm -f /usr/share/keyrings/microsoft.gpg
	sudo rm -f /etc/apt/keyrings/packages.microsoft.gpg
	
	sudo mkdir -p /etc/apt/keyrings
	wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/microsoft.gpg > /dev/null

	echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | \
	sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

	sudo apt update
	sudo apt install code -y


	# sudo apt install wget gpg
	# wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
	# sudo install -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/
	# sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
	# sudo apt update


# 3 — ROS 2 Foxy --------------------------------------------------------------
install_ros2_foxy: install_cmake
	@echo "Bienvenu dans l'installation de ROS2 Foxy"
	locale || true                               # check current locale (non‑fatal)
	sudo apt update
	sudo apt install -y locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8
	locale || true                               # verify settings
	sudo apt install -y software-properties-common curl
	sudo add-apt-repository -y universe
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $$(. /etc/os-release && echo $$UBUNTU_CODENAME) main" | \
		sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	sudo apt update
	sudo apt upgrade -y
	sudo apt install -y ros-foxy-desktop python3-argcomplete ros-dev-tools
	# Add ROS 2 environment setup to bashrc only once
	grep -qxF "source /opt/ros/foxy/setup.bash" $$HOME/.bashrc || ( \
		echo "" >> $$HOME/.bashrc && \
		echo "# ROS 2 Foxy" >> $$HOME/.bashrc && \
		echo "source /opt/ros/foxy/setup.bash" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)
	sudo apt install ros-foxy-nav2-bringup -y

setup_bashrc:
# Add some custom information into ~/.bashrc
	grep -qxF "# Custom commands" $$HOME/.bashrc || ( \
		echo "# Custom commands" >> $$HOME/.bashrc && \
		echo "alias ros-build=\"colcon build && source install/setup.bash\"" >> $$HOME/.bashrc && \
		echo "alias ros-build-sym=\"colcon build --symlink-install && source install/setup.bash\"" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)

# 4 — Gazebo 11 (classic) ------------------------------------------------------
install_gazebo:
	sudo apt update
	sudo apt install -y ros-foxy-gazebo-ros-pkgs
# deps Gazebo
	sudo apt install libasio-dev

# 5 — Python 3.10 & pip --------------------------------------------------------
install_python_3_10:
	sudo apt update
	sudo apt install -y software-properties-common
	sudo add-apt-repository -y ppa:deadsnakes/ppa
	sudo apt update
	sudo apt install -y python3.10 python3.10-venv python3.10-dev
	curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
	$(pip) --version
	$(pip) install pyparrot djitellopy


install_discord-snap: install_snap
	@echo "Installation de Discord via Snap..."
	@sudo snap install discord
	@echo "Discord installé avec Snap."

# -----------------------------------------------------------------------------
# Test targets (run manually; they only print the commands to execute) ---------

# ROS 2 minimal pub/sub demo
# Open **two** terminals and run the printed commands.

test_ros2:
	@echo "Terminal 1 ➜ source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"
	@echo "Terminal 2 ➜ source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_py listener"

# Gazebo + Twist publisher demo

test_gazebo:
	@echo "Terminal 1 ➜ gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world"
	@echo "Terminal 2 ➜ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1"

# ------------------------------Session-2--------------------------------------


# Variables
HOME_DIR := $(PWD)
ROS2_SHARED := $(HOME_DIR)/ros2_shared
TELLO_MSGS := $(HOME_DIR)/tello_msgs
FAME := $(HOME_DIR)/fame
FAME_AGRI := $(FAME)/fame_agricultural
FAME_ENGINE := $(FAME)/fame_engine
FAME_SIMU := $(FAME)/fame_simulation
GZ_MODEL_DIR := $(HOME_DIR)/.gazebo/models # might need to be $(HOME) and not $(HOME_DIR)
MBROS_DIR := /home/ubuntu/mbros/fame_engine/process

define pip 
	python3.10 -m pip 
endef

install_deps:
# sudo apt update
	sudo apt install python3-pip -y
	$(pip) install transformations djitellopy

clone_ros2_shared:
	@if [ ! -d "$(ROS2_SHARED)" ]; then \
		git clone https://github.com/ptrmu/ros2_shared.git $(ROS2_SHARED); \
	fi

setup_ros2_shared:
	cd $(ROS2_SHARED) && colcon build && source install/setup.bash

clear_ros2_shared:
	@cd $(ROS2_SHARED) && echo -n "[$(ROS2_SHARED)] " && $(call _clear_ros)

clone_tello_msgs:
	@if [ ! -d "$(TELLO_MSGS)" ]; then \
		git clone https://github.com/clydemcqueen/tello_ros.git $(TELLO_MSGS); \
	fi

setup_tello_msgs: setup_ros2_shared
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(TELLO_MSGS) && nvm install --lts=gallium && nvm use 16 && \
		cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && colcon build && source install/setup.bash

clear_tello_msgs:
	@cd $(TELLO_MSGS) && echo -n "[$(TELLO_MSGS)] " && $(call _clear_ros)

install_examples:
	@if [ ! -d "$(FAME)" ]; then \
		git clone https://bitbucket.org/proslabteam/fame.git $(FAME); \
	fi

setup_FaMe_agri: setup_tello_msgs install_examples install_node
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_AGRI) && nvm install --lts=gallium && nvm use 16 && \
		cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		cd $(FAME_AGRI) && colcon build
	cd $(FAME_AGRI) && source install/setup.bash && source /usr/share/gazebo/setup.bash

copy_models_FaMe_agri:
	mkdir -p $(GZ_MODEL_DIR)
	cp -R $(FAME_AGRI)/models/* $(GZ_MODEL_DIR)

clear_fame_agri:
	@cd $(FAME_AGRI) && echo -n "[$(FAME_AGRI)] " && $(call _clear_ros)

launch_gazebo:
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
		ros2 launch fame_agricultural multi_launch.py \
			# --ros-args -r gazebo_ros_force:=blade_force

install_FaMe_engine:
	# sudo apt update
	# sudo apt install ros-foxy-rmw-cyclonedds-cpp -y
	sudo mkdir -p $(MBROS_DIR)
	sudo ln -sf $(FAME_ENGINE)/process $(MBROS_DIR)
	# @export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
	# 	cd $(FAME_ENGINE) && nvm install 12 && nvm use 12
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm install --lts=gallium && nvm use 16
	cd $(FAME_ENGINE) && rm -rf node_modules package-lock.json

	# @export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
	# 	cd $(FAME_ENGINE) && nvm install 12 && nvm use 12 && \
	# 	cd $(FAME_ENGINE) && npm install
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm install --lts=gallium && nvm use 16 && \
		cd $(FAME_ENGINE) && npm install

	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
		cd $(FAME_ENGINE) && colcon build
	

	cd $(FAME_ENGINE) && rm -rf node_modules package-lock.json

	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
		cd $(FAME_ENGINE) && npm pkg set "dependencies.rclnodejs=^0.21.0"

	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
		cd $(FAME_ENGINE) && npm install

	cd $(FAME_ENGINE)/install/fame_engine/share/fame_engine && rm -rf node_modules/rclnodejs
	cd $(FAME_ENGINE)/install/fame_engine/share/fame_engine && npm install rclnodejs@^0.21.0  

	# @export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
	# 	cd $(FAME_ENGINE) && npm i rclnodejs@^0.21.0  

	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
		cd $(FAME_ENGINE) && colcon build

	# export NODE_OPTIONS="--unhandled-rejections=strict"
	# ros2 launch fame_engine agri_engine.launch.py

setup_FaMe_engine:
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
		cd $(FAME_ENGINE) && colcon build

clear_ros2_FaMe_engine:
	@cd $(FAME_ENGINE) && echo -n "[$(FAME_ENGINE)] " && $(call _clear_ros)

	

MBROS_DIR      := /home/ubuntu/mbros/fame_engine
NVM_SCRIPT     := $$HOME/.nvm/nvm.sh          # ≠ variable d’env. de nvm
NODE_VERSION   := 16                          # LTS Gallium (ABI 93)

.PHONY: install_FaMe_engine
# install_FaMe_engine_not_opti_GPT:
# 	# 1. Lien symbolique une seule fois
# 	sudo install -d $(MBROS_DIR)
# 	sudo ln -sf $(FAME_ENGINE)/process $(MBROS_DIR)

# 	# 2. Tout le reste dans un shell bash unique
# 	@bash -ec '\
# 		. $(NVM_SCRIPT); nvm install --lts=gallium; nvm use $(NODE_VERSION); \
# 		cd $(FAME_ENGINE); \
# 		# Dépendances Node : install propre + version exacte de rclnodejs
# 		npm pkg set dependencies.rclnodejs="0.27.1"; \
# 		npm ci --prefer-offline; \
# 		# Génération des messages JS (oblige rclnodejs >=0.20)      
# 		npx rclnodejs-cli generate-ros-messages; \
# 		# Build ROS 2 : symlink-install pour éviter les copies redondantes
# 		colcon build --packages-select fame_engine; \
# 	'


# ros2-source:
# 	cd $(ROS2_SHARED) && source install/setup.bash
# 	cd $(TELLO_MSGS) && source install/setup.bash
# 	cd $(FAME_AGRI) && source install/setup.bash && source /usr/share/gazebo/setup.bash
#deprecated
launch_comportement:
	@echo "be sure to have use 'make install_FaMe_engine' before using this command"
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm install --lts=gallium && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		ros2 launch fame_engine agri_engine.launch.py


.ONESHELL: launch_comportement_agri
launch_comportement_agri:
	make -i kill_all
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && source install/setup.bash && \
		ros2 launch fame_agricultural multi_launch.py &
	PID_SIM=$$!
	sleep $(DELAY)

	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && source install/setup.bash && \
		ros2 launch fame_engine agri_engine.launch.py
	PID_ENG=$$!
	wait $$PID_SIM $$PID_ENG
	@echo "======= agri and engine done ======="

setup_FaMe_simulation:
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && colcon build


launch_example_alone:
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		ros2 launch fame_engine example.launch.py 
	
setup_example: setup_tello_msgs setup_FaMe_engine setup_FaMe_simulation setup_FaMe_agri


.ONESHELL: launch_example
launch_example:
	make -i kill_all
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && source install/setup.bash && \
		ros2 launch fame_simulation multi_launch.py &
	PID_SIM=$$!
	sleep $(DELAY)

	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && source install/setup.bash && \
		ros2 launch fame_engine example.launch.py
	PID_ENG=$$!
	wait $$PID_SIM $$PID_ENG
	@echo "======= simu and engine done ======="

# # garantit qu’un seul shell est utilisé pour toute la recette
# .ONESHELL: launch_example_GPT
# launch_example_GPT:
# # Prépare env. ROS + Node une seule fois
# 	source /usr/share/gazebo/setup.bash
# 	export NVM_DIR="$$HOME/.nvm"; . "$$NVM_DIR/nvm.sh"; nvm use 16 >/dev/null
# 	export NODE_OPTIONS="--unhandled-rejections=strict"

# 	for ws in "$(ROS2_SHARED)" "$(TELLO_MSGS)" "$(FAME_AGRI)" "$(FAME_ENGINE)"; do \
# 	  [ -f "$$ws/install/setup.bash" ] && source "$$ws/install/setup.bash"; \
# 	done

# # Lance la simulation en arrière-plan, capture son PID
# 	@echo "▶️  Launch simulation"
# 	ros2 launch fame_simulation multi_launch.py & \
# 	PID_SIM=$$!

# 	# Délai paramétrable sans bloquer la simulation
# 	@echo "⏳ Waiting $$DELAY s…"; sleep $(DELAY)

# 	# Lance le moteur de comportement, capture son PID
# 	@echo "▶️  Launch engine"
# 	ros2 launch fame_engine example.launch.py & \
# 	PID_ENG=$$!

# 	# Attend proprement la fin des deux processus
# 	wait $$PID_SIM $$PID_ENG
# 	@echo "✅  Both launches exited."

launch_fame_modeler:
	cd ./fame-modeler && npm start

# cd /home/dell/ros2_shared && source install/setup.bash && \
# 	cd /home/dell/tello_msgs && source install/setup.bash && \
# 	source /usr/share/gazebo/setup.bash && \
# 	export NVM_DIR="$HOME/.nvm" && . $NVM_DIR/nvm.sh && \
# 	cd /home/dell/fame/fame_engine && nvm install --lts=hydrogen && nvm use 18 && \
# 	export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 	cd /home/dell/fame/fame_agricultural && source install/setup.bash && \
# 	cd /home/dell/fame/fame_engine && source install/setup.bash && \
# 	ros2 launch fame_engine agri_engine.launch.py

kill_all:
	killall -9 gzserver
	killall -9 gzclient

#ensta3012*

PATH_TELLO_WS=$(PWD)/Simulation_Gazebo/tello_ros_ws

setup_pfe_simulation_gazebo:
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && source install/setup.bash && \
		cd $(PATH_TELLO_WS) && \
		colcon build

clear_pfe_simulation_gazebo:
	@cd $(PATH_TELLO_WS) && echo -n "[$(PATH_TELLO_WS)] " && $(call _clear_ros)

PATH_TELLO_WS=$(PWD)/Simulation_Gazebo_old/tello_ros_ws
clear_pfe_simulation_gazebo_old:
	@cd $(PATH_TELLO_WS) && echo -n "[$(PATH_TELLO_WS)] " && $(call _clear_ros)

PATH_TELLO_WS=$(PWD)/Simulation_Gazebo_SW/tello_ros_ws
clear_pfe_simulation_gazebo_SW:
	@cd $(PATH_TELLO_WS) && echo -n "[$(PATH_TELLO_WS)] " && $(call _clear_ros)


launch_pfe_simulation_gazebo:
	make -i kill_all
	cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		source /usr/share/gazebo/setup.bash && \
		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_ENGINE) && nvm use 16 && \
		export NODE_OPTIONS="--unhandled-rejections=strict" && \
		cd $(FAME_AGRI) && source install/setup.bash && \
		cd $(FAME_ENGINE) && source install/setup.bash && \
		cd $(FAME_SIMU) && source install/setup.bash && \
		cd $(PATH_TELLO_WS) && source install/setup.bash && \
		ros2 launch tello_gazebo someaze.py


install_github_desktop_2004:
# # UPDATE (2024-01-24)

# ## Direct copy-paste from official instrubtions
# ## Github Desktop for Ubuntu
# ## Get the @shiftkey package feed
# 	wget -qO - https://apt.packages.shiftkey.dev/gpg.key | gpg --dearmor | sudo tee /usr/share/keyrings/shiftkey-packages.gpg > /dev/null
# 	sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/shiftkey-packages.gpg] https://apt.packages.shiftkey.dev/ubuntu/ any main" > /etc/apt/sources.list.d/shiftkey-packages.list'
# ## Install Github Desktop for Ubuntu
# 	sudo apt update && sudo apt install github-desktop

	if [ ! -f "$(PWD)/GitHubDesktop-linux-2.9.6-linux1.deb" ]; then \
		wget https://github.com/shiftkey/desktop/releases/download/release-2.9.6-linux1/GitHubDesktop-linux-2.9.6-linux1.deb;
	fi
	sudo apt-get update
	sudo apt-get install gdebi-core -y
	sudo gdebi GitHubDesktop-linux-2.9.6-linux1.deb -y
	sudo dpkg -i GitHubDesktop-linux-2.9.6-linux1.deb 
	sudo apt-get install -f -y
	sudo apt-mark hold github-desktop


install_github_desktop_2404:
# # UPDATE (2024-01-24)

# ## Direct copy-paste from official instrubtions
# ## Github Desktop for Ubuntu
# ## Get the @shiftkey package feed
# 	wget -qO - https://apt.packages.shiftkey.dev/gpg.key | gpg --dearmor | sudo tee /usr/share/keyrings/shiftkey-packages.gpg > /dev/null
# 	sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/shiftkey-packages.gpg] https://apt.packages.shiftkey.dev/ubuntu/ any main" > /etc/apt/sources.list.d/shiftkey-packages.list'
# ## Install Github Desktop for Ubuntu
# 	sudo apt update && sudo apt install github-desktop

	if [ ! -f "$(PWD)/GitHubDesktop-linux-3.1.1-linux1.deb" ]; then \
		wget https://github.com/shiftkey/desktop/releases/download/release-3.1.1-linux1/GitHubDesktop-linux-3.1.1-linux1.deb;
	fi
	sudo apt-get update
	sudo apt-get install gdebi-core -y
	sudo gdebi GitHubDesktop-linux-3.1.1-linux1.deb -y
	sudo dpkg -i GitHubDesktop-linux-3.1.1-linux1.deb 
	sudo apt-get install -f -y
# 	sudo apt-mark hold github-desktop


# -----------------------------------------------------------------------------
# Helper target: display installed versions -----------------------------------

versions:
	python3 --version || true
	python3.10 --version || true
	node -v # Bonus

setup_gazebo_models:
	@echo "for the moment unable to find where does the coke can belongs from"
	@echo "so, for the moment, you need to copy the \`.gazebo/models\` folder to you working space"

# /====================================\
# |         Github integration         |
# \====================================/

check_with_user_first_time:
	@echo ""
	@echo "You are REALLY going to ERASE EVERYTHING of the local version"
	@echo "So are you really sure you want to do it ?"
	@read -p ""

check_with_user:
	@echo "You are going to erase the local version"
	@read -p "Press enter to continue"
	@read -p "Just to be sure press enter again to continue"
	@read -p "Jk do it again"


copy_to_github:						\
	copy_simu_gazebo_from_Github	\
	copy_makefile_from_Github

copy_from_github:					\
	check_with_user					\
	check_with_user_first_time		\
	copy_simu_gazebo_from_Github	\
	copy_makefile_from_Github


copy_simu_gazebo_to_Github: 
	cp -r ~/Simulation_Gazebo/tello_ros_ws/ ~/PFE/Simulation_Gazebo_new/

copy_simu_gazebo_from_Github: check_with_user
	cp -r ~/PFE/Simulation_Gazebo_new/ ~/Simulation_Gazebo/tello_ros_ws/ 


copy_makefile_to_Github:
	cp  ~/Makefile ~/PFE/Makefile

copy_makefile_from_Github: check_with_user
	cp  ~/PFE/Makefile ~/Makefile

#TODO: copy gazebo models