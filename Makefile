# Makefile — Automated setup for development environment @04Jun2025
# ----------------------------------------------------
# Usage examples:
#   make              # install everything (default)
#   make install_vscode
#   make test_ros2
#   make versions
# ----------------------------------------------------

SHELL := /bin/bash

.PHONY: all install_all install_terminator install_vscode_2004 correct_vscode_2004 install_ros2_foxy install_gazebo_2004 install_python_3_10 \
        test_ros2 test_gazebo versions install_FaMe_modeler run_FaMe_modeler install_nvm install_node install_cmake \
		install_discord-snap install_deps clone_ros2_shared setup_ros2_shared clone_tello_msgs setup_tello_msgs install_FaMe \
		setup_FaMe_agri copy_models_FaMe_agri setup_gazebo launch_gazebo_2004 install_FaMe_engine launch_comportement setup_FaMe_simulation \
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
	install_software_2004		\
	install_software_2004_bis
	@echo "After clonning you need to execute 'make copy_from_github'"

min_install_2404: 				\
	sudo_upgrade				\
	install_github_desktop_2404	\
	install_deps_2404			\
	install_software

# /====================================\
# |           Macro  install           |
# \====================================/

install_deps_2404:	\
	install_git			

# Works on 2004 and 2404
install_software:			\
	sudo_upgrade			\
	install_discord-snap	\
	install_terminator		\
	install_code_2404		\


# Install targets -------------------------------------------------------------
install_software_2004:		\
	sudo_upgrade			\
	install_terminator		\
	install_cmake			\
	install_discord-snap	\
	install_vscode_2004		\
	correct_vscode_2004		\
	install_FaMe_modeler	\
	install_deps

install_software_2004:		\
	sudo_upgrade			\
	install_terminator		\
	install_cmake			\
	install_discord-snap	\
	install_vscode_2004		\
	correct_vscode_2004		\
	install_FaMe_modeler	\
	install_deps

install_software_2004_bis:	\
	install_gazebo_2004			\
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
	install_FaMe 				\
	setup_FaMe_agri 			\
	copy_models_FaMe_agri		\
	setup_gazebo 				\
	install_FaMe_engine 		\
	setup_FaMe_simulation 		\
	install_github_desktop_2004 \
	setup_bashrc 				\
	setup_pfe_simulation_gazebo

install_all_2404:\
	install_node

update: sudo_upgrade



# /====================================\
# |            Define Macro            |
# \====================================/

define _clear_ros
	if [ -d "build" ] && [ -d "install" ] && [ -d "log" ]; then 					\
		echo "Tous les dossiers sont présents. Suppression..."; 					\
		rm -rf "build" "install" "log"; 											\
	else 																			\
		echo "Un ou plusieurs dossiers sont manquants. Aucun dossier supprimé."; 	\
	fi;	
endef

define pip 
	python3.10 -m pip 
endef

# /====================================\
# |           System install           |
# \====================================/

sudo_upgrade:
	sudo apt update
	sudo apt upgrade -y


update_source:
	source ~/.bashrc


refresh_env:
	@echo "Sourcing .bashrc..."
	source ~/.bashrc 
	@echo "Environment refreshed"

# /====================================\
# |         install  softwares         |
# \====================================/


# install_FaMe: 
# 	git clone https://github.com/SaraPettinari/fame-modeler.git -q
# 	cd fame-modeler/
# 	pwd
# 	npm install
# 	@npm run start


install_%:
	sudo apt install -y $<

install_git:
	sudo apt install -y git

install_snap:
	sudo apt update
	sudo apt install snapd

install_cmake: install_snap
	sudo snap install cmake --classic

install_nvm: update_source
	curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
	@echo "NVM installation complete."

NPM_VERSION := 16
install_node: install_nvm update_source
#TODO change install depending on version to have only LTS versions of npm
	@echo "Setting up Node.js version ${NPM_VERSION}..."
	@export NVM_DIR="$$HOME/.nvm" && 						\
		. $$NVM_DIR/nvm.sh && 								\
		nvm install --lts=gallium && 						\
		nvm use ${NPM_VERSION} && 							\
		nvm alias default ${NPM_VERSION}
	@echo "Node.js version ${NPM_VERSION} is now active."

install_terminator:
	sudo add-apt-repository -y ppa:mattrose/terminator
	sudo apt update
	sudo apt install -y terminator

install_discord-snap: install_snap
	@echo "Installation de Discord via Snap..."
	@sudo snap install discord
	@echo "Discord installé avec Snap."

install_FaMe_modeler: update_source
	@if [ ! -d "fame-modeler" ]; then 										\
		git clone https://github.com/SaraPettinari/fame-modeler.git; 		\
	else 																	\
		echo "Directory 'fame-modeler' already exists. Skipping clone."; 	\
	fi
	cd fame-modeler && . $$HOME/.nvm/nvm.sh && npm install

install_python_3_10:
	sudo apt update
	sudo apt install -y software-properties-common
	sudo add-apt-repository -y ppa:deadsnakes/ppa
	sudo apt update
	sudo apt install -y python3.10 python3.10-venv python3.10-dev
	curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
	$(pip) --version
	$(pip) install pyparrot djitellopy

install_deps:
# sudo apt update
	sudo apt install python3-pip -y
	$(pip) install transformations djitellopy

# /====================================\
# |       install softwares  2004      |
# \====================================/

install_vscode_2004:
	sudo apt-get update
	sudo apt-get install -y wget gpg apt-transport-https
	wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
	sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
	echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
	rm -f packages.microsoft.gpg
	sudo apt update
	sudo apt install -y code  # or code-insiders

correct_vscode_2004:
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


install_ros2_foxy: install_cmake
	@echo "Bienvenu dans l'installation de ROS2 Foxy"
	locale || true                               # check current locale (non-fatal)
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
# 	Add ROS 2 environment setup to bashrc only once
	grep -qxF "# ROS 2 Foxy" $$HOME/.bashrc || ( \
		echo "" >> $$HOME/.bashrc && \
		echo "# ROS 2 Foxy" >> $$HOME/.bashrc && \
		echo "source /opt/ros/foxy/setup.bash" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)
	sudo apt install ros-foxy-nav2-bringup -y
# 	deps for husky
	sudo apt install ros-foxy-xacro
	sudo apt install ros-foxy-controller-interface ros-foxy-ros2-control ros-foxy-ros2-controllers
	sudo apt install \
		ros-foxy-robot-localization \
		ros-foxy-gazebo-ros-pkgs \
		ros-foxy-ros2-control \
		ros-foxy-ros2-controllers \
		ros-foxy-controller-manager \
		ros-foxy-controller-manager-msgs
	sudo apt install ros-foxy-interactive-marker-twist-server ros-foxy-interactive-markers
	sudo apt install ros-foxy-twist-mux


install_gazebo_2004:
	sudo apt update
	sudo apt install -y ros-foxy-gazebo-ros-pkgs
# deps Gazebo
	sudo apt install libasio-dev


# /====================================\
# |       install softwares  2404      |
# \====================================/

install_vscode_2404:
	sudo apt install code -y

setup_ros2_jazzy:
	@echo "Bienvenu dans le setup de ROS2 Jazzy"
	locale  # check for UTF-8

	sudo apt update && sudo apt install locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

	locale  # verify settings

	sudo apt install software-properties-common
	sudo add-apt-repository universe

	sudo apt update && sudo apt install curl -y
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
	sudo dpkg -i /tmp/ros2-apt-source.deb

install_ros2_jazzy: setup_ros2_jazzy
	@echo "Bienvenu dans l'installation de ROS2 Jazzy"
	sudo apt install ros-jazzy-desktop

	grep -qxF "# ROS 2 Jazzy" $$HOME/.bashrc || ( \
		echo "" >> $$HOME/.bashrc && \
		echo "# ROS 2 Jazzy" >> $$HOME/.bashrc && \
		echo "source /opt/ros/jazzy/setup.bash" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)

install_gazebo_2404:
	sudo apt-get update
	sudo apt-get install curl lsb-release gnupg

	sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	sudo apt-get update
	sudo apt-get install gz-harmonic

	sudo apt-get install ros-jazzy-ros-gz


# /====================================\
# |           bashrc & params          |
# \====================================/

setup_bashrc:
# Add some custom information into ~/.bashrc
	grep -qxF "# Custom commands" $$HOME/.bashrc || ( 																					\
		echo "" >> $$HOME/.bashrc && 																									\
		echo "# Custom commands" >> $$HOME/.bashrc && 																					\
		echo "alias ros-build=\"colcon build && source install/setup.bash\"" >> $$HOME/.bashrc && 										\
		echo "alias ros-build-sym=\"colcon build --symlink-install && source install/setup.bash\"" >> $$HOME/.bashrc && 				\
		echo "alias ros-build-sym-ver="colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON && source install/setup.bash"\"" >> $$HOME/.bashrc && \
		echo "alias ros-build-sym-pac-ver='temp(){ colcon build --packages-select \"\$1\" --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON && source install/setup.bash; unset -temp temp; }; temp'\"" >> $$HOME/.bashrc && \
		echo "alias ros-sc=\"source install/setup.bash\"" >> $$HOME/.bashrc && 															\
		echo "alias sc-ros=\"source install/setup.bash\"" >> $$HOME/.bashrc && 															\
		echo "alias bash-sc=\"source ~/.bashrc\"" >> $$HOME/.bashrc && 																	\
		echo "alias my-sc=\"source $(TELLO_MSGS)/install/setup.bash && source $(ROS2_SHARED)/install/setup.bash\"" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc && 																									\
		echo "this-sc() {" >> $$HOME/.bashrc && 																						\
		echo "    cd "$1" && source install/setup.bash && cd -> /dev/null 2>&1 " >> $$HOME/.bashrc && 									\
		echo "}" >> $$HOME/.bashrc && 																									\
		echo "" >> $$HOME/.bashrc 																										\
	)


# /====================================\
# |          random & unsorted         |
# \====================================/

test_nvm_install: refresh_env
#command -v nvm
	source ~/.bashrc && nvm -v

run_FaMe_modeler:
	cd fame-modeler && . $$HOME/.nvm/nvm.sh && npm run start &

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


kill_all:
	killall -9 gzserver
	killall -9 gzclient

# /====================================\
# |          Paths & Variables         |
# \====================================/

HOME_DIR := $(PWD)
ROS2_SHARED := $(HOME_DIR)/ros2_shared
TELLO_MSGS := $(HOME_DIR)/tello_msgs
FAME := $(HOME_DIR)/fame
FAME_AGRI := $(FAME)/fame_agricultural
FAME_ENGINE := $(FAME)/fame_engine
FAME_SIMU := $(FAME)/fame_simulation
GZ_MODEL_DIR := $(HOME_DIR)/.gazebo/models # might need to be $(HOME) and not $(HOME_DIR)
MBROS_DIR := /home/ubuntu/mbros/fame_engine/process


MBROS_DIR      := /home/ubuntu/mbros/fame_engine
NVM_SCRIPT     := $$HOME/.nvm/nvm.sh          # ≠ variable d’env. de nvm
NODE_VERSION   := 16                          # LTS Gallium (ABI 93)

# /====================================\
# |            package  deps           |
# \====================================/

define from_git_clean
clone_$1:
	if [ ! -f $2 ] ; then mkdir -p $2 ; fi
	if [ -d $2 ] ; then echo -n "clonning $3 into $2" && git clone $3 $2 -b $4 ; fi
clean_$1:
	@rm -r $2
endef

$(eval $(call from_git_clean,ros2_shared,$(ROS2_SHARED),https://github.com/ptrmu/ros2_shared.git,master))
$(eval $(call from_git_clean,tello_msgs,$(TELLO_MSGS),https://github.com/clydemcqueen/tello_ros.git,master))
$(eval $(call from_git_clean,FaMe,$(FAME),https://bitbucket.org/proslabteam/fame.git,master))
$(eval $(call from_git_clean,husky,~/husky_ws/husky,https://github.com/husky/husky.git,foxy-devel))

setup_husky:
	cp $(PFE)/husky_ws/gazebo_cats.launch.py ~/husky_ws/husky/husky_gazebo/launch


define clear_package_ros
clear_$1:
	@cd $2 && echo -n "[$2] " && $(call _clear_ros)
endef

$(eval $(call clear_package_ros,ros2_shared,$(ROS2_SHARED)))
$(eval $(call clear_package_ros,tello_msgs,$(TELLO_MSGS)))
$(eval $(call clear_package_ros,FaMe,$(FAME)))

clone_FaMe_deps:	  \
	clone_ros2_shared \
	clone_tello_msgs  \


# clone_ros2_shared:
# 	@if [ ! -d "$(ROS2_SHARED)" ]; then git clone https://github.com/ptrmu/ros2_shared.git $(ROS2_SHARED); fi
# clear_ros2_shared:
# 	@cd $(ROS2_SHARED) && echo -n "[$(ROS2_SHARED)] " && $(call _clear_ros)

setup_ros2_shared:
	cd $(ROS2_SHARED) && colcon build && source install/setup.bash


# clone_tello_msgs:
# 	@if [ ! -d "$(TELLO_MSGS)" ]; then git clone https://github.com/clydemcqueen/tello_ros.git $(TELLO_MSGS); fi
# clear_tello_msgs:
# 	@cd $(TELLO_MSGS) && echo -n "[$(TELLO_MSGS)] " && $(call _clear_ros)

setup_tello_msgs: setup_ros2_shared
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(TELLO_MSGS) && nvm install --lts=gallium && nvm use 16 && \
		cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && colcon build && source install/setup.bash

setup_FaMe_agri: setup_tello_msgs install_FaMe install_node
	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
		cd $(FAME_AGRI) && nvm install --lts=gallium && nvm use 16 && \
		cd $(ROS2_SHARED) && source install/setup.bash && \
		cd $(TELLO_MSGS) && source install/setup.bash && \
		cd $(FAME_AGRI) && colcon build
	cd $(FAME_AGRI) && source install/setup.bash && source /usr/share/gazebo/setup.bash

#deprecated
copy_models_FaMe_agri:
	mkdir -p $(GZ_MODEL_DIR)
	cp -R $(FAME_AGRI)/models/* $(GZ_MODEL_DIR)

# clear_fame_agri:
# 	@cd $(FAME_AGRI) && echo -n "[$(FAME_AGRI)] " && $(call _clear_ros)

# /====================================\
# |               Gazebo               |
# \====================================/

launch_gazebo_2004:
	cd $(ROS2_SHARED) && source install/setup.bash && 		\
		cd $(TELLO_MSGS) && source install/setup.bash && 	\
		cd $(FAME_AGRI) && source install/setup.bash && 	\
		source /usr/share/gazebo/setup.bash && 				\
		# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 			\
		ros2 launch fame_agricultural multi_launch.py 		
# 			--ros-args -r gazebo_ros_force:=blade_force


# /====================================\
# |                FaMe                |
# \====================================/

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

launch_fame_modeler:
	cd ./fame-modeler && npm start


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

$(eval $(call clean_package,pfe_simulation_gazebo,$(PATH_TELLO_WS)))

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
	if [ ! -f "$(PWD)/GitHubDesktop-linux-3.1.1-linux1.deb" ]; then \
		wget https://github.com/shiftkey/desktop/releases/download/release-3.1.1-linux1/GitHubDesktop-linux-3.1.1-linux1.deb;
	fi
	sudo apt-get update
	sudo apt-get install gdebi-core -y
	sudo gdebi GitHubDesktop-linux-3.1.1-linux1.deb -y
	sudo dpkg -i GitHubDesktop-linux-3.1.1-linux1.deb 
	sudo apt-get install -f -y
# 	sudo apt-mark hold github-desktop


test_python_versions:
	python3 --version || true
	python3.10 --version || true
	node -v # Bonus

setup_gazebo_models_2004:
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
	copy_simu_gazebo_to_Github		\
	copy_makefile_to_Github			\
	copy_bashrc_to_Github			\
	copy_code_setup_to_Github		\
	copy_gazebo_models_to_Github	\

copy_from_github:					\
	check_with_user					\
	check_with_user_first_time		\
	copy_simu_gazebo_from_Github	\
	copy_code_setup_from_Github		\
	copy_makefile_from_Github

define github
copy_$1_to_Github:
	if [ -d $2 ] || [ -f $2 ]; then cp -r $2 $3; fi
copy_$1_from_Github: check_with_user
# 	@echo "$2"
	@if [ -f $2 ]; then echo "file $2"; cp -r $3 $2 ; fi
	@if [ ! -f $2 ] ; then mkdir -p $(dir ${2:/=}) ; fi
	@if [ -d $2 ] ; then echo "dic $2 $(dir ${2:/=})"; cp -r $3/* $(dir ${2:/=}); fi
clean_$1:	
	rm -r $2
endef

# Don't forget to let a folder of space while copying a folder
PATH_PFE:=~/PFE
$(eval $(call github,simu_gazebo,~/Simulation_Gazebo/tello_ros_ws/,${PATH_PFE}/Simulation_Gazebo_new/))
$(eval $(call github,makefile,~/Makefile,${PATH_PFE}/Makefile))
$(eval $(call github,bashrc,~/.bashrc,${PATH_PFE}/.bashrc))
$(eval $(call github,code_setup,~/.config/Code/User/,${PATH_PFE}/Code/))
$(eval $(call github,gazebo_models,~/.gazebo/models,${PATH_PFE}/models))
# $(eval $(call github,,,))

