SHELL := /bin/bash

.PHONY: all install_all install_terminator install_vscode_2004 correct_vscode_2004 install_ros2_foxy install_gazebo_2004 install_python_3_10 \
        test_ros2 test_gazebo versions install_FaMe_modeler run_FaMe_modeler install_nvm install_node install_cmake \
		install_discord_snap install_deps_python clone_ros2_shared setup_ros2_shared clone_tello_msgs setup_tello_msgs install_FaMe \
		setup_FaMe_agricultural setup_models_FaMe_agri setup_gazebo launch_gazebo_2004 install_FaMe_engine launch_comportement \
		setup_FaMe_simulation install_github_desktop_2004 min_install_2004 install_github_desktop_2404 min_install_2404

%:
	@:


# /====================================\
# |          Paths & Variables         |
# \====================================/

HOME_DIR := $(PWD)
ROS2_SHARED := $(HOME_DIR)/ros2_shared
TELLO_MSGS := $(HOME_DIR)/tello_msgs
FAME := /home/dell/Documents/GitHub/my_FaMe
# FAME := $(HOME_DIR)/fame
FAME_AGRI := $(FAME)/fame_agricultural
FAME_ENGINE := $(FAME)/fame_engine
FAME_SIMU := $(FAME)/fame_simulation
GZ_MODEL_DIR := $(HOME_DIR)/.gazebo/models # might need to be $(HOME) and not $(HOME_DIR)
MBROS_DIR := /home/ubuntu/mbros/fame_engine/process
HUSKY_WS := $(HOME)/husky_ws
HUSKY := $(HUSKY_WS)/husky
SIMU_GAZEBO := ~/Simulation_Gazebo/tello_ros_ws


MBROS_DIR      := /home/ubuntu/mbros/fame_engine
NVM_SCRIPT     := $$HOME/.nvm/nvm.sh          # ≠ variable d’env. de nvm
NODE_VERSION   := 16                          # LTS Gallium (ABI 93)
NPM_VERSION := 16

DELAY ?= 20

PATH_TELLO_WS=$(HOME)/Simulation_Gazebo/tello_ros_ws
PATH_TELLO_WS_OLD=$(PFE)/Simulation_Gazebo_old/tello_ros_ws
PATH_TELLO_WS_SW=$(PFE)/Simulation_Gazebo_SW/tello_ros_ws

PATH_PFE:=$(HOME)/PFE
PFE:=$(HOME)/PFE


# /====================================\
# |            Random Macro            |
# \====================================/

# Default target
all: min_install_2004

# meh
clean:
	make -i try_clean

try_clean: 						\
	clear_ros2_shared 			\
	clear_tello_msgs 			\
	clear_fame_agri				\
	clear_ros2_FaMe_engine		\
	clear_pfe_simulation_gazebo	

min_install_2004: 				\
	sudo_upgrade				\
	install_github_desktop_2004	\
	install_python_3_10			\
	install_software_2004		\
	install_software
	@echo "After clonning you need to execute 'make copy_from_github'"

setup_2404:						\
	sudo_upgrade				\
	install_github_desktop_2404	\
	install_deps_2404			\
	install_software_2404		\
	install_software
	@echo "After clonning you need to execute 'make copy_from_github'"

setup_deps:
	echo "WIP"

setup_deps_24_04:
	${update}
	${install} \
		ros-$$ROS_DISTRO-rclcpp-components \
		ros-$$ROS_DISTRO-cv-bridge \
		ros-$$ROS_DISTRO-image-transport \
		ros-$$ROS_DISTRO-camera-info-manager \
		libopencv-dev \
		libasio-dev


# /====================================\
# |           Macro  install           |
# \====================================/

#TODO: redo those macros as they are almost all deprecated and may not work completely
	

# Works on 2004 and 2404
install_software:			\
	sudo_upgrade			\
	install_discord_snap	\
	install_terminator		\
	install_FaMe_modeler	\

install_software_2004:
	install_vscode_2004		\
	correct_vscode_2004		\
	install_gazebo_2004		\
	install_ros2_foxy		

install_software_2404: \
	install_code_2404		

# deprecated use github
install_deps_2404:	\
	install_git		
	
# deprecated use github
install_software_2004_old:	\
	sudo_upgrade			\
	install_terminator		\
	install_cmake			\
	install_discord_snap	\
	install_vscode_2004		\
	correct_vscode_2004		\
	install_FaMe_modeler	\
	install_deps_python


# deprecated use github
install_software_2004_bis:	\
	install_gazebo_2004		\
	install_ros2_foxy		

# deprecated use github
install_all: 			\
	install_software	\
	install_all2

# deprecated use github
install_all2: 					\
	clone_ros2_shared 			\
	setup_ros2_shared			\
	clone_tello_msgs 			\
	setup_tello_msgs			\
	install_FaMe 				\
	setup_FaMe_agricultural 	\
	setup_models_FaMe_agri		\
	setup_gazebo 				\
	install_FaMe_engine 		\
	setup_FaMe_simulation 		\
	install_github_desktop_2004 \
	setup_bashrc 				\
	setup_pfe_simulation_gazebo

# deprecated use github
install_all_2404:	\
	install_node

install_dependencies:	\
	install_node		\
	



# /====================================\
# |           Install macros           |
# \====================================/

.PHONY: sudo_update sud sudo_upgrade sug sudg

define install
	sudo apt install -y
endef
define update
	sudo apt update
endef
define upgrade
	sudo apt upgrade -y
endef

define update_upgrade
	sudo apt update
	sudo apt upgrade -y
endef

update_upgrade:
	$(update_upgrade)

update: sudo_upgrade

sudo_update:
	sudo apt update
sud: sudo_update

sudo_upgrade:
	sudo apt upgrade -y
sug: sudo_upgrade

sudg: sudo_update sudo_upgrade

# i_%:
# 	sudo apt install -y $*
# install_%:
# 	sudo apt install -y $<

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

update_source:
	source ~/.bashrc


refresh_env:
	@echo "Sourcing .bashrc..."
	source ~/.bashrc 
	@echo "Environment refreshed"

# /====================================\
# |         install  softwares         |
# \====================================/


install_git:
	$(install) git

install_snap:
	$(update)
	$(install) snapd

install_cmake: install_snap
	sudo snap install cmake --classic

install_nvm: update_source
	curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
	@echo "NVM installation complete."

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
	$(update)
	$(install) terminator

install_discord_snap: install_snap
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
	$(update)
	$(install) software-properties-common
	sudo add-apt-repository -y ppa:deadsnakes/ppa
	$(update)
	$(install) python3.10 python3.10-venv python3.10-dev
	curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10
	$(pip) --version
	$(pip) install pyparrot djitellopy

install_deps_python:
# sudo apt update
	$(install) python3-pip
	$(pip) install transformations djitellopy

# /====================================\
# |       install softwares  2004      |
# \====================================/

install_vscode_2004:
	$(update)
	$(install) wget gpg apt-transport-https
	wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
	sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
	echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null
	rm -f packages.microsoft.gpg
	$(update)
	$(install) code # or code-insiders

correct_vscode_2004:
	sudo rm -f /etc/apt/sources.list.d/vscode.list
	sudo rm -f /etc/apt/sources.list.d/vscode.sources
	sudo rm -f /usr/share/keyrings/microsoft.gpg
	sudo rm -f /etc/apt/keyrings/packages.microsoft.gpg
	
	sudo mkdir -p /etc/apt/keyrings
	wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/keyrings/microsoft.gpg > /dev/null

	echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | \
	sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

	$(update)
	$(install) code


	# sudo apt install wget gpg
	# wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
	# sudo install -o root -g root -m 644 microsoft.gpg /usr/share/keyrings/
	# sudo sh -c 'echo "deb [arch=amd64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
	# sudo apt update


install_ros2_foxy: install_cmake
	@echo "Bienvenu dans l'installation de ROS2 Foxy"
	locale || true                               # check current locale (non-fatal)
	$(update)
	$(install) locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8
	locale || true                               # verify settings
	$(install) software-properties-common curl
	sudo add-apt-repository -y universe
	sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
	echo "deb [arch=$$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $$(. /etc/os-release && echo $$UBUNTU_CODENAME) main" | \
		sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
	$(update_upgrade)
	$(install) ros-foxy-desktop python3-argcomplete ros-dev-tools
# 	Add ROS 2 environment setup to bashrc only once
	grep -qxF "# ROS 2 Foxy" $$HOME/.bashrc || ( \
		echo "" >> $$HOME/.bashrc && \
		echo "# ROS 2 Foxy" >> $$HOME/.bashrc && \
		echo "source /opt/ros/foxy/setup.bash" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)
	$(install) ros-foxy-nav2-bringup
# 	deps for husky
	$(install) ros-foxy-xacro
	$(install) ros-foxy-controller-interface ros-foxy-ros2-control ros-foxy-ros2-controllers
	$(install) \
		ros-foxy-robot-localization \
		ros-foxy-gazebo-ros-pkgs \
		ros-foxy-ros2-control \
		ros-foxy-ros2-controllers \
		ros-foxy-controller-manager \
		ros-foxy-controller-manager-msgs
	$(install) ros-foxy-interactive-marker-twist-server ros-foxy-interactive-markers
	$(install) ros-foxy-twist-mux


install_gazebo_2004:
	$(update)
	$(install) ros-foxy-gazebo-ros-pkgs
# deps Gazebo
	$(install) libasio-dev


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


# /====================================\
# |       install softwares  2404      |
# \====================================/

install_vscode_2404:
	$(install) code

setup_ros2_jazzy:
	@echo "Bienvenu dans le setup de ROS2 Jazzy"
	locale  # check for UTF-8

	$(update) && $(install)locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

	locale  # verify settings

	$(install) software-properties-common
	sudo add-apt-repository universe

	$(update) && $(install) curl
	export ROS_APT_SOURCE_VERSION=$$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$${ROS_APT_SOURCE_VERSION}/ros2-apt-source_$${ROS_APT_SOURCE_VERSION}.$$(. /etc/os-release && echo $$VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $$UBUNTU_CODENAME
	sudo dpkg -i /tmp/ros2-apt-source.deb

install_ros2_jazzy: setup_ros2_jazzy
	@echo "Bienvenu dans l'installation de ROS2 Jazzy"
	$(install) ros-jazzy-desktop

	grep -qxF "# ROS 2 Jazzy" $$HOME/.bashrc || ( \
		echo "" >> $$HOME/.bashrc && \
		echo "# ROS 2 Jazzy" >> $$HOME/.bashrc && \
		echo "source /opt/ros/jazzy/setup.bash" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)

.PHONY: install_ros2_jazzy_bis
install_ros2_jazzy_bis: setup_ros2_jazzy i_ros-jazzy-desktop
	@echo "Bienvenu dans l'installation de ROS2 Jazzy"
# 	sudo apt install ros-jazzy-desktop

	grep -qxF "# ROS 2 Jazzy" $$HOME/.bashrc || ( \
		echo "" >> $$HOME/.bashrc && \
		echo "# ROS 2 Jazzy" >> $$HOME/.bashrc && \
		echo "source /opt/ros/jazzy/setup.bash" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc \
	)

install_gazebo_2404:
	$(update)
	$(install) curl lsb-release gnupg

	sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	$(update)
	$(install) gz-harmonic ros-jazzy-ros-gz

.PHONY: install_gazebo_2404_bis
install_gazebo_2404_bis: i_curl i_lsb-release i_gnupg i_gz-harmonic i_ros-jazzy-ros-gz
	sudo apt-get update
# 	sudo apt-get install curl lsb-release gnupg

	sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	sudo apt-get update
# 	sudo apt-get install gz-harmonic

# 	sudo apt-get install ros-jazzy-ros-gz

.PHONY: install_gazebo_2404_ter
install_gazebo_2404_ter: 
	sudo apt-get update
	sudo apt-get install curl lsb-release gnupg

	sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
	echo "deb [arch=$$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
	sudo apt-get update
	sudo apt-get install gz-harmonic

	sudo apt-get install ros-jazzy-ros-gz

install_github_desktop_2404:
	if [ ! -f "$(PWD)/GitHubDesktop-linux-3.1.1-linux1.deb" ]; then \
		wget https://github.com/shiftkey/desktop/releases/download/release-3.1.1-linux1/GitHubDesktop-linux-3.1.1-linux1.deb;
	fi
	$(update)
	$(install) gdebi-core -y
	sudo gdebi GitHubDesktop-linux-3.1.1-linux1.deb -y
	sudo dpkg -i GitHubDesktop-linux-3.1.1-linux1.deb 
	$(install) -f
# 	sudo apt-mark hold github-desktop

install_github_desktop_2404_bis: i_gdebi-core
	if [ ! -f "$(PWD)/GitHubDesktop-linux-3.1.1-linux1.deb" ]; then \
		wget https://github.com/shiftkey/desktop/releases/download/release-3.1.1-linux1/GitHubDesktop-linux-3.1.1-linux1.deb;
	fi
	sudo apt-get update
# 	sudo apt-get install gdebi-core -y
	sudo gdebi GitHubDesktop-linux-3.1.1-linux1.deb -y
	sudo dpkg -i GitHubDesktop-linux-3.1.1-linux1.deb 
	sudo apt-get install -f -y
# 	sudo apt-mark hold github-desktop

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
		echo "alias ros-build-sym-ver="colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON && source install/setup.bash\"" >> $$HOME/.bashrc && \
		echo "alias ros-build-sym-pac-ver='temp(){ colcon build --packages-select \"\$1\" --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON && source install/setup.bash; unset -temp temp; }; temp'\"" >> $$HOME/.bashrc && \
		echo "alias ros-sc=\"source install/setup.bash\"" >> $$HOME/.bashrc && 															\
		echo "alias sc-ros=\"source install/setup.bash\"" >> $$HOME/.bashrc && 															\
		echo "alias bash-sc=\"source ~/.bashrc\"" >> $$HOME/.bashrc && 																	\
		echo "alias my-sc=\"source $(TELLO_MSGS)/install/setup.bash && source $(ROS2_SHARED)/install/setup.bash\"" >> $$HOME/.bashrc && \
		echo "" >> $$HOME/.bashrc && 																									\
		echo "this-sc\(\) {" >> $$HOME/.bashrc && 																						\
		echo "    cd \"$1\" && source install/setup.bash && cd -> /dev/null 2>&1 " >> $$HOME/.bashrc && 								\
		echo "}" >> $$HOME/.bashrc && 																									\
		echo "" >> $$HOME/.bashrc 																										
	)

.PHONY: setup_bashrc_GPT
.ONESHELL:setup_bashrc_GPT
setup_bashrc_GPT:
	set -e
	BRC="$$HOME/.bashrc"
	# Sauvegarde une fois
	[ -f "$$BRC.bak" ] || cp "$$BRC" "$$BRC.bak"
	# Retire l'ancien bloc (s'il existe)
	sed -i '/^# >>> CATS Custom commands >>>/,/^# <<< CATS Custom commands <<</d' "$$BRC"
	# Ajoute le nouveau bloc
	cat >> "$$BRC" <<'EOF'

	# >>> CATS Custom commands >>>

	# Custom commands
	alias ros-build="colcon build && source install/setup.bash"
	alias ros-build-sym="colcon build --symlink-install && source install/setup.bash"
	alias ros-build-sym-ver='colcon build --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON && source install/setup.bash'
	alias ros-build-sym-pac-ver='temp(){ colcon build --packages-select "$1" --symlink-install --event-handlers console_cohesion+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON && source install/setup.bash; unset -f temp; }; temp'
	alias ros-sc="source install/setup.bash"
	alias sc-ros="source install/setup.bash"
	alias bash-sc="source ~/.bashrc"
	alias my-sc="source ${TELLO_MSGS}/install/setup.bash && source ${ROS2_SHARED}/install/setup.bash"

	this-sc() {
	    cd "$1" && source install/setup.bash && cd - >/dev/null 2>&1
	}

	# <<< CATS Custom commands <<<
	EOF
	@echo "✔ Bloc 'Custom commands' mis à jour dans $$HOME/.bashrc"



# /====================================\
# |          random & unsorted         |
# \====================================/

test_nvm_install: refresh_env
#	command -v nvm
	source ~/.bashrc && nvm -v


test_ros2:
	@echo "Terminal 1 ➜ source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_cpp talker"
	@echo "Terminal 2 ➜ source /opt/ros/foxy/setup.bash && ros2 run demo_nodes_py listener"

# Gazebo + Twist publisher demo

test_gazebo:
	@echo "Terminal 1 ➜ gazebo --verbose /opt/ros/foxy/share/gazebo_plugins/worlds/gazebo_ros_diff_drive_demo.world"
	@echo "Terminal 2 ➜ ros2 topic pub /demo/cmd_demo geometry_msgs/Twist '{linear: {x: 1.0}}' -1"

test_python_versions:
	python3 --version || true
	python3.10 --version || true
	node -v # Bonus

setup_gazebo_models_2004:
	@echo "for the moment unable to find where does the coke can belongs from"
	@echo "so, for the moment, you need to copy the \`.gazebo/models\` folder to you working space"

kill_all:
	killall -9 gzserver
	killall -9 gzclient


print_supported_version:
	@if [ "$(shell lsb_release -is)" = "Ubuntu" ] & [ "$(shell lsb_release -rs)" = "24.04" ] ; then
		echo "curent version is Ubuntu 24.04" 
	elif [ "$(shell lsb_release -is)" = "Ubuntu" ] & [ "$(shell lsb_release -rs)" = "20.04" ] ; then
		echo "curent version is Ubuntu 20.04" 
	else 
		echo "unsuported version : \"$(shell lsb_release -a)\"" 
	fi

# /====================================\
# |            package  deps           |
# \====================================/

define from_git_clean
.PHONY: clone_$1 clean_$1
clone_$1:
	@-if [ ! -f $2 ] ; then echo "mkdir -p $2";  mkdir -p $2 ; fi
	@-if [ -d $2 ] ; then echo -n "git clone $3 $2 -b $4" && git clone $3 $2 -b $4 ; fi
clean_$1: check_with_user
	sudo rm -r $2
endef

$(eval $(call from_git_clean,ros2_shared,$(ROS2_SHARED),https://github.com/ptrmu/ros2_shared.git,master))
$(eval $(call from_git_clean,tello_msgs,$(TELLO_MSGS),https://github.com/clydemcqueen/tello_ros.git,master))
$(eval $(call from_git_clean,FaMe_bitbucket,$(FAME),https://bitbucket.org/proslabteam/fame.git,master))
$(eval $(call from_git_clean,husky_2004,$(HUSKY),https://github.com/husky/husky.git,foxy-devel))

setup_with_git:				\
	clone_ros2_shared		\
	clone_tello_msgs		\
	clone_FaMe_bitbucket	\
	clone_husky_2004		\
	correct_git_clone

correct_git_clone:
	@if [ "$(shell lsb_release -is)" = "Ubuntu" ] & [ "$(shell lsb_release -rs)" = "24.04" ] ; then
# 		echo "curent version is Ubuntu 24.04" 
		chmod +x $(TELLO_MSGS)/tello_description/src/replace.py
		$(install) 											\
			libasio-dev										\
			ros-$$ROS_DISTRO-camera-calibration-parsers		\
			ros-$$ROS_DISTRO-camera-info-manager			\
			ros-$$ROS_DISTRO-image-transport				\
			ros-$$ROS_DISTRO-rclcpp-components				\
			ros-$$ROS_DISTRO-joy							\
			ros-$$ROS_DISTRO-rclcpp-components 				\
			ros-$$ROS_DISTRO-cv-bridge 						\
			ros-$$ROS_DISTRO-image-transport 				\
			ros-$$ROS_DISTRO-camera-info-manager 			\
			ros-$$ROS_DISTRO-sensor-msgs 					\
			ros-$$ROS_DISTRO-geometry-msgs
		@touch "$(TELLO_MSGS)/tello_gazebo/COLCON_IGNORE"
		@touch "$(TELLO_MSGS)/tello_driver/COLCON_IGNORE"
	elif [ "$(shell lsb_release -is)" = "Ubuntu" ] & [ "$(shell lsb_release -rs)" = "20.04" ] ; then
		echo "curent version is Ubuntu 20.04" 
	else 
		echo "unsuported version : \"$(shell lsb_release -a)\"" 
	fi

define setup_pkg
.PHONY: setup_$(1) clear_$(1)
setup_$(1):
	@if [ -n "$(4)" ]; then 
		echo "export NVM_DIR=\"$$$$HOME/.nvm\"" ; export NVM_DIR="$$$$HOME/.nvm"; 
		if [ -f "$$$$HOME/.nvm/nvm.sh" ]; then 
			echo "source \"$$$$HOME/.nvm/nvm.sh\"" ; . "$$$$HOME/.nvm/nvm.sh"; 
		else 
			echo "NVM introuvable (cherché: $$$$HOME/.nvm/nvm.sh). Installe NVM puis relance." >&2; 
			exit 127; 
		fi; 
	  	echo "nvm use $(NODE_VERSION)" ; nvm use $(NODE_VERSION); 
	fi; 
	for d in $(3); do 
		if [ -f "$$$$d/install/setup.bash" ]; then 
			echo "source \"$$$$d/install/setup.bash\"" ; . "$$$$d/install/setup.bash"; 
		fi; 
	done; 
	echo "cd $(2)" ; cd $(2);

	@if [ "$(5)" == "build" ]; then 
		echo "colcon build" ; colcon build ;
# 	elif [[ "$(5)" == "npm" ]]; then
# 		echo "npm install" ; npm install ;
# 		echo "colcon build" ; colcon build ;
	else 
		echo "colcon build --symlink-install" ; colcon build --symlink-install ;
	fi;

clear_$(1):
	@cd $(2) && echo -n "[$(2)] " && $(call _clear_ros)
endef

$(eval $(call setup_pkg,ros2_shared,$(ROS2_SHARED),,,))
$(eval $(call setup_pkg,tello_msgs,$(TELLO_MSGS),$(ROS2_SHARED),nvm,))
$(eval $(call setup_pkg,husky,$(HUSKY),$(SIMU_GAZEBO),,))

setup_FaMe_link:
	-sudo mkdir /home/ubuntu
	-sudo mkdir /home/ubuntu/mbros
	sudo ln -sf $(FAME_ENGINE) $(MBROS_DIR)
	@echo "Link succesfully created"

# symlink -> might not be working for some reasons, need to clear before setup # to be checked as it might be rectified
$(eval $(call setup_pkg,FaMe,$(FAME),,nvm,build))
$(eval $(call setup_pkg,FaMe_engine,$(FAME_ENGINE),,nvm,))
$(eval $(call setup_pkg,FaMe_agricultural,$(FAME_AGRI),$(FAME_ENGINE),nvm,build)) # symlink -> not working
$(eval $(call setup_pkg,FaMe_simulation,$(FAME_SIMU),$(FAME_ENGINE),nvm,build)) # to check

# setup_FaMe_simulation:
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && colcon build

setup_husky_launch:
	cp $(PFE)/husky_ws/gazebo_cats.launch.py ~/husky_ws/husky/husky_gazebo/launch


define clear_package_ros
.PHONY: clear_$1
clear_$1:
	@cd $2 && echo -n "[$2] " && $(call _clear_ros)
endef

# $(eval $(call clear_package_ros,ros2_shared,$(ROS2_SHARED)))
# $(eval $(call clear_package_ros,tello_msgs,$(TELLO_MSGS)))
# $(eval $(call clear_package_ros,FaMe,$(FAME)))
# $(eval $(call clear_package_ros,FaMe_agri,$(FAME_AGRI)))
# $(eval $(call clear_package_ros,FaMe_engine,$(FAME_ENGINE)))
# $(eval $(call clear_package_ros,FaMe_simu,$(FAME_SIMU)))

# $(eval $(call clear_package_ros,pfe_simulation_gazebo,$(PATH_TELLO_WS)))
$(eval $(call clear_package_ros,simulation_gazebo,$(PATH_TELLO_WS)))
$(eval $(call clear_package_ros,pfe_simulation_gazebo_old,$(PATH_TELLO_WS_OLD)))
$(eval $(call clear_package_ros,pfe_simulation_gazebo_SW,$(PATH_TELLO_WS_SW)))


# /====================================\
# |               Gazebo               |
# \====================================/

# deprecated
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

launch_FaMe_modeler:
	cd fame-modeler && . $$HOME/.nvm/nvm.sh && npm run start &

clone_FaMe_deps:	  \
	clone_ros2_shared \
	clone_tello_msgs  

#deprecated
setup_models_FaMe_agri:
	mkdir -p $(GZ_MODEL_DIR)
	cp -R $(FAME_AGRI)/models/* $(GZ_MODEL_DIR)

# install_FaMe_engine:
# # 	sudo apt update
# # 	sudo apt install ros-foxy-rmw-cyclonedds-cpp -y
# 	sudo mkdir -p $(MBROS_DIR)
# 	sudo ln -sf $(FAME_ENGINE)/process $(MBROS_DIR)
# # 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# # 		cd $(FAME_ENGINE) && nvm install 12 && nvm use 12
# 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm install --lts=gallium && nvm use 16
# 	cd $(FAME_ENGINE) && rm -rf node_modules package-lock.json

# # 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# # 		cd $(FAME_ENGINE) && nvm install 12 && nvm use 12 && \
# # 		cd $(FAME_ENGINE) && npm install
# 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm install --lts=gallium && nvm use 16 && \
# 		cd $(FAME_ENGINE) && npm install

# 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
# 		cd $(FAME_ENGINE) && colcon build
	

# 	cd $(FAME_ENGINE) && rm -rf node_modules package-lock.json

# 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
# 		cd $(FAME_ENGINE) && npm pkg set "dependencies.rclnodejs=^0.21.0"

# 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
# 		cd $(FAME_ENGINE) && npm install

# 	cd $(FAME_ENGINE)/install/fame_engine/share/fame_engine && rm -rf node_modules/rclnodejs
# 	cd $(FAME_ENGINE)/install/fame_engine/share/fame_engine && npm install rclnodejs@^0.21.0  

# # 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
# # 		cd $(FAME_ENGINE) && npm i rclnodejs@^0.21.0  

# 	@export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && nvm use 16 && \
# 		cd $(FAME_ENGINE) && colcon build

# # 	export NODE_OPTIONS="--unhandled-rejections=strict"
# # 	ros2 launch fame_engine agri_engine.launch.py


define launch_pkg # name_fn [param_launch_ros] [ros_packages] [literals_deps] bool bool
.PHONY: launch_$(1)
launch_$(1):
	@if [ -n "$(4)" ]; then 
		echo "make --ignore-errors kill_all"; make --ignore-errors kill_all;
	fi;
	if [ -n "$(3)" ]; then
		echo "export NVM_DIR=\"$$$$HOME/.nvm\"" ; export NVM_DIR="$$$$HOME/.nvm"; 
		if [ -f "$$$$HOME/.nvm/nvm.sh" ]; then 
			echo "source \"$$$$HOME/.nvm/nvm.sh\"" ; . "$$$$HOME/.nvm/nvm.sh"; 
		else 
			echo "NVM introuvable (cherché: $$$$HOME/.nvm/nvm.sh). Installe NVM puis relance." >&2; 
			exit 127; 
		fi;
	  	echo "nvm use $(NODE_VERSION)" ; nvm use $(NODE_VERSION); 
	fi; 
	for d in $(5); do 
	  if [ -f "$$$$d/install/setup.bash" ]; then 
	    echo "source \"$$$$d/install/setup.bash\""; . "$$$$d/install/setup.bash"; 
	  fi; 
	done; 
	for rf in $(6); do 
	  if [ -f "$$$$rf" ]; then 
	    echo "source \"$$$$rf\""; . "$$$$rf"; 
	  fi; 
	done; 
	for var in $(7); do 
	  if [ -f "$$$$var" ]; then 
	    echo "export \"$$$$var\""; export "$$$$var"; 
	  fi; 
	done; 
	echo "ros2 launch $(2)" ; ros2 launch $(2) 
endef

# TODO add : ros2 launch tello_gazebo tello_synchro_launch_cats_3.py # and similar

$(eval $(call launch_pkg,FaMe_CATS,fame_engine my_CATS.py,nvm,,$(FAME_ENGINE),,))


$(eval $(call launch_pkg,FaMe_agricultural_multi,fame_agricultural multi_launch.py,nvm,kill,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))
$(eval $(call launch_pkg,FaMe_engine_agri,fame_engine agri_engine.launch.py,nvm,,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))


# Takes the first target as command
Command := $(firstword $(MAKECMDGOALS))
# Skips the first word
Arguments := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))

# hello:
# 	@echo "Hello, ${Arguments}!"
arg_command := fame_engine my_CATS.py ${Arguments}
arg_deps := $(FAME_ENGINE) $(FAME_AGRI)
arg_deps2 := /usr/share/gazebo/setup.bash
arg_export := NODE_OPTIONS="--unhandled-rejections=strict"
launch_FaMe:
# 	$(eval $(call launch_pkg,FaMe_macro,fame_engine my_CATS.py,nvm,,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))
	echo "export NVM_DIR=\"$$HOME/.nvm\"" ; export NVM_DIR="$$HOME/.nvm"; 
	if [ -f "$$HOME/.nvm/nvm.sh" ]; then 
		echo "source \"$$HOME/.nvm/nvm.sh\"" ; . "$$HOME/.nvm/nvm.sh"; 
	else 
		echo "NVM introuvable (cherché: $$HOME/.nvm/nvm.sh). Installe NVM puis relance." >&2; 
		exit 127; 
	fi;
	echo "nvm use $(NODE_VERSION)" ; nvm use $(NODE_VERSION); 
	for d in ${arg_deps}; do 
	  if [ -f "$$d/install/setup.bash" ]; then 
	    echo "source \"$$d/install/setup.bash\""; . "$$d/install/setup.bash"; 
	  fi; 
	done; 
	for rf in ${arg_deps2}; do 
	  if [ -f "$$rf" ]; then 
	    echo "source \"$$rf\""; . "$$rf"; 
	  fi; 
	done; 
	for var in ${arg_export}; do 
	  if [ -f "$$var" ]; then 
	    echo "export \"$$var\""; export "$$var"; 
	  fi; 
	done; 
	echo "ros2 launch ${arg_command}" ; ros2 launch ${arg_command} 




# #deprecated
# launch_comportement:
# 	@echo "be sure to have use 'make install_FaMe_engine' before using this command"
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm install --lts=gallium && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		ros2 launch fame_engine agri_engine.launch.py


$(eval $(call launch_pkg,FaMe_simulation_multi,fame_simulation multi_launch.py,nvm,kill,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI) $(FAME_SIMU),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))
# $(eval $(call launch_pkg,FaMe_engine_agri,fame_engine agri_engine.launch.py,nvm,,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI) $(FAME_SIMU),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))
.ONESHELL: launch_comportement_agri
launch_comportement_agri:
	@
	make launch_FaMe_simulation_multi &
	PID_SIM=$$!
	sleep $(DELAY)
	@echo "============== END OF SLEEP =============="
	make launch_FaMe_engine_agri &
	PID_ENG=$$!
	@echo "======== agri and engine LAUNCHED ========"
	wait $$PID_SIM $$PID_ENG
	@echo "========== agri and engine DONE =========="

# .ONESHELL: launch_comportement_agri
# launch_comportement_agri:
# 	make -i kill_all
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && source install/setup.bash && \
# 		ros2 launch fame_agricultural multi_launch.py &
# 	PID_SIM=$$!
# 	sleep $(DELAY)

# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && source install/setup.bash && \
# 		ros2 launch fame_engine agri_engine.launch.py
# 	PID_ENG=$$!
# 	wait $$PID_SIM $$PID_ENG
# 	@echo "======= agri and engine done ======="



# launch_example_alone:
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		ros2 launch fame_engine example.launch.py 
	
setup_example: setup_tello_msgs setup_FaMe_engine setup_FaMe_simulation setup_FaMe_agricultural

# $(eval $(call launch_pkg,FaMe_simulation_multi,fame_simulation multi_launch.py,nvm,kill,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI) $(FAME_SIMU),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))
$(eval $(call launch_pkg,FaMe_engine_example,fame_engine example.launch.py,nvm,,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI) $(FAME_SIMU),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))
.ONESHELL: launch_example
launch_example:
	@
	make launch_FaMe_simulation_multi &
	PID_SIM=$$!
	sleep $(DELAY)
	@echo "============== END OF SLEEP =============="
	make launch_FaMe_engine_example &
	PID_ENG=$$!
	@echo "======== simu and engine LAUNCHED ========"
	wait $$PID_SIM $$PID_ENG
	@echo "========== simu and engine DONE =========="
# launch_example:
# 	( make launch_FaMe_simulation_multi & ) ; \
# 	sleep $(DELAY) ; \
# 	@echo "============== END OF SLEEP ==============" ; \
# 	( make launch_FaMe_engine_example & ) ; \
# 	wait || true
# 	@echo "======== simu and engine launched ========"



# .ONESHELL: launch_example
# launch_example:
# 	make -i kill_all
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && source install/setup.bash && \
# 		ros2 launch fame_simulation multi_launch.py &
# 	PID_SIM=$$!
# 	sleep $(DELAY)

# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && source install/setup.bash && \
# 		ros2 launch fame_engine example.launch.py
# 	PID_ENG=$$!
# 	wait $$PID_SIM $$PID_ENG
# 	@echo "======= simu and engine done ======="

launch_fame_modeler:
	cd ./fame-modeler && npm start

$(eval $(call setup_pkg,pfe_simulation_gazebo,$(PATH_TELLO_WS),$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_SIMU),nvm,)) # to check

# setup_pfe_simulation_gazebo:
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && source install/setup.bash && \
# 		cd $(PATH_TELLO_WS) && \
# 		colcon build

$(eval $(call launch_pkg,pfe_simulation_gazebo,tello_gazebo someaze.py,nvm,kill,$(ROS2_SHARED) $(TELLO_MSGS) $(FAME_ENGINE) $(FAME_AGRI) $(FAME_SIMU) $(PATH_TELLO_WS),/usr/share/gazebo/setup.bash,NODE_OPTIONS="--unhandled-rejections=strict"))

# launch_pfe_simulation_gazebo:
# 	make -i kill_all
# 	cd $(ROS2_SHARED) && source install/setup.bash && \
# 		cd $(TELLO_MSGS) && source install/setup.bash && \
# 		source /usr/share/gazebo/setup.bash && \
# 		export NVM_DIR="$$HOME/.nvm" && . $$NVM_DIR/nvm.sh && \
# 		cd $(FAME_ENGINE) && nvm use 16 && \
# 		export NODE_OPTIONS="--unhandled-rejections=strict" && \
# 		cd $(FAME_AGRI) && source install/setup.bash && \
# 		cd $(FAME_ENGINE) && source install/setup.bash && \
# 		cd $(FAME_SIMU) && source install/setup.bash && \
# 		cd $(PATH_TELLO_WS) && source install/setup.bash && \
# 		ros2 launch tello_gazebo someaze.py


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
	copy_simu_gazebo_to_github		\
	copy_makefile_to_github			\
	copy_bashrc_to_github			\
	copy_code_setup_to_github		\
	copy_gazebo_models_to_github	\
	copy_FaMe_to_github				\
	copy_clearpath_to_github		\
	copy_clearpath_ws_to_github

copy_from_github:					\
	check_with_user					\
	check_with_user_first_time		\
	copy_simu_gazebo_from_github	\
	copy_code_setup_from_github		\
	copy_gazebo_models_from_github	\
	copy_FaMe_from_github			\
	copy_makefile_from_github		\
	copy_clearpath_from_github		\
	copy_clearpath_ws_from_github

define github
.PHONY: copy_$(1)_to_github copy_$(1)_from_github clean_$(1)

copy_$(1)_to_github:
	@set -e; \
	src="$(2)"; dst="$(3)"; \
	case "$$$$src" in "~/"*) src="$$$$HOME/$$$${src#~/}";; "~") src="$$$$HOME";; esac; \
	case "$$$$dst" in "~/"*) dst="$$$$HOME/$$$${dst#~/}";; "~") dst="$$$$HOME";; esac; \

	if [ ! -e "$$$$src" ]; then echo "skip: $$$$src is missing"; exit 0; fi; \
	if [ -d "$$$$src" ]; then \
		mkdir -p "$$$$dst"; \
		echo "rsync -a --delete $$$$src/ $$$$dst/"; \
		sudo rsync -a --delete "$$$$src"/ "$$$$dst"/; \
		if [ -d "$$$$dst/.git" ]; then \
			echo "rm -r $$$$dst/.git"; \
			sudo rm -r $$$$dst/.git; \
		fi; \
	else \
		mkdir -p "$$$$(dirname "$$$$dst")"; \
		echo "install -m 0644 $$$$src $$$$dst"; \
		sudo install -m 0644 "$$$$src" "$$$$dst"; \
		if [ -d "$$$$dst/.git" ]; then \
			echo "rm -r $$$$dst/.git"; \
			sudo rm -r $$$$dst/.git; \
		fi; \
	fi

copy_$(1)_from_github: check_with_user
	@set -e; \
	dst="$(2)"; src="$(3)"; \
	case "$$$$src" in "~/"*) src="$$$$HOME/$$$${src#~/}";; "~") src="$$$$HOME";; esac; \
	case "$$$$dst" in "~/"*) dst="$$$$HOME/$$$${dst#~/}";; "~") dst="$$$$HOME";; esac; \
	if [ ! -e "$$$$src" ]; then echo "error: $$$$src does not exist"; exit 1; fi; \
	if [ -d "$$$$src" ]; then \
		# On veut synchroniser un DOSSIER vers un DOSSIER
		mkdir -p "$$$$dst"; \
    	command -v rsync >/dev/null 2>&1 || { echo "error: rsync not found"; exit 127; }; \
		echo "rsync -a --delete $$$$src/ $$$$dst/"; \
		sudo rsync -a --delete "$$$$src"/ "$$$$dst"/; \
	else \
		# Si la destination est un répertoire alors qu'on attend un fichier, on stoppe :
		if [ -d "$$$$dst" ]; then \
			echo "error: destination $$$$dst is a directory but src is a file (expected a file path like $$$$dst)"; \
			echo "fix: remove or rename $$$$dst (e.g. mv $$$$dst $$$$dst.dir)"; \
			exit 2; \
		fi; \
		mkdir -p "$$$$(dirname "$$$$dst")"; \
		echo "install -m 0644 $$$$src $$$$dst"; \
		sudo install -m 0644 "$$$$src" "$$$$dst"; \
	fi

clean_$(1): check_with_user
	@sudo rm -rf "$(2)"
endef

# Don't forget to let a folder of space while copying a folder
$(eval $(call github,simu_gazebo,$(HOME)/Simulation_Gazebo/tello_ros_ws/,${PATH_PFE}/Simulation_Gazebo_new/))
$(eval $(call github,makefile,$(HOME)/Makefile,${PATH_PFE}/Makefile))
$(eval $(call github,bashrc,$(HOME)/.bashrc,${PATH_PFE}/.bashrc))
$(eval $(call github,code_setup,$(HOME)/.config/Code/User/,${PATH_PFE}/Code/User))
$(eval $(call github,gazebo_models,$(HOME)/.gazebo/models,${PATH_PFE}/models))
$(eval $(call github,FaMe,$(FAME)/,${PATH_PFE}/fame))
$(eval $(call github,husky,$(HOME)/husky_ws/,${PATH_PFE}/husky_ws))
$(eval $(call github,clearpath,$(HOME)/clearpath/,${PATH_PFE}/clearpath))
$(eval $(call github,clearpath_ws,$(HOME)/clearpath_ws/,${PATH_PFE}/clearpath_ws))
$(eval $(call github,tello_msgs,$(TELLO_MSGS)/,${PATH_PFE}/tello_msgs))
# $(eval $(call github,,,))


