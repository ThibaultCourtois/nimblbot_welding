NEXTCLOUD_SHARE_LINK = https://files.nimbl-bot.com/s/eASNJt3BtyMeADS/download
DEB_FILE = rpclib_2.3.0-1_amd64.deb

ROBOT_CONFIGS = https://gitlab.nimbl-bot.com/tcourtois/nimblbot-welding/-/raw/main/welding_robot_configurations/nb120_3m_welding.yaml/download \
								https://gitlab.nimbl-bot.com/tcourtois/nimblbot-welding/-/raw/main/welding_robot_configurations/nb55_v7_welding.yaml/download

ROBOT_CONFIG_DIR = $(or $(NB_ROBOTS_CONFIGS),$(HOME)/NimblBot/nimblbot-robots/conf)

CONFLICTING_PACKAGES = ros-iron-control-msgs ros-iron-realtime-tools

.ONESHELL:

.PHONY: install
install: check-ros manual-deps deps robot-configs repos remove-conflicting-packages build
	@echo "Installation succeed"
	@echo "Please run the following command to update your environment:"
	@echo "source ~/.zshrc"
	@echo "You will then be able to use Nimblbot Welding :) !"

.PHONY: check-ros
check-ros:
	@echo "ROS2 verification ..."
	@if ! command -v colcon >/dev/null 2>&1 || ! command -v rosdep >/dev/null 2>&1; then \
			echo "Warning, before installing this repository, you need to complete the ROS2 installation"; \
			echo "Link to ROS2 iron installation : https://docs.ros.org/en/iron/Installation.html"
			exit 1; \
	fi

.PHONY: manual-deps
manual-deps:
	@echo "Installing ROS2 Iron dependencies manually..."
	@sudo apt update
  
	#Prerequities
	@echo "Installing prerquities ..."
	@sudo apt install -y \
	  git  \
		wget \

	# Core build tools
	@echo "Installing core build tools..."
	@sudo apt install -y \
		ros-iron-ament-cmake \
		ros-iron-rosidl-default-generators \
		ros-iron-rosidl-default-runtime \
		|| echo "Some core build tools failed to install"
	
	# ROS2 Core packages
	@echo "Installing ROS2 core packages..."
	@sudo apt install -y \
		ros-iron-rclcpp \
		ros-iron-rclcpp-lifecycle \
		ros-iron-rclcpp-action \
		ros-iron-rclpy \
		ros-iron-rcutils \
		ros-iron-rcl-interfaces \
		|| echo "Some ROS2 core packages failed to install"
	
	# Standard message packages
	@echo "Installing message packages..."
	@sudo apt install -y \
		ros-iron-std-msgs \
		ros-iron-std-srvs \
		ros-iron-builtin-interfaces \
		ros-iron-geometry-msgs \
		ros-iron-sensor-msgs \
		ros-iron-trajectory-msgs \
		ros-iron-visualization-msgs \
		ros-iron-lifecycle-msgs \
		ros-iron-test-msgs \
		|| echo "Some message packages failed to install"
	
	# TF2 packages
	@echo "Installing TF2 packages..."
	@sudo apt install -y \
		ros-iron-tf2 \
		ros-iron-tf2-ros \
		ros-iron-tf2-geometry-msgs \
		|| echo "Some TF2 packages failed to install"
	
	# Control and hardware interface packages
	@echo "Installing control and hardware packages..."
	@sudo apt install -y \
		ros-iron-controller-interface \
		ros-iron-hardware-interface \
		ros-iron-controller-manager \
		ros-iron-hardware-interface-testing \
		ros-iron-ros2-control-test-assets \
		ros-iron-generate-parameter-library \
		|| echo "Some control packages failed to install"
	
	# MoveIt packages
	@echo "Installing MoveIt packages..."
	@sudo apt install -y \
		ros-iron-moveit-msgs \
		ros-iron-moveit-core \
		ros-iron-moveit-ros-planning \
		ros-iron-moveit-ros-planning-interface \
		ros-iron-moveit-common \
		ros-iron-moveit-planners \
		ros-iron-moveit-ros-move-group \
		ros-iron-moveit-kinematics \
		ros-iron-moveit-servo \
		|| echo "Some MoveIt packages failed to install"
	
	# RViz packages
	@echo "Installing RViz packages..."
	@sudo apt install -y \
		ros-iron-rviz-common \
		ros-iron-pluginlib \
		|| echo "RViz packages failed to install"
	
	# System libraries
	@echo "Installing system dependencies..."
	@sudo apt install -y \
		libboost-dev \
		libcap-dev \
		qtbase5-dev \
		python3-numpy \
		python3-scipy \
		python3-pytest \
		|| echo "Some system libraries failed to install"
	
	# Testing and linting tools
	@echo "Installing testing and linting tools..."
	@sudo apt install -y \
		ros-iron-ament-cmake-gmock \
		ros-iron-ament-lint-auto \
		ros-iron-ament-lint-common \
		ros-iron-ament-copyright \
		ros-iron-ament-flake8 \
		ros-iron-ament-pep257 \
		|| echo "Some testing tools failed to install"
	
	# Additional Python dependencies for tf_transformations
	@echo "Installing additional Python dependencies..."
	@pip3 install tf-transformations || echo "tf-transformations installation failed"
	
	@echo "Manual dependencies installation completed!"
	@echo "Note: 'nimblpy' is a custom dependency and needs to be installed separately"

.PHONY: deps
deps: $(DEB_FILE)
	@echo "RPC C++ library installation ..."
	@if sudo dpkg -i $(DEB_FILE); then \
		echo "Library installation succeed"; \
	else \
		echo "Error during C++ library installation"; \
		exit 1; \
	fi
	@sudo apt-get install -f

$(DEB_FILE):
	@echo "Downloading $(DEB_FILE) ..."
	@if wget -q "$(NEXTCLOUD_SHARE_LINK)" -O $(DEB_FILE); then \
		echo "Downloading succeed"; \
	else \
		echo "Downloading failed"; \
		exit 1; \
	fi

.PHONY: robot-configs
robot-configs:
	@if [ -z "$(NB_ROBOTS_CONFIGS)" ]; then \
		echo "NB_ROBOTS_CONFIGS not set, using default: $(ROBOT_CONFIG_DIR)"; \
		echo "Consider adding 'export NB_ROBOTS_CONFIGS=~/NimblBot/nimblbot-robots/conf' to your ~/.zshrc"; \
	else \
		echo "Using NB_ROBOTS_CONFIGS: $(ROBOT_CONFIG_DIR)"; \
	fi
	@if [ ! -d "$(ROBOT_CONFIG_DIR)" ]; then \
		echo "Directory $(ROBOT_CONFIG_DIR) does not exist!"; \
		echo "Please create it first or set NB_ROBOTS_CONFIGS properly."; \
		exit 1; \
	fi
	@echo "Downloading robot configuration files to $(ROBOT_CONFIG_DIR)..."
	@for config in $(ROBOT_CONFIGS); do \
		filename=$$(echo $$config | cut -d: -f1); \
		url=$$(echo $$config | cut -d: -f2-); \
		echo "Downloading $$filename..."; \
		if wget -q "$$url" -O $(ROBOT_CONFIG_DIR)/$$filename; then \
			echo "$$filename downloaded successfully"; \
		else \
			echo "Failed to download $$filename"; \
			exit 1; \
		fi; \
	done
	@echo "All robot configurations downloaded"

.PHONY: repos
repos: control_msgs realtime_tools gpio_controllers

control_msgs:
	@if [ ! -d "../control_msgs" ]; then \
		echo "Cloning control_msgs (master branch) ..."; \
		git clone -b master https://gitlab.nimbl-bot.com/tcourtois/control_msgs.git ../control_msgs; \
	else \
		echo "control_msgs already cloned"; \
	fi

realtime_tools:
	@if [ ! -d "../realtime_tools" ]; then \
		echo "Cloning realtime_tools (iron branch) ..."; \
		git clone -b iron https://gitlab.nimbl-bot.com/tcourtois/realtime_tools.git ../realtime_tools; \
	else \
		echo "realtime_tools already cloned"; \
	fi

gpio_controllers:
	@if [ ! -d "../gpio_controllers" ]; then \
		echo "Cloning gpio_controllers (iron branch) ..."; \
		git clone -b iron https://gitlab.nimbl-bot.com/tcourtois/gpio_controllers.git ../gpio_controllers; \
	else \
		echo "gpio_controllers already cloned"; \
	fi

.PHONY: remove-conflicting-packages
remove-conflicting-packages:
	@echo "Removing potentially conflicting debian packages ..."
	@for package in $(CONFLICTING_PACKAGES); do \
		if dpkg -l | grep -q $$package; then \
		  echo "Removing $$package ..."; \
			sudo apt remove -y $$package || echo "Failed to remove $$package (might not be installed)"; \
		else \
		  echo "$$package not installed, skipping"; \
		fi; \
	done
	@echo "Conflicting packages removal completed"

.PHONY: build
build: repos
	@echo "Building cloned packages ..."
	@cd ../.. && colcon build --symlink-install --packages-select control_msgs realtime_tools gpio_controllers telesoud_api telesoud_nimblbot_interface welding_scene_publisher interface_rviz_plugin interface_custom_msgs --allow-overriding control_msgs realtime_tools
	@echo "Cloned packages built"

.PHONY: clean
clean:
	@echo "Cleaning cloned packages..."
	@rm -rf ../control_msgs ../realtime_tools ../gpio_controllers
	@rm -f $(DEB_FILE)
	@echo "Cleaning done"

.PHONY: help
help:
	@echo "Available commands:"
	@echo "  make install      - Complete installation"
	@echo "  make check-ros    - Checks if ROS2 is installed"
	@echo "  make manual-deps  - Install dependencies"
	@echo "  make repos        - Clone only the repositories"
	@echo "  make remove-conflicting-packages - Remove conflicting debian packages"
	@echo "  make build        - Build selected packages"
	@echo "  make clean        - Remove cloned packages"
	@echo "  make help         - Show this help"	
