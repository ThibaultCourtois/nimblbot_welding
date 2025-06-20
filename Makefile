NEXTCLOUD_SHARE_LINK = https://files.nimbl-bot.com/s/eASNJt3BtyMeADS/download
DEB_FILE = rpclib_2.3.0-1_amd64.deb 

.ONESHELL:

.PHONY: install
install: check-deps deps repos rosdep-install build
	@echo "Installation succeed"
	@echo "Please run the following command to update your environment:"
	@echo "source ~/.zshrc"
	@echo "You will then be able to use Nimblbot Welding :) !"

.PHONY: check-deps
check-deps:
	@echo "Prerequities verification ..."
	@command -v git >/dev/null 2>&1 || { echo "Git is not installed"; exit 1; }
	@command -v wget >/dev/null 2>&1 || { echo "wget is not installed"; exit 1; }
	@command -v make >/dev/null 2>&1 || { echo "make is not installed"; exit 1; }
	@command -v colcon >/dev/null 2>&1 || { echo "colcon is not installed"; exit 1; }
	@command -v rosdep >/dev/null 2>&1 || { echo "rosdep is not installed"; exit 1; }
	@echo "Prerequities checked"

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

.PHONY: rosdep-install
rosdep-install: repos
	@echo "Installing packages dependancies ..."
	@if [ ! -d "/etc/ros/rosdep" ]; then \
		echo "Initializing rosdep..."; \
		sudo rosdep init || true; \
	fi
	@rosdep update
	@cd ../.. && rosdep install --from-paths src/control_msgs src/realtime_tools src/gpio_controllers src/nimblbot-welding/telesoud_api src/nimblbot-welding/telesoud_nimblbot_interface src/nimblbot-welding/welding_scene_publisher src/nimblbot-welding/interface_rviz_plugin src/nimblbot-welding/interface_custom_msgs --ignore-src -r -y
	@echo "Dependancies installed"

.PHONY: build
build: repos rosdep-install
	@echo "Building cloned packages ..."
	@cd ../.. && colcon build --symlink-install --packages-select control_msgs realtime_tools gpio_controllers telesoud_api telesoud_nimblbot_interface welding_scene_publisher interface_rviz_plugin interface_custom_msgs
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
	@echo "  make install       - Complete installation"
	@echo "  make deps         - Install only the C++ library"
	@echo "  make repos        - Clone only the repositories"
	@echo "  make rosdep-install - Install ROS2 dependencies"
	@echo "  make build        - Build selected packages"
	@echo "  make clean        - Remove cloned packages"
	@echo "  make help         - Show this help"	
