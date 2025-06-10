# Nimbl'Bot Welding 

This repository contains all the code necessary to use the WeezTouch controller with a Nimbl'bot robot (NB-55 or NB-120) to define and execute welding trajectories, as well as STL files for torch geometries and welding scene environments.

## Features and Documentation

- NB55 and NB120 modular and cartesian control with the WeezTouch
- All WeezTouch features (definition and execution of linear welding paths, circle-arc paths, spline paths, and horse-saddle paths) are supported by NimblBot robots
- A custom RViz plugin `interface_rviz_plugin` and a custom RViz configuration file `projet_soudure.rviz` are included in the repository
- Mesh publishing for concrete welding applications is available to test the trajectories proposed by WeezU-Welding

The complete documentation of the project is available inside the repository at [`documentation/html/`](documentation/html/). After cloning the repository, you can open the `index.html` file in your browser for detailed information.

## Requirements 

1. Complete the ROS2 configuration page in Notion
2. Use the upcoming release of the `feat/cartesian_control` branch of the `nimblbot-ros2` packages directory with a `nimblbot_cartesian_command` package that contains a node called `multiple_command.py`
3. You also need to have the NimblBot-WeezTouch electrical armory connected to your computer with an ethernet cable. To set up the required TCP/IP connection between the two systems, follow these steps: 

### TCP/IP Connection Setup

1. **Power on the electrical armory** 
    - This will turn on the WeezTouch controller and the Telesoud system especielly the AEON industrial computer.

2. **Set up computer's IP address:**
   - On the NimblBot computer, open a terminal and run `ip link show`
   - Look for the device name of the interface with status `<BROADCAST, MULTICAST, UP, LOWER_UP>` corresponding to the cabled ethernet interface (it should begin with `enp`)
   - Then configure your IP adress for the Telesoud computer network (`192.168.10.1`) by running the following line : 
   ```bash
   sudo ip addr add 192.168.10.2 dev <your device name>
   sudo ip link set <your device name> up
   ```
   - Now you should be able to ping the Telesoud computer `ping 192.168.10.1` from Nimbl'bot computer, the TCP/IP connexion is estalished

## Installation
1. Clone this repository into your ROS2 workspace: 
    
    ```bash
    roscd
    cd src
    git clone ssh://git@gitlab.nimbl-bot.com:9022/tcourtois/nimblbot-welding.git
    ```

2. By cloning this repository you downloaded the following file : `rcplib_2.3.0-1_amd64.deb`. This C++ library is required for the RPC communication protocole used by the package `telesoud_api`. Run the following command to install it in your home directory : 
    ```
    mkdir -p ~/.local && dpkg-deb -x rpclib_2.3.0-1_amd64.deb temp && cp -r temp/usr/local/* ~/.local/ && rm -rf temp
    ```
    
3. Next install the following dependencies required for the installation (`test-msgs` is necessary for `gpio_controllers_iron` and `ros-iron-usb-cam` is used for NB-55 torch laser device) :
    ```bash
    cd ~
    sudo apt install ros-iron-test-msgs
    sudo apt install ros-iron-usb-cam
    source ~/.zshrc
    ```

4. Build the two dependencies of gpio_controllers_iron : 
    ```bash
    roscd
    colcon build --packages-select custom_control_msgs custom_realtime_tools --symlink-install
    source ~/.zshrc
    ```

5. Then you can build gpio_controllers_iron : 

    ```bash
    roscd
    colcon build --packages-select gpio_controllers_iron --symlink-install
    source ~/.zshrc
    ```

6. Finally you can build every other packages of the repository : 

    ```bash
    roscd
    colcon build --packages-select interface_custom_msgs interface_rviz_plugin telesoud_api telesoud_nimblbot_interface welding_scene_publisher --symlink-install
    source ~/.zshrc
    ```

## Usage

### Launch in simulation only with a NB-55:

    ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_55.launch.py simulation:=True
    

### Launch with a real NB-120 robot : 

    ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_120.launch.py simulation:=False
    

## Rviz Plugin
In Rviz if the custom panel don't show up directly go to `Panels`,  `Add New Panel` and select the panel inside the `interface_rviz_plugin directory`
