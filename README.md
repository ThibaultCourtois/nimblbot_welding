# Nimbl'Bot Welding 

This repository contains all the code necessary to use the WeezTouch controller with a Nimbl'bot robot (NB-55 or NB-120) to define and execute welding trajectories, as well as STL files for torch geometries and welding scene environments.

## Features and Documentation

- NB55 and NB120 modular and cartesian control with the WeezTouch
- All WeezTouch features (definition and execution of linear welding paths, circle-arc paths, spline paths, and horse-saddle paths) are supported by NimblBot robots
- A custom RViz plugin `interface_rviz_plugin` and a custom RViz configuration file `projet_soudure.rviz` are included in the repository
- Mesh publishing for concrete welding applications is available to test the trajectories proposed by WeezU-Welding

## Requirements 

1. Complete the ROS2 configuration page in Notion
2. Use the upcoming release of the `feat/cartesian_control` branch of the `nimblbot-ros2` packages directory with a `nimblbot_cartesian_command` package that contains a node called `multiple_command.py`
3. This installation guide assumes you have a working Nimbl'Bot system with functional `NimblPy` and `PyLuos` libraries. Ensure you can successfully run the `teleop_demo_launch` from the `nimblbot-ros2` packages directory.
5. You also need to have the NimblBot-WeezTouch electrical armory connected to your computer with an ethernet cable. To set up the required TCP/IP connection between the two systems, follow these steps: 

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
2. Now you can run the automatic installation script: 
    
    ```bash
    cd nimblbot-welding
    make
    ```
3. Finally you can source your environment: 

    ```bash
    source ~/.zshrc
    ```

## Usage

### Launch in simulation only with a NB-55:

    ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_55.launch.py simulation:=true
    
### Launch with a real NB-120 robot : 

    ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_120.launch.py simulation:=false
    
## Rviz Plugin
In Rviz if the custom panel don't show up directly go to `Panels`,  `Add New Panel` and select the panel inside the `interface_rviz_plugin directory`
