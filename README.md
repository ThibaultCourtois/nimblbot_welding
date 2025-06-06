# Nimbl'Bot Welding 

This repository contains all the code necessary to use the WeezTouch controller with a Nimbl'bot robot (NB-55 or NB-120) to define and execute welding trajectories, as well as STL files for torch geometries and welding scene environments.

## Features and Documentation

- NB55 and NB120 modular and cartesian control with the WeezTouch
- All WeezTouch features (definition and execution of linear welding paths, circle-arc paths, spline paths, and horse-saddle paths) are supported by NimblBot robots
- A custom RViz plugin `interface_rviz_plugin` and a custom RViz configuration file `projet_soudure.rviz` are included in the repository
- Mesh publishing for concrete welding applications is available to test the trajectories proposed by WeezU-Welding

Here is the current complete documentation of the project for more information about its features, architecture, and code:

## Requirements 

1. Complete the ROS2 configuration page in Notion
2. Use the upcoming release of the `feat/cartesian_control` branch of the `nimblbot-ros2` packages directory with a `nimblbot_cartesian_command` package that contains a node called `multiple_command.py`
3. You also need to have the NimblBot-WeezTouch electrical armory connected to your computer with an ethernet cable. To set up the required TCP/IP connection between the two systems, follow these steps: [NimblBot Welding Documentation](https://tcourtois.gitlab.io/nimblbot-welding/index.html)

### TCP/IP Connection Setup

1. **Power on the electrical armory** (this will start the WeezTouch, which will remain on its starting menu)

2. **Find your computer's IP address:**
   - On the NimblBot computer, open a terminal and run `ip addr show`
   - Look for the IP address of interface `enp58s0` with status `<BROADCAST, MULTICAST, UP, LOWER_UP>`
   - Note down this IP address (format: xxx.xxx.xxx.xxx)

3. **Access the Telesoud computer:**
   - Use the keyboard of the electrical armory and press `Ctrl+Alt+F2` to open a session on the Telesoud computer
   - Login credentials: 
     - Username: `weez-u-welding`
     - Password: `0000`

4. **Configure the WeezTouch IP settings:**
   - Run the following command to open the WeezTouch configuration file:
     ```bash
     nano ../runtime/Telesoud_V1/_runtime/user-config/welding-config.yml
     ```
   - Find the following section:
     ```yaml
     robot:
       type: WeezLite
     ```
   - Locate the `weezliteConfig` section and update the IP address to match the one you found in step 2:
     ```yaml
     weezliteConfig:
       loopPeriod: 25
       tcpConnectionConfig:
         ipAddress: xxx.xxx.xxx.xxx  # Replace with your computer's IP
     ```

5. **Save and restart:**
   - Save the file (`Ctrl+X`, then `Y`, then `Enter`)
   - Reboot the electrical armory for the changes to take effect

You should now be able to fully use the WeezTouch controller and pass the starting screen you had before the IP setup.

## Installation
1. Clone this repository into your ROS2 workspace: 
    
    ``` bash
    roscd
    cd src
    git clone ssh://git@gitlab.nimbl-bot.com:9022/tcourtois/nimblbot-welding.git
    ```

2. First build the following packages :
  
    ``` bash
    roscd
    colcon build --packages_select custom_control_msgs custom_realtime_tools --symlink-install
    ```

3. Then you can build gpio_controllers_iron : 

    ``` bash
    roscd
    colcon build --packages_select gpio_controllers_iron --symlink-install
    ```

4. Finally you can build every other packages of the repository : 

    ``` bash
    roscd
    colcon build --packages_select interface_custom_msgs interface_rviz_plugin telesoud_api telesoud_nimblbot_interface welding_scene_publisher --symlink-install
    ```

## Usage

### Launch in simulation only with a NB-55:

    ``` bash
    ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_55.launch.py simulation:=True
    ```

### Launch with a real NB-120 robot : 

    ```bash
    ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_120.launch.py simulation:=False
    ```

## Rviz Plugin
In Rviz if the custom panel don't show up directly go to `Panels`,  `Add New Panel` and select the panel inside the `interface_rviz_plugin directory`
