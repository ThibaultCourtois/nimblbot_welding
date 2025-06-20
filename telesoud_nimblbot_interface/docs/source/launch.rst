Launch Files
============

Overview
--------

The Telesoud NimblBot Interface provides launch files that start  
all necessary nodes for complete system operation. These launch files support 
both simulation and real robot control with different robot configurations.

Available Launch Files
----------------------

The package includes two main launch configurations:

telesoud_nimblbot_interface_55.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose**: Launch configuration for NB55 robot with laser welding torch simulation.

**Default Configuration**:
   * Robot type: ``nb55_v7_welding``
   * Welding tool: Simulated laser torch (``NB55_torche_laser_low.stl``)
   * TCP position: ``[0.0, 0.0, 0.03]`` with ``[0.0, -1.5707, 0.0]`` orientation
   * Use case: Laser welding applications and development

telesoud_nimblbot_interface_120.launch.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Purpose**: Launch configuration for NB120 robot with real welding torch.

**Default Configuration**:
   * Robot type: ``nb120_3m_welding``
   * Welding tool: Real welding torch bracket (``TorchBracket.stl``)
   * TCP position: ``[0.0, 0.0, 0.075]`` with ``[0.0, 1.5707, 3.14159265359]`` orientation
   * Use case: Real welding operations with physical torch

Welding Meshes
~~~~~~~~~~~~~~

**Mesh Storage**
   The welding tool meshes are included with the project repository in:

   .. code-block:: text

      telesoud_nimblbot_interface/
      └── welding_meshes/
          ├── NB55_torche_laser_low.stl    # Laser torch for NB55
          └── TorchBracket.stl             # Welding torch for NB120

Launch Arguments
----------------

Both launch files support the same set of configurable parameters:

Core Parameters
~~~~~~~~~~~~~~~

.. list-table:: Launch Arguments
   :header-rows: 1
   :widths: 20 20 60

   * - Argument
     - Default
     - Description
   * - ``robot_namespace``
     - ``nb``
     - ROS2 namespace for robot nodes
   * - ``robot_type``
     - ``nb55_v7_welding`` / ``nb120_3m_welding``
     - Robot configuration type
   * - ``simulation``
     - ``true``
     - **Enable/disable real robot control**
   * - ``simulate_dynamics``
     - ``false``
     - Enable kinematic simulation
   * - ``rviz_moveit``
     - ``true``
     - Launch RViz visualization
   * - ``rviz_template``
     - ``projet_soudure.rviz``
     - RViz configuration file

Key Launch Argument: simulation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``simulation`` parameter controls the robot control mode:

**simulation=true** (Default)
   * Robot simulation only
   * No connection to real hardware
   * Safe for development and testing
   * Uses simulated robot models

**simulation=false**
   * Connects to real NimblBot hardware
   * Requires ``luos_wrapper_launch`` for hardware interface
   * **Use only with real robot connected**

System Components
-----------------

Each launch file starts the complete system stack:

Core Interface Nodes
~~~~~~~~~~~~~~~~~~~~

*Telesoud API* (:doc:`telesoud_api`)

*Translator* (:doc:`translator`)

*Welding Command Handler* (:doc:`welding_command_handler`)

Robot System Nodes
~~~~~~~~~~~~~~~~~~

**NimblBot Nodes** (``nb_wrapper_launch``)
   * Robot hardware drivers (when ``simulation=false``)
   * Joint controllers and kinematics

**MoveIt Planning** (``moveit_launch``)
   * Inverse kinematics solving

Visualization and Tools
~~~~~~~~~~~~~~~~~~~~~~~

*Welding Tool Mesh* (``mesh_publisher``)

*RViz Visualization*

*TCP Path Tracing* (``tf_path_trail``)

*USB Camera for NB55 setup* (``usb_cam_node``)

*Welding Scene Publisher* (:doc:`welding_scene_publisher`)
