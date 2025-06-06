Interface_Custom_Msgs
=====================

Overview
--------

The Telesoud NimblBot Interface uses custom ROS2 message definitions to handle 
communication between the Telesoud system and robot control components. These 
messages provide structured data exchange with proper type safety and compatibility.

Message Architecture
--------------------

The message flow follows this pattern:

.. code-block:: text

    TelesoudInstruction → Command → CommandStatus → RobotData
    (from Telesoud)      (to robot)  (feedback)     (status)

Message Definitions
-------------------

TelesoudInstruction.msg
~~~~~~~~~~~~~~~~~~~~~~~

**Purpose**: Receives instructions from Telesoud system via RPC interface.

.. code-block:: none

   uint16 instruction_code      # Telesoud command type
   float64[6] pose1            # Target pose [x,y,z,roll,pitch,yaw]
   float64[6] speed_vector     # Velocity vector [vx,vy,vz,wx,wy,wz]
   float64 speed               # Movement speed (m/s)
   bool free_drive_btn_status  # Free drive button state

**Usage**: Published on ``/telesoud/instructions`` topic by the API node.

**Coordinate System**: 
   * Position: meters (x, y, z)
   * Orientation: radians (roll, pitch, yaw)
   * Velocity: m/s and rad/s

Command.msg
~~~~~~~~~~~

**Purpose**: Internal robot commands translated from Telesoud instructions.

.. code-block:: none

   uint8 command_type                # Robot command type
   uint8 COMMAND_STOP=0             # Stop all movement
   uint8 COMMAND_GET_ROBOT_DATA=1   # Request robot status
   uint8 COMMAND_SET_DYNAMIC=7      # Set dynamic velocity
   uint8 COMMAND_START_DYNAMIC=8    # Start dynamic movement
   uint8 COMMAND_PLAY_CARTESIAN=15  # Execute cartesian trajectory
   uint8 COMMAND_PLAY_JOINT=16      # Execute joint trajectory
   geometry_msgs/Pose target_pose   # Target end-effector pose
   float64 speed                    # Movement speed
   geometry_msgs/Twist speed_vector # Cartesian velocity vector
   uint32 command_id               # Unique command identifier

**Usage**: Published on ``/translator/command`` topic by TranslatorNode.

**Command Types**:

.. list-table:: Command Type Reference
   :header-rows: 1
   :widths: 10 20 70

   * - Code
     - Command
     - Description
   * - 0
     - STOP
     - Emergency stop all robot movements
   * - 1
     - GET_ROBOT_DATA
     - Request current robot state and pose
   * - 7
     - SET_DYNAMIC
     - Update dynamic movement velocity vector
   * - 8
     - START_DYNAMIC
     - Begin real-time cartesian velocity control
   * - 15
     - PLAY_CARTESIAN
     - Execute cartesian trajectory to target pose
   * - 16
     - PLAY_JOINT
     - Execute joint-space trajectory to target pose

CommandStatus.msg
~~~~~~~~~~~~~~~~~

**Purpose**: Execution feedback from robot control system.

.. code-block:: none

   uint32 command_id        # Matching command identifier
   bool success            # Execution success/failure
   string message          # Status description or error details
   uint8 command_type      # Echo of original command type
   RobotData robot_data    # Current robot state (optional)

**Usage**: Published on ``/welding_command_handler/command_status`` topic.

**Status Flow**:
   1. Command received with unique ID
   2. Execution attempted by robot controller
   3. Status published with same ID for tracking
   4. Includes current robot data if available

RobotData.msg
~~~~~~~~~~~~~

**Purpose**: Comprehensive robot state and pose information.

.. code-block:: none

   # Position data
   geometry_msgs/Pose pose          # Current pose (position + quaternion)
   float64[] xyzwpr                # Euler format [x,y,z,roll,pitch,yaw]
   
   # Robot states  
   bool robot_in_fault_status      # Hardware fault detected
   string error_message            # Fault description
   bool robot_in_slave_mode_status # Compatibility field (unused)
   bool collision_status           # Compatibility field (unused)
   bool emergency_stop             # Compatibility field (unused)
   bool welding_trigger_plc_signal # Welding activation signal
   uint8 operation_mode           # Welding operation mode
   
   # Timestamp
   builtin_interfaces/Time timestamp # Data acquisition time

**Usage**: 
   * Embedded in CommandStatus messages
   * Published on ``/translator/robotData`` topic for Telesoud feedback

**Pose Formats**:
   * ``pose``: Standard ROS geometry_msgs/Pose (quaternion orientation)
   * ``xyzwpr``: Telesoud-compatible format (Euler angles in radians)

**Operation Modes**:
   * 0 = Auto mode
   * 1 = 4T welding mode  
   * 2 = 2T welding mode

Message Flow Example
--------------------

Cartesian Movement
~~~~~~~~~~~~~~~~~~

.. code-block:: text

   1. Telesoud → TelesoudInstruction (instruction_code=15, pose1=[0.5,0.2,0.3,0,0,1.57])
   2. Translator → Command (COMMAND_PLAY_CARTESIAN, target_pose, command_id=123)
   3. Robot Controller → CommandStatus (command_id=123, success=true, robot_data)
   4. Translator → RobotData (current pose, status)

The messages are defined in the ``interface_custom_msgs`` package and must be 
built before the main telesoud_nimblbot_interface package.
