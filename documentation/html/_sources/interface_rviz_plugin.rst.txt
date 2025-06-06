Interface Rviz Plugin
=====================

Overview
--------

The **Interface RViz Plugin** provides a graphical control panel integrated 
directly into RViz for monitoring and controlling the Telesoud NimblBot system. 
This plugin offers real-time visualization and manual control capabilities for 
operators and developers.

Purpose
-------

The plugin enables operators to:

* **Monitor robot state** with color-coded status indicators
* **Adjust control parameters** in real-time during operation
* **Switch control modes** between cartesian and modular control
* **View live TCP pose** information
* **Trigger emergency stops** and system calibration
* **Clear visualization traces** for better viewing

Plugin Features
---------------

Robot State Display
~~~~~~~~~~~~~~~~~~~

**Color-coded Status Indicator**
   Displays current robot state with intuitive color scheme:

   * **IDLE** - Robot ready for commands
   * **DYNAMIC_MOVEMENT** - Teleoperation or Telesoud execution active  
   * **CARTESIAN_TRAJECTORY** - Executing cartesian path
   * **JOINT_TRAJECTORY** - Executing non-interpolated trajectory
   * **MODULAR_CONTROL** - Direct modular control active
   * **PAUSE** - Safety pause or emergency stop

Control Parameter Adjustment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Position Gain Slider (1.0 - 10.0)**
   Adjusts sensitivity for cartesian position control during dynamic movement.
   Higher values = more responsive position tracking.

**Quaternion Gain Slider (1.0 - 10.0)**  
   Controls orientation sensitivity during dynamic movement.
   Higher values = more responsive orientation tracking.

**Modular Gain Slider (1.0 - 10.0)**
   Sets velocity scaling for direct module control.
   Higher values = faster module movements.

Live TCP Pose Display
~~~~~~~~~~~~~~~~~~~~~

**Position Information (mm)**
   * X, Y, Z coordinates of tool center point
   * Real-time updates during robot movement

**Orientation Information (degrees)**
   * Roll, Pitch, Yaw angles in degrees
   * Converted from quaternion for readability

Control Mode Management
~~~~~~~~~~~~~~~~~~~~~~~

**Modular Command Toggle**
   * **OFF**: Standard cartesian control mode
   * **ON**: Direct module control mode with additional indicators:
     - Current control mode (AZIMUTH/TILT)
     - Current movement speed
     - Module selection feedback

Safety Controls
~~~~~~~~~~~~~~~

**Emergency Stop Button**
   * Large red button for immediate motion halt
   * Toggle state: Normal → **EMERGENCY STOP ACTIVATED**
   * Overrides all other commands when active

**Set Zeros Button**
   * Calibrates robot joint positions to zero configuration
   * Only available in modular control mode
   * Automatically switches to appropriate control mode

Utility Functions
~~~~~~~~~~~~~~~~~

**Clear TCP Trace**
   * Removes TCP path visualization traces from RViz

User Interface
--------------

Layout Organization
~~~~~~~~~~~~~~~~~~~

The plugin panel is organized in sections:

.. code-block:: text

   ┌─────────────────────────────────┐
   │        Robot State: IDLE        │  ← Status display
   ├─────────────────────────────────┤
   │  [Clear TCP Trace] [Set Zeros]  │  ← Utility buttons
   ├─────────────────────────────────┤
   │  Pos Gain  Quat Gain  Mod Gain  │  ← Control sliders
   │    [▬▬▬]     [▬▬▬]     [▬▬▬]    │
   ├─────────────────────────────────┤
   │           TCP Pose              │  ← Live pose display
   │  Position (mm)  Orientation(°)  │
   │   X: 245.7       W: 12.3        │
   │   Y: 133.2       P: -5.1        │ 
   │   Z: 89.4        R: 0.8         │
   ├─────────────────────────────────┤
   │     [Modular Command]           │  ← Mode toggle
   │   Current mode: AZIMUTH         │
   │   Current speed: 0.25           │
   ├─────────────────────────────────┤
   │      [EMERGENCY STOP]           │  ← Safety control
   └─────────────────────────────────┘
