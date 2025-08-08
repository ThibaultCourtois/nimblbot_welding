Welding Scene Publisher
=======================

Overview
--------
The **Welding Scene Publisher** is a ROS2 visualization node that publishes complete 3D welding scenes in RViz. It enables simulation of different welding environments and configurations for development and validation of welding trajectories.

Available Scenes
----------------

The node provides several predefined scenes following the naming convention:
``robot_number_sceneName_orientation`` (H for horizontal, V for vertical)

.. list-table:: Available Welding Scenes
   :widths: 20 15 15 50
   :header-rows: 1

   * - Scene Name
     - Robot
     - Orientation
     - Description
   * - ``55_standard_V``
     - 55
     - Vertical
     - Standard configuration with cylinder, welding saddle and base. Classic development setup.
   * - ``120_standard_V``
     - 120
     - Vertical
     - Simplified configuration with cylinder and welding saddle for robot 120.
   * - ``55_cuve_H``
     - 55
     - Horizontal
     - Tank welding with horizontally positioned cylinder and base support.
   * - ``120_2025_H``
     - 120
     - Horizontal
     - **Historical Scene**: Faithful reproduction of the welding setup implemented during the June 2025 internship, including support with curtains and welding table.

.. note::
   The ``120_2025_H`` scene is particularly important as it exactly reproduces the test environment used during the June 2025 internship, enabling direct validation of developed algorithms.

Components
----------

Available 3D Objects
~~~~~~~~~~~~~~~~~~~~

**Workpieces**
   * ``Cylindre.stl`` - Cylindrical piece for pipe welding
   * ``SoudureSelleCheval.stl`` - Welding saddle for complex geometries
   * ``Cuve.stl`` - Tank for reservoir welding

**Supports and Environment**
   * ``Support_NB120.stl`` - Standard support for NB120 robot
   * ``Support_NB120_Rideaux.stl`` - Support with protective curtains
   * ``table_soudure_montage_soudure_plat.stl`` - Complete welding table
   * ``base.stl`` - Environment base/floor

Scene Geometries
----------------

**File Structure**
   The 3D geometries are stored in:
   
   .. code-block:: text
   
      welding_scene_publisher/
      └── scene_meshes/
          ├── Cylindre.stl
          ├── SoudureSelleCheval.stl  
          ├── Cuve.stl
          ├── Support_NB120.stl
          ├── Support_NB120_Rideaux.stl
          ├── table_soudure_montage_soudure_plat.stl
          └── base.stl

Usage
-----

The **Welding Scene Publisher** is typically launched as part of the complete welding system through launch files (see :doc:`launch` section for more details). The welding scene can be specified using the ``welding_scene`` argument.

**Launch via launch file with default scene**

.. code-block:: bash

   ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_55.launch.py simulation:=true

**Launch via launch file with specific scene**

.. code-block:: bash

   ros2 launch telesoud_nimblbot_interface telesoud_nimblbot_interface_55.launch.py simulation:=true welding_scene:=55_standard_V

**Available scenes**
   * ``55_standard_V`` (default)
   * ``120_standard_V``
   * ``55_cuve_H``
   * ``120_2025_H``


Creating New Scenes
-------------------

To add a new scene, simply add an entry to the ``SCENES_CONFIG`` dictionary:

.. code-block:: python

   SCENES_CONFIG = {
       "new_scene_name": [
           {
               "path": PATH_TO_MESH,               # Path to STL file
               "id": 0,                            # Unique marker ID
               "position": {"x": 0.0, "y": 0.0, "z": 0.0},    # 3D position
               "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},  # Quaternion
               "scale": {"x": 0.001, "y": 0.001, "z": 0.001}, # Object scale
               "color": {"r": 0.7, "g": 0.7, "b": 0.7, "a": 1.0}  # RGBA color
           },
           # Add other objects if needed...
       ]
   }

**Object Configuration Structure**
   * ``path``: Path to mesh file (.stl)
   * ``id``: Unique identifier for each object in the scene
   * ``position``: 3D coordinates (x, y, z) in meters
   * ``orientation``: Quaternion (x, y, z, w) for orientation
   * ``scale``: Scale factor for each axis
   * ``color``: RGBA color (values between 0.0 and 1.0)

**Recommended Naming Convention**
   Follow the format: ``robotNum_configurationName_orientation``
   
   * ``robotNum``: Robot number/type (e.g.: 55, 120)
   * ``configurationName``: Scene description (e.g.: standard, cuve, 2025)
   * ``orientation``: H (horizontal) or V (vertical)
