Welding Scene Publisher
=======================

Overview
--------

The **Welding Scene Publisher** is a simple visualization node that displays 
3D workpieces in RViz to simulate a welding environment. It provides visual 
context for welding operations.

Purpose
-------

* **3D Scene Visualization**: Displays welding workpieces in RViz
* **Development Aid**: Provides visual targets for welding path planning
* **Demonstration**: Creates realistic welding scenarios for testing

Components
----------

Published Objects
~~~~~~~~~~~~~~~~~

The node publishes two standard welding workpieces:

**Cylindre** (``Cylindre.stl``)
   * Position: ``[0.0, 0.0, 0.0]`` 
   * Color: Gray (0.7, 0.7, 0.7)
   * Use case: Pipe welding simulation

**Welding Saddle** (``SoudureSelleCheval.stl``)
   * Position: ``[2.0, 0.0, 0.0]``
   * Color: Gray (0.7, 0.7, 0.7) 
   * Use case: Complex geometry welding

Scene Geometries
----------------

**Mesh Storage**
   The 3D geometries are included with the project repository in:
   
   .. code-block:: text
   
      welding_scene_publisher/
      └── scene_meshes/
          ├── Cylindre.stl              # Cylindrical workpiece
          └── SoudureSelleCheval.stl    # Welding saddle geometry

This node is **optional** and can be disabled if not needed for the specific 
welding application.
