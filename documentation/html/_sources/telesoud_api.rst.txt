Telesoud API Package
====================

Overview
--------

The **Telesoud API Package** provides the RPC communication layer between 
Telesoud welding software and the robot control system. It acts as the 
network entry point for all Telesoud commands.

Purpose
-------

* **RPC Server**: Receives commands from Telesoud over network
* **Protocol Bridge**: Converts RPC calls to ROS2 messages  
* **Data Translation**: Handles format conversion between systems

Architecture
------------

.. code-block:: text

    Telesoud PC  →  [API RPC Server]  →  telesoud_nimblbot_interface
                         (port 8080)           (ROS2 topics)

The API package acts as a simple gateway:

1. **Listens** for RPC commands from Telesoud
2. **Publishes** commands to ``/telesoud/instructions`` topic
3. **Subscribes** to ``/translator/robotData`` for feedback
4. **Returns** robot status via RPC response

Key Components
--------------

**api_node.cpp**
   Main RPC server handling Telesoud communication

**dataFromAndToTelesoudTranslator.cpp**  
   Data format conversion utilities

Integration
-----------

The API package works with the main interface:

* **Sends commands** → ``TranslatorNode`` in telesoud_nimblbot_interface
* **Receives feedback** ← ``WeldingCommandHandlerNode`` status
* **Provides network interface** for remote Telesoud operation

This package handles **only the network communication layer**. All robot 
control logic is implemented in the ``telesoud_nimblbot_interface`` package.
