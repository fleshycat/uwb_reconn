## Project Overview

This project is a ROS2-based simulation and control system for a swarm of drones. The system uses Ultra-Wideband (UWB) ranging for localization and a potential field-based approach for formation control. The primary goal is to enable multiple drones to fly in a coordinated formation, locate a target, and perform a mission.

The project is composed of several ROS2 packages:

*   **`drone_manager`**: The core package that contains the main control node for each drone. It subscribes to UWB data, communicates with the PX4 flight controller, and uses a particle filter to estimate the drone's position and the target's position. It also implements formation control using potential fields.
*   **`uwb_sim`**: This package contains the Gazebo simulation environment. It likely includes models for the drones, UWB anchors, and tags.
*   **`nlink_parser_ros2`**: A package for parsing UWB data from nlink devices.
*   **`rosmsgs`**: Contains custom ROS2 message definitions used throughout the project.
*   **`jfi_comm`**: A package for serial communication, likely for interfacing with hardware.

The system is designed to be modular and extensible, with each drone running its own `drone_manager` node. The drones communicate with each other to share their positions and target estimates, enabling them to work together as a swarm.

## Building and Running

### Building the Project

To build the project, use the standard ROS2 build tool, `colcon`:

```bash
colcon build
```

### Running the Simulation

To run the simulation, you can use the provided launch file. This will start Gazebo, spawn the drones, and launch the necessary nodes for localization and control.

```bash
# Running a Gazebo World with Anchors and Tags
ros2 launch uwb_sim robot_in_gazebo_world_run.launch.py
```

### Running the Drone Manager

To run the drone manager for a single drone, you can use the following launch file. You can specify the drone's `system_id` and other parameters as arguments.

```bash
ros2 launch uwb_sim drone_manager.launch.py
```

### Running the Localization Node

This project also includes a separate localization node that can be run independently.

```bash
# Running UWB localization Node
ros2 run uwb_localization sqrrange_leastsqr_localization.py
```

## Development Conventions

*   **Coding Style**: The Python code generally follows the PEP 8 style guide.
*   **Testing**: The project includes a `test` directory in the `drone_manager` package with tests for copyright, flake8, and pep257.
*   **Architecture**: The system is designed around the ROS2 node and topic architecture. Each drone is an independent node that communicates with other nodes through topics.
*   **Configuration**: The `drone_manager` node is highly configurable through ROS2 parameters. These parameters can be set in the launch file or on the command line.
