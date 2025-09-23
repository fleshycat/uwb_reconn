
# Project Overview

This project is a ROS 2-based system for controlling a swarm of drones. It utilizes UWB (Ultra-Wideband) technology for relative positioning and formation control. The system is designed to be decentralized, with each drone running a `drone_manager` node that communicates with other drones in the swarm.

The project is composed of the following ROS 2 packages:

*   **`drone_manager`**: The core package that manages the drone's state, formation control, and communication with other drones.
*   **`uwb_sim`**: A package for simulating the UWB environment and launching the drone manager.
*   **`jfi_comm`**: A package for serial communication with J-Fi UWB modules.
*   **`nlink_parser_ros2`**: A package for parsing data from Nooploop UWB products.
*   **`rosmsgs`**: A package containing custom ROS messages for UWB data.

## Architecture

The system follows a decentralized architecture where each drone runs its own `drone_manager` node. The `drone_manager` nodes communicate with each other to share their positions and maintain the desired formation. The formation control is based on a potential field approach, where each drone is attracted to its desired position in the formation and repelled by other drones.

The `drone_manager` node uses a particle filter to estimate the position of a target based on UWB range measurements. The drones can operate in different modes, such as searching for a target, collecting data, and returning to base.

# Building and Running

To build and run the project, you can use the following commands:

```bash
# Build the project
colcon build

# Source the workspace
source install/setup.bash

# Launch the drone manager
ros2 launch uwb_sim drone_manager.launch.py
```

# Development Conventions

*   **Coding Style**: The project follows the standard ROS 2 coding style guidelines.
*   **Testing**: The project includes a set of tests for the `drone_manager` package. To run the tests, you can use the following command:

```bash
colcon test
```

*   **Contribution**: Contributions to the project are welcome. Please follow the ROS 2 contribution guidelines.
