## Relevant Files

- `src/uwb_sim/launch/drone_manager.launch.py`: Main launch file that needs to be modified to set the environment variable for static discovery.
- `src/uwb_sim/config/fastdds_profiles.xml`: The main Fast DDS profiles file that enables static discovery.
- `src/uwb_sim/config/static_discovery.xml`: The detailed XML file containing all participant (node) definitions.
- `src/drone_manager/drone_manager/drone_manager.py`: Source code for the drone_manager node, needs to be analyzed to identify published/subscribed topics.
- `src/J-Fi/src/jfi_comm/src/serial_comm_node.cpp`: Source code for the serial_comm_node, needs to be analyzed for topics.
- `src/nlink_parser_ros2/src/nlink_parser_ros2/src/linktrack/main.cpp`: Source code for the linktrack node, needs to be analyzed for topics.

### Notes

- To verify the implementation, launch the system and use a network analysis tool like Wireshark to monitor network traffic. You should no longer see the multicast packets associated with the Simple Discovery Protocol (SDP) on port 7400. All nodes should still communicate correctly.
- **Rule:** When creating the XML configuration, strictly follow the structure and approach shown in the `proposed_xml/` directory. The `STATIC_FASTRTPS_PROFILE_TEST.xml` and `test.xml` files serve as the primary reference.

## Tasks

- [x] 1.0 Analyze System Architecture and Topics
  - [x] 1.1 Inspect `drone_manager.launch.py` to identify all nodes launched for a single drone.
  - [x] 1.2 Recursively inspect any included launch files (e.g., `linktrack.launch.py`) to find all participants.
  - [x] 1.3 For each node, analyze its source code (`.py` or `.cpp`) to list all publishers and subscribers.
  - [x] 1.4 Document the topic name, message type, and QoS for each publisher and subscriber.
  - [x] 1.5 Identify topics for the `MicroXRCEAgent` based on the messages it bridges from the flight controller.
- [x] 2.0 Create Static DDS Discovery Configuration Files
  - [x] 2.1 Create a new directory `src/uwb_sim/config`.
  - [x] 2.2 Create the main `fastdds_profiles.xml` file that enables static discovery (`<EDP>STATIC</EDP>`).
  - [x] 2.3 In `fastdds_profiles.xml`, point to the detailed participant definition file using the `<static_edp_xml_config>` tag.
  - [x] 2.4 Create the `static_discovery.xml` file.
  - [x] 2.5 For each of the 4 drones, define all its nodes (participants) in `static_discovery.xml` with unique names (e.g., `drone_manager_1`).
  - [x] 2.6 For each participant, add `<writer>` and `<reader>` tags corresponding to the topics identified in step 1.
- [ ] 3.0 Integrate Configuration into the ROS 2 Launch System
  - [ ] 3.1 Modify `src/uwb_sim/launch/drone_manager.launch.py`.
  - [ ] 3.2 Import `SetEnvironmentVariable` from `launch.actions`.
  - [ ] 3.3 Add the `SetEnvironmentVariable` action to the `LaunchDescription`.
  - [ ] 3.4 Set the `FASTRTPS_DEFAULT_PROFILES_FILE` variable to the absolute path of `fastdds_profiles.xml`.
- [ ] 4.0 Verify the Static Discovery Implementation
  - [ ] 4.1 Build the workspace using `colcon build`.
  - [ ] 4.2 Source the workspace (`source install/setup.bash`).
  - [ ] 4.3 Launch the system using `ros2 launch uwb_sim drone_manager.launch.py`.
  - [ ] 4.4 Check that all nodes start up correctly and there are no communication errors in the logs.
  - [ ] 4.5 Use `ros2 topic list` and `ros2 topic echo` to verify that topics are being published and subscribed to correctly.
  - [ ] 4.6 (Optional) Use Wireshark to confirm that RTPS discovery multicast traffic on UDP port 7400 has been eliminated.