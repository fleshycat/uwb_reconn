# PRD: Static DDS Discovery Configuration

## 1. Introduction/Overview

This document outlines the requirements for implementing static discovery for the ROS 2 DDS communication layer. The goal is to move from the default dynamic discovery (Simple Discovery Protocol) to a static configuration to reduce network multicast traffic and create a more deterministic system at startup. This is particularly important for a multi-drone system where network bandwidth and reliability are critical.

This configuration will be specifically for a 4-drone system, where each drone runs an identical set of ROS 2 nodes.

## 2. Goals

*   **Reduce Network Traffic:** Eliminate the multicast traffic associated with the default dynamic discovery process.
*   **Improve Startup Determinism:** Ensure that nodes discover each other in a predictable manner without relying on network-wide multicast announcements.
*   **Define a Scalable Configuration:** Create a configuration pattern that can be understood and potentially extended for more drones in the future.

## 3. User Stories

*   **As a developer,** I want to configure the DDS communication to use static discovery so that I can reduce network overhead and improve system stability.
*   **As a system operator,** I want the drone swarm to initialize quickly and reliably without discovery-related delays or failures, especially in challenging network environments.

## 4. Functional Requirements

1.  **Identify All Participants:** The system must identify all ROS 2 nodes that act as DDS participants. For a single drone, these are:
    *   `MicroXRCEAgent`
    *   `drone_manager{system_id}`
    *   `serial_comm_node{system_id}`
    *   `linktrack` (from `nlink_parser_ros2`)

2.  **Identify All Topics:** The system must map out all topics, including their message types and QoS, for every publisher and subscriber in the nodes listed above. This information is critical for the static discovery configuration.
    *   **(To be completed by analyzing source code)**

3.  **Create a Shared XML Configuration:**
    *   A single XML file (`fastdds_static_profiles.xml`) shall be created to define the static discovery settings.
    *   This file will be shared by all four drones in the system.
    *   The file will contain definitions for all participants (4 drones * 4 nodes/drone = 16 participants), including their unique names and the topics they publish or subscribe to.

4.  **Define Participant Profiles:**
    *   Each participant in the XML file must have a unique name. The naming convention should be based on the node name and the drone's `system_id` (e.g., `drone_manager_1`, `linktrack_3`).
    *   Each participant profile will list its readers (subscribers) and writers (publishers).

5.  **System Integration:**
    *   The ROS 2 launch process must be updated to use the generated XML configuration file. This will be achieved by setting the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable to the path of the `fastdds_static_profiles.xml` file.

## 5. Non-Goals (Out of Scope)

*   This configuration will not support dynamic discovery as a fallback. The system will be strictly static.
*   This PRD does not cover the implementation of the nodes themselves, only their communication configuration.
*   The configuration will be for a fixed size of 4 drones. A solution for a dynamically scaling number of drones is not in scope.

## 6. Design Considerations

*   **Configuration File Location:** The recommended location for the generated `fastdds_static_profiles.xml` file is in a new `config` directory within the `uwb_sim` package (`src/uwb_sim/config/`).
*   **File Format:** The configuration will be an XML file compatible with eProsima Fast DDS.

## 7. Technical Considerations

*   **DDS Implementation:** The target DDS implementation is eProsima Fast DDS.
*   **Environment Variable:** The `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable is the standard method for loading the Fast DDS profiles and must be set in the launch environment of all nodes.

## 8. Success Metrics

*   **Verification of Communication:** All nodes in the 4-drone system must be able to communicate successfully (publish and subscribe to all topics) after the static configuration is applied.
*   **Reduced Network Traffic:** A network analysis (e.g., using Wireshark) should confirm the absence of the multicast traffic associated with Simple Discovery Protocol (SDP).

## 9. Open Questions

*   What are all the topics, message types, and QoS settings for the `MicroXRCEAgent`, `drone_manager`, `serial_comm_node`, and `linktrack` nodes? (This will be answered by the implementation of this PRD).
