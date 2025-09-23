# Topic Definitions

This document lists all the topics published and subscribed to by the nodes in the system.

## `drone_manager`

**Node Name:** `drone_manager{system_id}`

### Publishers

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/fmu/in/offboard_control_mode` | `px4_msgs.msg.OffboardControlMode` | 10 |
| `drone{system_id}/fmu/in/trajectory_setpoint` | `px4_msgs.msg.TrajectorySetpoint` | SensorData |
| `drone{system_id}/manager/out/monitoring` | `px4_msgs.msg.Monitoring` | SensorData |
| `drone{system_id}/manager/out/actuator_motors` | `px4_msgs.msg.ActuatorMotors` | SensorData |
| `drone{system_id}/fmu/in/vehicle_command` | `px4_msgs.msg.VehicleCommand` | 10 |
| `drone{system_id}/jfi/in/ranging` | `uwb_msgs.msg.Ranging` | SensorData |
| `drone{system_id}/jfi/in/target` | `px4_msgs.msg.TrajectorySetpoint` | SensorData |

### Subscribers

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/nlink_linktrack_nodeframe2` | `nlink_parser_ros2_interfaces.msg.LinktrackNodeframe2` | SensorData |
| `drone{system_id}/fmu/out/monitoring` | `px4_msgs.msg.Monitoring` | SensorData |
| `drone{system_id}/fmu/out/actuator_motors` | `px4_msgs.msg.ActuatorMotors` | SensorData |
| `qhac/manager/in/timestamp` | `std_msgs.msg.Header` | 10 |
| `drone{system_id}/manager/in/global_path` | `px4_msgs.msg.GlobalPath` | 10 |
| `drone{system_id}/manager/in/mode_change` | `std_msgs.msg.UInt8` | SensorData |
| `drone{system_id}/jfi/out/drone{i}/ranging` | `uwb_msgs.msg.Ranging` | SensorData |
| `drone{system_id}/jfi/out/drone{i}/target` | `px4_msgs.msg.TrajectorySetpoint` | SensorData |

---

## `serial_comm_node`

**Node Name:** `serial_comm_node{system_id}`

### Publishers

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/jfi/out/drone{id}/ranging` | `uwb_msgs.msg.Ranging` | SensorData |
| `drone{system_id}/jfi/out/drone{id}/target` | `px4_msgs.msg.TrajectorySetpoint` | SensorData |

### Subscribers

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/jfi/in/ranging` | `uwb_msgs.msg.Ranging` | SensorData |
| `drone{system_id}/jfi/in/target` | `px4_msgs.msg.TrajectorySetpoint` | SensorData |

---

## `linktrack`

**Node Name:** `linktrack`

### Publishers

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/nlink_linktrack_nodeframe2` | `nlink_parser_ros2_interfaces.msg.LinktrackNodeframe2` | SensorData |

### Subscribers

*None*

---

## `MicroXRCEAgent`

**Node Name:** `MicroXRCEAgent`

This node acts as a bridge between the ROS 2 DDS world and the PX4 flight controller.

### Publishers (to ROS 2)

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/fmu/out/monitoring` | `px4_msgs.msg.Monitoring` | SensorData |
| `drone{system_id}/fmu/out/actuator_motors` | `px4_msgs.msg.ActuatorMotors` | SensorData |

### Subscribers (from ROS 2)

| Topic | Message Type | QoS Profile |
|---|---|---|
| `drone{system_id}/fmu/in/offboard_control_mode` | `px4_msgs.msg.OffboardControlMode` | 10 |
| `drone{system_id}/fmu/in/trajectory_setpoint` | `px4_msgs.msg.TrajectorySetpoint` | SensorData |
| `drone{system_id}/fmu/in/vehicle_command` | `px4_msgs.msg.VehicleCommand` | 10 |
