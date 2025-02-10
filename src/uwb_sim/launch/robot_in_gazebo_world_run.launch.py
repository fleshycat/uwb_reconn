import os
import random
import yaml
from jinja2 import Environment, FileSystemLoader
from ament_index_python import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from decimal import Decimal
import xml.etree.ElementTree as ET
import subprocess
from jinja2 import Template
import launch
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
)
import xml.etree.ElementTree as ET

from launch.event_handlers import OnProcessStart, OnProcessExit


def launch_setup(context, *args, **kwargs):

    # Configuration
    current_package_path = get_package_share_directory('uwb_sim')
    uwb_plugin_package_path = get_package_share_directory('gazebo_uwb_sensor_plugins')
    px4_src_path = LaunchConfiguration('px4_src_path').perform(context)
    gazebo_classic_path = f'{px4_src_path}/Tools/simulation/gazebo-classic/sitl_gazebo-classic'
    anchor_conf_path = LaunchConfiguration('anchor_conf_path').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    world_type = LaunchConfiguration('world_type').perform(context)
    px4_sim_env = SetEnvironmentVariable('PX4_SIM_MODEL', f'gazebo-classic_{robot_type}')

    # Environments
    resource_path_env = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', f'/usr/share/gazebo-11')
    # model_path_env = SetEnvironmentVariable('GAZEBO_MODEL_PATH', f'{gazebo_classic_path}/models/')
    # plugin_path_env = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
    #                                         f'{uwb_plugin_package_path}:{px4_src_path}/build/px4_sitl_default/build_gazebo-classic/')
    model_path_env = SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                                            f'{current_package_path}:{current_package_path}/models:{gazebo_classic_path}/models')
    plugin_path_env = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH',
                                            f'{uwb_plugin_package_path}:{px4_src_path}/build/px4_sitl_default/build_gazebo-classic/')

    # Anchor position
    with open(anchor_conf_path) as f:
        anchor_conf = yaml.safe_load(f)
    anchors_data = [[anchor['id']] +  anchor['pos'] for anchor in anchor_conf['anchors']]
    num_anchor = len(anchors_data)


    # gazebo world
    env = Environment(loader=FileSystemLoader(os.path.join(current_package_path, 'worlds')))
    jinja_world = env.get_template(f'{world_type}.world.jinja')
    forest_world = jinja_world.render(num_anchor = num_anchor, anchors_data=anchors_data)
    world_file_path = os.path.join('/tmp', 'output.world')
    with open(world_file_path, 'w') as f:
        f.write(forest_world)

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_file_path, 'verbose':'false', 'gui':'true' }.items()
    )

    tag_pos_node = Node(
        package='uwb_localization',
        executable='sqrrange_leastsqr_localization',
        name='tag_pos',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    tag_kalman_pos_node = Node(
        package='uwb_localization',
        executable='kalman_localization',
        name='kalman_pos',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    jinja_cmd = [
        f'{gazebo_classic_path}/scripts/jinja_gen.py',
        f'{current_package_path}/models/{robot_type}_uwb/{robot_type}_uwb.sdf.jinja',
        f'{current_package_path}',
        '--mavlink_tcp_port', '4560',
        '--mavlink_udp_port', '14560',
        '--mavlink_id', '1',
        '--gst_udp_port' , '5600' ,
        '--video_uri', '5600',
        '--mavlink_cam_udp_port', '14530',
        '--output-file', f'/tmp/model_{0}.sdf'
    ]
    jinja_process = ExecuteProcess(
        cmd=jinja_cmd,
        output='screen',
    )

    # node to spawn robot model in gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', f'/tmp/model_{0}.sdf', '-entity', f'robot_{0}', '-x', f'0', '-y', f'0' ],
        output='screen')

    # PX4
    # build_path/bin/px4 -i $N -d "$build_path/etc" >out.log 2>err.log &
    cmd = [
        f'{px4_src_path}/build/px4_sitl_default/bin/px4',
        '-i', f'{0}',
        '-d',
        f'{px4_src_path}/build/px4_sitl_default/etc',
        '-w', f'{px4_src_path}/build/px4_sitl_default/ROMFS/instance{0}',
        '>out.log', '2>err.log',
    ]
    px4_process = ExecuteProcess(
        cmd=cmd,
        output='screen',
    )

    # rviz configuration
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("uwb_sim"), "confs", "default.rviz"]
    )

    # rviz node for visualizing robot model
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': True}],
    )

    nodes_to_start = [
        resource_path_env,
        px4_sim_env,
        model_path_env,
        plugin_path_env,
        gazebo_node,
        tag_pos_node,
        tag_kalman_pos_node,
        jinja_process,
        spawn_entity_node,
        px4_process,
        rviz_node,
    ]

    return nodes_to_start

def generate_launch_description():
    current_package_path = get_package_share_directory('uwb_sim')
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'px4_src_path',
            default_value='/home/user/PX4Swarm',
            description='px4 source code path'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'anchor_conf_path',
            default_value=f'{current_package_path}/confs/anchors_3.yaml',
            description='anchors position configuration file path'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_type',
            default_value='iris',
            description='Type of the robot to spawn'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'world_type',
            default_value='empty',
            description='Type of the world to spawn'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
