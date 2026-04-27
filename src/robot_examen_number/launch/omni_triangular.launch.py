from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
 
    pkg = get_package_share_directory('robot_examen_number')
 
    urdf_file   = os.path.join(pkg, 'model', '#####')
    rviz_config = os.path.join(pkg, 'rviz',  '#####')
 
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
 
    # ── Argumentos desde CLI ──────────────────────────────────────────────
    goal_x_arg = DeclareLaunchArgument(
        'goal_x', default_value='0.0',
        description='Coordenada X del punto objetivo [m]'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y', default_value='0.0',
        description='Coordenada Y del punto objetivo [m]'
    )
    goal_theta_arg = DeclareLaunchArgument(        
        'goal_theta', default_value='0.0',
        description='Orientación final del robot en grados X'
    )
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius', default_value='0.',
        description='Radio de las ruedas [m]'
    )
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius', default_value='0.0',
        description='Circunradio del triángulo [m]'
    )
    wheel_width_arg = DeclareLaunchArgument(
        'wheel_width', default_value='0.0',
        description='Ancho de la rueda [m]'
    )
 
    # ── Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
 
    # ── Simulador omni triangular ─────────────────────────────────────────
    simulator_node = Node(
        package='robot_examen_number',
        executable='omni_triangular_simulator',
        name='omni_triangular_simulator',
        output='screen',
        parameters=[{
            'wheel_radius':  LaunchConfiguration('wheel_radius'),
            'robot_radius':  LaunchConfiguration('robot_radius'),
            'update_rate':   50.0,
            'max_wheel_vel': 5.0,
        }]
    )
 
    # ── Goto point ────────────────────────────────────────────────────────
    goto_node = Node(
        package='robot_examen_number',
        executable='goto_point',
        name='goto_point',
        output='screen',
        parameters=[{
            'goal_x':          LaunchConfiguration('goal_x'),
            'goal_y':          LaunchConfiguration('goal_y'),
            'goal_theta':      LaunchConfiguration('goal_theta'),
            'kp_linear':       0.0,
            'kp_angular':      0.0,
            'dist_tolerance':  0.05,
            'angle_tolerance': 0.05,
            'max_linear_vel':  0.4,
            'max_angular_vel': 1.2,
        }]
    )
 
    # ── RViz ──────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
 
    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        goal_theta_arg,        
        wheel_radius_arg,
        robot_radius_arg,
        wheel_width_arg,
        robot_state_publisher,
        simulator_node,
        goto_node,
        rviz_node,
    ])
