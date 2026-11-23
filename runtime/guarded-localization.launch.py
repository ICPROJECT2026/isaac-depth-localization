"""Launch upstream localization using generated local paths and shared tuning."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent
    names = ['localization', 'global-search', 'global-startup', 'global-recovery']
    for name in names:
        if not (root/f'{name}.local.yaml').is_file():
            raise RuntimeError('Missing local configuration. Run scripts/configure.py.')
    core = Node(package='lidar_localization_ros2', executable='lidar_localization_node',
                parameters=[str(root/'localization.local.yaml')],
                remappings=[('cloud', '/depth/points'), ('odom', '/wheel/odom')], output='screen')
    activate = ExecuteProcess(cmd=['ros2', 'run', 'lidar_localization_ros2',
        'start_lifecycle_node.py', '/lidar_localization', '--timeout', '120'], output='screen')
    nodes = [core, activate]
    for executable, params in [('global_localization_node.py', 'global-search'),
        ('startup_initialization_node.py', 'global-startup'),
        ('reinitialization_supervisor_node.py', 'global-recovery')]:
        nodes.append(Node(package='lidar_localization_ros2', executable=executable,
                          parameters=[str(root/f'{params}.local.yaml')], output='screen'))
    return LaunchDescription(nodes)
