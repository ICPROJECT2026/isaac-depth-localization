"""Configuration-only launch of upstream ros2_control controllers."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parent
    manager=Node(package='controller_manager',executable='ros2_control_node',
                 parameters=[str(root.parent/'config/controllers.yaml'),{'robot_description':(root/'robot.urdf').read_text(),'use_sim_time':True}],
                 remappings=[('/diff_drive_controller/cmd_vel_unstamped','/cmd_vel'),('/diff_drive_controller/odom','/wheel/odom')],output='screen')
    spawn=Node(package='controller_manager',executable='spawner',
               arguments=['diff_drive_controller','--controller-manager','/controller_manager','--controller-manager-timeout','60','--service-call-timeout','60','--switch-timeout','60'],output='screen')
    feedback=Node(package='controller_manager',executable='spawner',
                  arguments=['joint_state_broadcaster','--controller-manager','/controller_manager','--controller-manager-timeout','60','--service-call-timeout','60','--switch-timeout','60'],output='screen')
    axle_to_body=Node(package='tf2_ros',executable='static_transform_publisher',
                     name='rear_axle_to_base_link',
                     arguments=['--x','0.12454909547','--y','-0.00016280464','--z','0',
                                '--frame-id','rear_axle','--child-frame-id','base_link'],
                     parameters=[{'use_sim_time':True}],output='screen')
    reference=ExecuteProcess(cmd=['python3',str(root/'wheel-odom-reference.py'),'--ros-args','-p','use_sim_time:=true'],output='screen')
    return LaunchDescription([axle_to_body,manager,reference,TimerAction(period=2.,actions=[spawn,feedback])])
