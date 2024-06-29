import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

 
def generate_launch_description():

    remappings = [  ('/tf_static', 'tf_static'), 
                    ('/tf', 'tf')]
    
    #Declare arguments
    declare_arg_namespace = DeclareLaunchArgument('namespace',
        default_value='robot1',
        description='Host Name / Namespace')

    # Create Launch configuratios
    namespace = LaunchConfiguration('namespace')
 
    start_recovery_pathfinder_node = Node(
            package='recovery_pathfinder',
            executable='pulse_monitor',
            name='recovery_pathfinder',
            namespace=namespace,
            output="screen",
            remappings= remappings,
            emulate_tty=True,
        )
    
    ld = LaunchDescription()
    ld.add_action(declare_arg_namespace)
    ld.add_action(start_recovery_pathfinder_node)
    
    return ld
