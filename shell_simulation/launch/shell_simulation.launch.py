from launch import LaunchDescription
from launch_ros.actions import Node

# Example ROS launch file
def generate_launch_description():
    
    example_control = Node(
        package='shell_simulation',
        namespace='',
        executable='example_control',
        name='example_control',
        output='screen'
    )
    
    return LaunchDescription([  
        # Nodes
        example_control
    ])