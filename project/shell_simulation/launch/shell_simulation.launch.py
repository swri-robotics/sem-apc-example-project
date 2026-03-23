from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter

# Example ROS launch file
def generate_launch_description():
    
    # This sets use_sim_time=True for all subsequent nodes in this launch file, 
    # which is required for correct time synchronization with the CARLA server 
    # when using ROS time.
    SetParameter(name='use_sim_time', value=True),
    
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