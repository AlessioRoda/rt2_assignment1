import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    container = ComposableNodeContainer(
        name='rt2_assignment1',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
        ComposableNode(
            package='rt2_assignment1',
            plugin='rt2_assignment1::RandomServer',
            name='position_service_component'),
        
        ComposableNode(
            package='rt2_assignment1',
            plugin='rt2_assignment1::StateMachine',
            name='state_machine_component')
    ],
     output='screen',
    )

    return launch.LaunchDescription([container])
