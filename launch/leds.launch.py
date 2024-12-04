from launch import LaunchDescription

import launch_ros.actions



def generate_launch_description():

    ld = LaunchDescription()

    leds_node = launch_ros.actions.Node(
        package='turtlebro',
        namespace='',
        executable='leds.py',
        name='leds'
    )

    ld.add_action(leds_node)

        
    return ld    