from launch import LaunchDescription

import launch.actions  
import launch.events 
import launch_ros.actions
import launch_ros.events 
import launch_ros.events.lifecycle  

import lifecycle_msgs.msg 

def generate_launch_description():
    ld = LaunchDescription()

    leds_node = launch_ros.actions.LifecycleNode(
        name='leds_lifecycle', namespace='',
        package='turtlebro', executable='leds_lifecycle.py', output='screen')
    
    leds_node_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(leds_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    reaches_configure_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=leds_node, goal_state='configuring',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'talker' reached the 'configure' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(leds_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )  
    # start_state= 'unconfigured',
    # goal_state='configuring',


    ld.add_action(leds_node)
    ld.add_action(leds_node_configure_transition)
    ld.add_action(reaches_configure_state)

        
    return ld    