from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    lc = LaunchContext()

    turtlebro_shared_path  = get_package_share_path('turtlebro')
    
    robot_model = EnvironmentVariable("ROBOT_MODEL", default_value="turtlebro2")

    robot_model_path = f'{turtlebro_shared_path}/urdf/{robot_model.perform(lc)}.urdf'
    
    model_arg = DeclareLaunchArgument(name='model', default_value=robot_model_path,
                                      description='Absolute path to robot urdf file')

    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )


    return LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
    ])
