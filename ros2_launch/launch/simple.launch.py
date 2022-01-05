import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions\
#import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    foo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                foo_dir + '/launch/empty_world.launch.py'
            )
        ),
        launch_ros.actions.Node(
            package='telegram_pub',
            executable='telegram_pub',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='high_level_controller',
            executable='minimal_publisher',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='ros2_launch',
            executable='exercise_3',
            output='screen',
        ),


    ])
