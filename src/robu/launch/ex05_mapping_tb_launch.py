import os
import launch
import launch_ros.actions
from launch.actions.execute_process import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config_dir = os.path.join(get_package_share_directory('robu'),
                                   'rviz', 'tb3_cartographer.rviz')
    #print([ThisLaunchFileDir(), '/occupancy_grid.launch.py'])
    #ros2 run rviz2 rviz2 -d C:\ws\ws_robu\install\share\robu\rviz\tb3_cartographer.rviz --ros-args -p "use_sim_time:=false"

    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='2'),
        launch.actions.SetEnvironmentVariable(name='LDS_MODEL', value='LDS-02'),
        launch.actions.SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('turtlebot3_bringup'), '/launch/robot.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('robu'), '/cartographer.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        
        launch_ros.actions.Node(
            package='robu',
            executable='camera',
            name='camera'
            ),

        # launch_ros.actions.Node(
        #     package='robu',
        #     executable='wiimotectrl',
        #     name='wiimotectrl',
        #     remappings=[                        #Topic/Service remapping
        #         ('/cmd_vel', '/cmd_vel_raw'),
        #     ]),
        launch_ros.actions.Node(
            package='robu',
            executable='oas',
            name='obstacle_avoidance_simple'
            )

        # launch_ros.actions.Node(
        #     package='robu',
        #     executable='remotectrl',
        #     name='remotectrl'
        #     ),

        # launch_ros.actions.Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
  ])
