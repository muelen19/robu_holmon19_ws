import launch
import launch_ros.actions
from launch.actions.execute_process import ExecuteProcess

def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable(name='ROS_DOMAIN_ID', value='6'),
        launch.actions.SetEnvironmentVariable(name='LDS_MODEL', value='LDS-02'),
        launch.actions.SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger'),
        
        # ExecuteProcess(
        #     name='bringup',
        #     cmd=['ssh -t pi@robpi "bash -i -c \'systemctl stop --user remote_ros_robot_launch.service\'"'],
        #     output="screen",
        #     shell=True,
        #     emulate_tty=True
        # )
        # ,        
        ExecuteProcess(
            name='bringup',
            cmd=['ssh -t pi@robpi6 "bash -i -c \'systemctl start --user turtlebot_bringup_launch.service\'"'],
            output="screen",
            shell=True,
            emulate_tty=True
        )
        ,
        # ExecuteProcess(
        #     name='bringup',
        #     cmd=['ssh -t pi@robpi "bash -i -c \'. /home/pi/work/ws_robu/src/robu/launch/turtlebot_bringup_launch.sh\'"'],
        #     output="screen",
        #     shell=True,
        #     emulate_tty=True
        # ),
        #ssh -t pi@robpi "bash -i -c '. /home/pi/work/ws_robu/src/robu/launch/turtlebot_bringup_launch.sh'"
        launch_ros.actions.Node(
            package='robu',
            executable='remotectrl_sus',
            name='remotectrl_sus',
            #output='screen',
            #emulate_tty=True,
            remappings=[                        #Topic/Service remapping
                ('/cmd_vel', '/cmd_vel_raw'),
            ]
            ),
        launch_ros.actions.Node(
            package='robu',
            executable='ex03_obstacle_avoidance_simple',
            name='ex03_obstacle_avoidance_simple'
            )
  ])
