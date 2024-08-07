import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
<<<<<<< HEAD
            arguments=['-d', '/home/cyril/dev_ws/src/my_bot/config/my_rv.rviz']  # Update this path to your actual config file
=======
            arguments=['-d', '/home/aleena/abcs/src/my_bot/config/my_rv.rviz']  # Update this path to your actual config file
>>>>>>> 294963a (updated all)
        ),
    ])
