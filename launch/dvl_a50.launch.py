import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    launch.actions.DeclareLaunchArgument('ip_address', default_value='192.168.194.95'),
    launch.actions.DeclareLaunchArgument('position_frame_id', default_value="map"),
    launch.actions.DeclareLaunchArgument('velocity_frame_id', default_value="base_link"),
    launch.actions.DeclareLaunchArgument('altitude_frame_id', default_value="pool_bottom"),
    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[
            {'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address')},
            {'position_frame_id': launch.substitutions.LaunchConfiguration('position_frame_id')},
            {'velocity_frame_id': launch.substitutions.LaunchConfiguration('velocity_frame_id')},
            {'altitude_frame_id': launch.substitutions.LaunchConfiguration('altitude_frame_id')},
        ],
        output='screen')


    return launch.LaunchDescription([
        dvl_a50,
    ])


       
