import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50', 
        executable='dvl_a50_sensor', 
        parameters=[
            {'dvl_ip_address': "192.168.194.95"},
            {'position_frame_id': "map"},
            {'velocity_frame_id': "base_link"},
            {'altitude_frame_id': "pool_bottom"},
        ],
        output='screen')


    return launch.LaunchDescription([
        dvl_a50,
    ])


       
