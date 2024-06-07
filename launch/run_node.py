from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
           
            launch_ros.actions.Node(
            namespace= "uto_per_mot", package='uto_per_mot', executable='uto_per_mot', output='screen'),
            launch_ros.actions.Node(
            namespace= "uto_per_fs", package='uto_per_fs', executable='uto_per_fs', output='screen'),
            launch_ros.actions.Node(
            namespace= "uto_per_pred", package='uto_per_pred', executable='uto_per_pred', output='screen'),
            launch_ros.actions.Node(
            namespace= "uto_per_psf", package='uto_per_psf', executable='uto_per_psf', output='screen'),
            launch_ros.actions.Node(
            namespace= "uto_per_tsr", package='uto_per_tsr', executable='uto_per_tsr', output='screen'),
            launch_ros.actions.Node(
            namespace= "uto_per_vme", package='uto_per_vme', executable='uto_per_vme', output='screen'),
    ])
