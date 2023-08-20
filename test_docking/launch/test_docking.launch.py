from launch import LaunchDescription
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    charger_description_dir = get_package_share_directory('charger_description')

    charger_description = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                charger_description_dir + '/launch/view_charger.launch.py'))

    realsense2_camera_dir = get_package_share_directory('realsense2_camera')

    realsense2_camera = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                realsense2_camera_dir + '/launch/rs_launch.py'))


    apriltag_ros_dir = get_package_share_directory('apriltag_ros')

    apriltag_ros = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                apriltag_ros_dir + '/launch/tag_realsense.launch.py'))
    
    aptags_tf_broadcast_dir = get_package_share_directory('aptags_tf_broadcast')

    aptags_tf_broadcast = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
                aptags_tf_broadcast_dir + '/launch/tf_broadcast.launch.py'))
    
    return LaunchDescription([
        realsense2_camera,
        apriltag_ros,
        aptags_tf_broadcast,
        charger_description,
        Node(
            package='test_docking',
            executable='apriltag_port',
        ),
    ])