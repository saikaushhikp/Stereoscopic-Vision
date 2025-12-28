"""
Launch file for stereoscopic vision pipeline.
Launches rectification, disparity, depth estimation, and 3D reconstruction nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Generate launch description for stereo pipeline."""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='stereo.yaml',
        description='Path to stereo calibration YAML file'
    )
    
    # Get configuration values
    use_sim_time = LaunchConfiguration('use_sim_time')
    calibration_file = LaunchConfiguration('calibration_file')

    return LaunchDescription([
        use_sim_time_arg,
        calibration_file_arg,

        # Rectification Node - Rectifies left and right images
        Node(
            package='stereo_perception',
            executable='rectification_node',
            name='rectification_node',
            namespace='stereo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'calibration_file': calibration_file}
            ],
            remappings=[
                ('/stereo/left/image_raw', '/left/image_raw'),
                ('/stereo/right/image_raw', '/right/image_raw'),
                ('/stereo/left/image_rect', '/left/image_rect'),
                ('/stereo/right/image_rect', '/right/image_rect'),
            ]
        ),

        # Disparity Node - Computes disparity map from rectified images
        Node(
            package='stereo_perception',
            executable='disparity_node',
            name='disparity_node',
            namespace='stereo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('/stereo/left/image_rect', '/left/image_rect'),
                ('/stereo/right/image_rect', '/right/image_rect'),
                ('/stereo/disparity', '/disparity'),
            ]
        ),

        # Depth Node - Converts disparity to depth map
        Node(
            package='stereo_perception',
            executable='depth_node',
            name='depth_node',
            namespace='stereo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'calibration_file': calibration_file}
            ],
            remappings=[
                ('/stereo/disparity', '/disparity'),
                ('/stereo/depth', '/depth'),
            ]
        ),

        # PointCloud Node - Generates 3D point cloud from depth map
        Node(
            package='stereo_mapping',
            executable='pointcloud_node',
            name='pointcloud_node',
            namespace='stereo',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'calibration_file': calibration_file}
            ],
            remappings=[
                ('/stereo/depth', '/depth'),
                ('/stereo/points', '/points'),
            ]
        )
    ])


if __name__ == "__main__":
    generate_launch_description()