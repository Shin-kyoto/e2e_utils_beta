import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('dummy_publisher')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'awsim_config_sample.yaml'),
        description='Path to the AWSIM configuration YAML file'
    )
    
    # Create the process using ExecuteProcess to run the Python script directly
    awsim_topic_pub_process = ExecuteProcess(
        cmd=['python3', '-m', 'dummy_publisher.awsim_topic_pub', LaunchConfiguration('config_file')],
        output='screen',
        name='awsim_topic_publisher'
    )
    
    return LaunchDescription([
        config_file_arg,
        awsim_topic_pub_process
    ])
