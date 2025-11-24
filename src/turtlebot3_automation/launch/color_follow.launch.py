from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare('turtlebot3_automation')
    color_config = PathJoinSubstitution([package_share, 'config', 'color_follow.yaml'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    target_color = LaunchConfiguration('target_color')

    declarations = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('target_color', default_value='red'),
    ]

    node = Node(
        package='turtlebot3_automation',
        executable='color_follow',
        name='color_follow',
        parameters=[color_config, {'use_sim_time': use_sim_time, 'target_color': target_color}],
        output='screen',
    )

    return LaunchDescription(declarations + [node])
