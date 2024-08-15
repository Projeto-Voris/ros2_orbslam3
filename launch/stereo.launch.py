from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.substitutions import LaunchConfiguration as LaunchConfig

def generate_launch_description():
    return LaunchDescription([
        LaunchArg(
            'left_image',
            default_value=['/left/image_raw']
        ),
        LaunchArg(
            'right_image',
            default_value=['/right/image_raw']
        ),
        Node(
            package='ros2_orbslam3',
            namespace='/sm2/debug',
            executable='stereo',
            name='stereo',
            arguments= [["/ws/src/ros2_orbslam3/vocabulary/ORBvoc.txt"], ["/ws/src/ros2_orbslam3/config/stereo/CALIBRED22072024.yaml"],["True"]],
            remappings=[
                ('/left/image_raw',  LaunchConfig('left_image')),
                ('/right/image_raw', LaunchConfig('right_image'))
            ]
        )

    ])

