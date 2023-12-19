from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='iwr6843aop_pub',
            executable='pcl_pub',
            arguments=["/dev/ttyMmr1Cmd", "/dev/ttyMmr1Data", "iwr6843_frame_1", "iwr6843_pcl_1"]
        ),
        
        Node(
            package='iwr6843aop_pub',
            executable='pcl_pub',
            arguments=["/dev/ttyMmr2Cmd", "/dev/ttyMmr2Data", "iwr6843_frame_2", "iwr6843_pcl_2"]
        ),
        Node(
            package='iwr6843aop_pub',
            executable='pcl_pub',
            arguments=["/dev/ttyMmr3Cmd", "/dev/ttyMmr3Data", "iwr6843_frame_3", "iwr6843_pcl_3"]
        ),
        Node(
            package='iwr6843aop_pub',
            executable='pcl_pub',
            arguments=["/dev/ttyMmr4Cmd", "/dev/ttyMmr4Data", "iwr6843_frame_4", "iwr6843_pcl_4"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0.076", "0.076", "0", "0", "0", "map", "iwr6843_frame_1"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0.076", "0", "0.076", "4.7123", "0", "0", "map", "iwr6843_frame_2"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "-0.076", "0.076", "3.1415", "0", "0", "map", "iwr6843_frame_3"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["-0.076", "0", "0.076", "1.5708", "0", "0", "map", "iwr6843_frame_4"]
        )
    ])
