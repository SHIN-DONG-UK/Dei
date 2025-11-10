#!/usr/bin/env python3
"""
llm_pkg의 노드들을 동시에 실행하는 launch 파일입니다.
이 파일은 wakeup_node.py, stream_stt_node.py, data_subscription_node.py를 실행합니다.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # (선택사항) 파라미터 설정 - 예를 들어 use_sim_time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # wakeup_node 실행
        Node(
            package='llm_pkg',
            executable='wakeup_node',  # setup.py에 등록한 executable 이름과 일치해야 함
            name='wakeup_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # stream_stt_node 실행
        Node(
            package='llm_pkg',
            executable='stream_stt_node',
            name='stream_stt_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
        # data_subscription_node 실행
        Node(
            package='llm_pkg',
            executable='daya_subscription_node',
            name='daya_subscription_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
