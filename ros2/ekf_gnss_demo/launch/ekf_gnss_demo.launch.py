# SPDX-License-Identifier: Apache-2.0
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    scenario = LaunchConfiguration("scenario")
    seed = LaunchConfiguration("seed")
    replay_rate_hz = LaunchConfiguration("replay_rate_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario", default_value="matched"),
            DeclareLaunchArgument("seed", default_value="1"),
            DeclareLaunchArgument("replay_rate_hz", default_value="100.0"),
            Node(
                package="ekf_gnss_demo",
                executable="sim_sensor_node",
                name="sim_sensor_node",
                output="screen",
                parameters=[
                    {
                        "scenario": scenario,
                        "seed": seed,
                        "replay_rate_hz": replay_rate_hz,
                    }
                ],
            ),
            Node(
                package="ekf_gnss_demo",
                executable="ekf_gnss_demo_node",
                name="ekf_gnss_demo_node",
                output="screen",
                parameters=[
                    {
                        "scenario": scenario,
                        "seed": seed,
                        "replay_rate_hz": replay_rate_hz,
                    }
                ],
            ),
        ]
    )
