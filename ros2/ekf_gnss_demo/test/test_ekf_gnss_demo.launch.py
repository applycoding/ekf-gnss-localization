# SPDX-License-Identifier: Apache-2.0
import os
import unittest

import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing_ros import WaitForTopics


def generate_test_description():
  launch_file = os.path.join(
      get_package_share_directory("ekf_gnss_demo"),
      "launch",
      "ekf_gnss_demo.launch.py",
  )
  return launch.LaunchDescription(
      [
          IncludeLaunchDescription(
              PythonLaunchDescriptionSource(launch_file),
              launch_arguments={"replay_rate_hz": "500.0"}.items(),
          ),
          WaitForTopics(
              ["/ekf_gnss/odometry", "/ekf_gnss/pose", "/sim/imu"],
              timeout=60.0,
          ),
          launch_testing.actions.ReadyToTest(),
      ]
  )


@launch_testing.post_shutdown_test()
class TestEkfGnssDemoShutdown(unittest.TestCase):
  def test_exit_codes(self, proc_info):
    launch_testing.asserts.assertExitCodes(proc_info)
