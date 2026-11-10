# SPDX-License-Identifier: Apache-2.0
"""Launch test: live sim topics through ekf_gnss_demo_node; RMSE checker exits non-zero on failure."""

from __future__ import annotations

import os
import sys
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_test_description() -> launch.LaunchDescription:
    share = get_package_share_directory("ekf_gnss_demo")
    launch_file = os.path.join(share, "launch", "ekf_gnss_demo.launch.py")
    checker = os.path.join(os.path.dirname(os.path.realpath(__file__)), "check_odom_rmse.py")
    checker_proc = ExecuteProcess(
        cmd=[
            sys.executable,
            checker,
            "--min-messages",
            "4000",
            "--rmse-max-m",
            "1.0",
            "--timeout-s",
            "90.0",
        ],
        output="screen",
    )
    return launch.LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "scenario": "matched",
                    "seed": "1",
                    "replay_rate_hz": "500.0",
                }.items(),
            ),
            TimerAction(period=3.0, actions=[checker_proc]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


@launch_testing.post_shutdown_test()
class TestEkfGnssLaunch(unittest.TestCase):
    def test_checker_exit_code(self, proc_info: launch_testing.ProcInfoHandler) -> None:
        checker_procs = [
            p
            for p in proc_info.processes
            if any("check_odom_rmse.py" in str(arg) for arg in p.args)
        ]
        self.assertTrue(checker_procs, "RMSE checker process not found")
        for proc in checker_procs:
            self.assertEqual(
                proc.returncode,
                0,
                msg=f"check_odom_rmse.py failed with code {proc.returncode}",
            )

    def test_demo_nodes_exit_cleanly(self, proc_info: launch_testing.ProcInfoHandler) -> None:
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0])
