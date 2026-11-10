# SPDX-License-Identifier: Apache-2.0
"""Launch test: full stack + RMSE checker (Humble active + post-shutdown)."""

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


def generate_test_description() -> tuple[launch.LaunchDescription, dict]:
    share = get_package_share_directory("ekf_gnss_demo")
    launch_file = os.path.join(share, "launch", "ekf_gnss_demo.launch.py")
    checker = os.path.join(os.path.dirname(os.path.realpath(__file__)), "check_odom_rmse.py")
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"
    check_odom_rmse = ExecuteProcess(
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
        env=proc_env,
        output="screen",
        name="check_odom_rmse",
    )
    # Keeps the launch alive until the active tests finish. Without it the
    # launch can end as soon as the nodes exit, and launch_testing then fails
    # with "Processes under test stopped before tests completed".
    keepalive = ExecuteProcess(cmd=["sleep", "300"], name="keepalive", output="log")
    return launch.LaunchDescription(
        [
            keepalive,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_file),
                launch_arguments={
                    "scenario": "matched",
                    "seed": "1",
                    "replay_rate_hz": "500.0",
                }.items(),
            ),
            TimerAction(period=3.0, actions=[check_odom_rmse]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"check_odom_rmse": check_odom_rmse}


class TestRmseCheckerActive(unittest.TestCase):
    """Runs while sim, EKF, and checker are live."""

    def test_rmse_line_on_stdout(self, proc_output, check_odom_rmse) -> None:
        proc_output.assertWaitFor(
            "RMSE horizontal:",
            process=check_odom_rmse,
            timeout=120.0,
            stream="stdout",
        )

    def test_checker_shuts_down(self, proc_info, check_odom_rmse) -> None:
        proc_info.assertWaitForShutdown(process=check_odom_rmse, timeout=120.0)


@launch_testing.post_shutdown_test()
class TestEkfGnssLaunch(unittest.TestCase):
    def test_checker_exit_code(self, proc_info, check_odom_rmse) -> None:
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=check_odom_rmse,
            allowable_exit_codes=[0],
        )
