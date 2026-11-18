#!/usr/bin/env python3
"""Per-goal convergence/admission tolerance never leaks to the next goal."""

import os
import sys

from ats_navigation_interfaces.action import NavigateToPose
from ats_navigation_interfaces.msg import LocalizationStatus, PlanningMapStatus
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as harness  # noqa: E402


class TestGoalTolerance(harness.TestGoalManagerLocalizationEpoch):
    robot_x = 3.94

    @classmethod
    def setUpClass(cls):
        harness.TOPIC_PREFIX = "/s3_goal_tolerance_contract"
        cls.extra_parameters = {
            "require_gimbal_status": False,
            "goal_position_tolerance": 0.03,
            "max_goal_position_tolerance": 0.10,
            "terminal_dwell_sec": 0.05,
        }
        super().setUpClass()

    @classmethod
    def health(cls):
        cls.publish_tf(0.0)
        stamp = cls.node.get_clock().now().to_msg()
        pose = Odometry()
        pose.header.stamp, pose.header.frame_id = stamp, "odom"
        pose.pose.pose.position.x = cls.robot_x
        pose.pose.pose.orientation.w = 1.0
        cls.odometry_pub.publish(pose)
        localization = LocalizationStatus()
        localization.header.stamp, localization.header.frame_id = stamp, "map"
        localization.state, localization.epoch = LocalizationStatus.STATE_TRACKING, 1
        localization.last_observation_stamp = stamp
        cls.localization_pub.publish(localization)
        status = PlanningMapStatus()
        status.header.stamp, status.header.frame_id = stamp, "odom"
        status.ready, status.localization_epoch = True, 1
        status.rog_generation, status.publication_sequence = 1, 1
        cls.map_status_pub.publish(status)
        cls.publish_planning_snapshot(1, 1)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered by test_goal_manager_epoch")

    def test_requested_tolerance_is_scoped_to_one_goal(self):
        client = ActionClient(self.node, NavigateToPose,
                              f"{harness.TOPIC_PREFIX}/navigate_to_pose")
        self.assertTrue(client.wait_for_server(timeout_sec=5.0))

        def send(tolerance):
            goal = NavigateToPose.Goal()
            goal.goal_pose.header.frame_id = "map"
            goal.goal_pose.pose.position.x = 4.0
            goal.goal_pose.pose.orientation.w = 1.0
            goal.position_tolerance = tolerance
            goal.timeout.sec = 30
            future = client.send_goal_async(goal)
            self.assertTrue(self.spin_until(future.done, periodic=self.health))
            return future.result()

        def dispatch(tolerance):
            goal_count = len(self.planner_goals)
            handle = send(tolerance)
            self.assertTrue(handle.accepted)
            reference_count = len(self.references)
            self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > goal_count,
                                           periodic=self.health))
            request = self.planner_goals[-1]
            self.publish_candidate(request, 1)
            self.assertTrue(self.spin_until(lambda: len(self.references) > reference_count,
                                           periodic=self.health))
            return handle, request

        try:
            handle, request = dispatch(0.08)
            self.assertAlmostEqual(request.position_tolerance, 0.08)
            result = handle.get_result_async()
            self.assertTrue(self.spin_until(result.done, periodic=self.health))
            self.assertEqual(result.result().result.result_code, NavigateToPose.Result.RESULT_SUCCEEDED)
            self.assertAlmostEqual(result.result().result.final_distance, 0.06, places=3)

            strict, request = dispatch(0.0)
            self.assertEqual(request.position_tolerance, 0.0, "previous override leaked")
            strict_result = strict.get_result_async()
            self.spin_until(lambda: False, timeout=0.25, periodic=self.health)
            self.assertFalse(strict_result.done(), "default 3 cm goal accepted a 6 cm endpoint")
            type(self).robot_x = 3.98
            self.assertTrue(self.spin_until(strict_result.done, periodic=self.health))
            self.assertEqual(strict_result.result().result.result_code,
                             NavigateToPose.Result.RESULT_SUCCEEDED)

        finally:
            client.destroy()
