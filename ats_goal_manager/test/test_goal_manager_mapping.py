#!/usr/bin/env python3
# Copyright 2026
"""S3: first accepted registration and current-transform endpoint retargeting."""
import os
import sys
import unittest

from ats_navigation_interfaces.msg import LocalizationStatus, PlanningMapStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as harness  # noqa: E402


class TestMappingGoals(harness.TestGoalManagerLocalizationEpoch):
    accepted = False
    offset = 0.0
    robot_x = 0.0

    @classmethod
    def setUpClass(cls):
        harness.TOPIC_PREFIX = "/s3_goal_contract"
        cls.extra_parameters = {
            "require_accepted_localization": True,
            "planner_start_in_planning_frame": True,
            "require_gimbal_status": False,
            "replan_without_stop_enabled": True,
            "retarget_threshold": 0.03,
            "progress_hold_distance_m": 0.20,
        }
        super().setUpClass()

    @classmethod
    def health(cls):
        cls.publish_tf(cls.offset)
        stamp = cls.node.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp, odometry.header.frame_id = stamp, "odom"
        odometry.pose.pose.position.x = cls.robot_x
        odometry.pose.pose.orientation.w = 1.0
        cls.odometry_pub.publish(odometry)
        status = LocalizationStatus()
        status.header.stamp, status.header.frame_id = stamp, "map"
        status.state, status.epoch = LocalizationStatus.STATE_TRACKING, 1
        if cls.accepted:
            status.last_observation_stamp = stamp
        cls.localization_pub.publish(status)
        map_status = PlanningMapStatus()
        map_status.header.stamp, map_status.header.frame_id = stamp, "odom"
        map_status.ready, map_status.localization_epoch = True, 1
        map_status.rog_generation, map_status.publication_sequence = 1, 1
        cls.map_status_pub.publish(map_status)
        cls.publish_planning_snapshot(1, 1)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered unchanged by test_goal_manager_epoch")

    def test_first_acceptance_and_endpoint_drift_use_latest_transform(self):
        self.assertTrue(self.spin_until(
            lambda: self.goal_pub.get_subscription_count() == 1, periodic=self.health))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.orientation.w = 4.0, 1.0
        self.goal_pub.publish(goal)
        self.spin_until(lambda: False, timeout=0.35, periodic=self.health)
        self.assertFalse(self.planner_goals, "goal escaped before the first accepted registration")
        type(self).accepted = True
        self.assertTrue(self.spin_until(lambda: bool(self.planner_goals), periodic=self.health))
        first = self.planner_goals[-1]
        self.assertEqual(first.start_pose.header.frame_id, "odom")
        self.publish_candidate(first, 1)
        self.assertTrue(self.spin_until(lambda: bool(self.references), periodic=self.health))
        self.assertTrue(self.spin_until(lambda: False in self.stop_states, periodic=self.health))
        release = len(self.stop_states)
        count = len(self.planner_goals)
        type(self).robot_x = 3.94
        type(self).offset = 0.15
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > count, periodic=self.health))
        replacement = self.planner_goals[-1]
        self.assertEqual(replacement.goal_id, first.goal_id)
        self.assertGreater(replacement.plan_request_sequence, first.plan_request_sequence)
        self.assertAlmostEqual(replacement.goal_pose.pose.position.x, 3.85, places=3)
        self.publish_candidate(replacement, 1)
        self.assertTrue(self.spin_until(lambda: len(self.references) == 2, periodic=self.health))
        self.assertNotIn(True, self.stop_states[release:], "ordinary endpoint retarget forced a stop")
        references = len(self.references)
        self.publish_candidate(first, 1)
        self.spin_until(lambda: False, timeout=0.25, periodic=self.health)
        self.assertEqual(len(self.references), references, "obsolete pre-retarget candidate revived")


if __name__ == "__main__":
    unittest.main()
