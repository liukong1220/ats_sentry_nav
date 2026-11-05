#!/usr/bin/env python3
"""Planner endpoint relocation is independent of map-to-odom goal drift."""
from copy import deepcopy
import os
import sys

from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as harness  # noqa: E402


class TestRetarget(harness.TestGoalManagerLocalizationEpoch):
    offset = 0.0

    @classmethod
    def setUpClass(cls):
        harness.TOPIC_PREFIX = "/s3_relocated_endpoint_contract"
        cls.extra_parameters = {
            "planner_start_in_planning_frame": True,
            "require_gimbal_status": False,
            "replan_without_stop_enabled": True,
            "retarget_threshold": 0.03,
        }
        super().setUpClass()

    @classmethod
    def health(cls):
        cls.publish_health(1, cls.offset)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered unchanged by test_goal_manager_epoch")

    def test_relocated_endpoint_does_not_retarget_until_transform_changes(self):
        self.assertTrue(self.spin_until(
            lambda: self.goal_pub.get_subscription_count() == 1, periodic=self.health))
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.orientation.w = 4.0, 1.0
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: bool(self.planner_goals), periodic=self.health))
        first = self.planner_goals[-1]
        # MINCO may relocate the terminal pose inside its map cell while
        # preserving the canonical goal and request identity.
        relocated = deepcopy(first)
        relocated.goal_pose.pose.position.x -= 0.04
        self.publish_candidate(relocated, 1)
        self.assertTrue(self.spin_until(lambda: len(self.references) == 1, periodic=self.health))
        count = len(self.planner_goals)
        self.spin_until(lambda: False, timeout=0.4, periodic=self.health)
        self.assertEqual(len(self.planner_goals), count,
                         "fixed planner relocation was mistaken for transform drift")

        type(self).offset = 0.15
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > count, periodic=self.health))
        replacement = self.planner_goals[-1]
        self.assertAlmostEqual(replacement.goal_pose.pose.position.x, 3.85, places=3)
        relocated = deepcopy(replacement)
        relocated.goal_pose.pose.position.x -= 0.04
        self.publish_candidate(relocated, 1)
        self.assertTrue(self.spin_until(lambda: len(self.references) == 2, periodic=self.health))
        count = len(self.planner_goals)
        self.spin_until(lambda: False, timeout=0.4, periodic=self.health)
        self.assertEqual(len(self.planner_goals), count,
                         "replacement did not advance the committed transform baseline")
        self.publish_candidate(first, 1)
        self.spin_until(lambda: False, timeout=0.2, periodic=self.health)
        self.assertEqual(len(self.references), 2, "old pre-retarget candidate revived")
