#!/usr/bin/env python3
# Copyright 2026
"""The production node must not commit on wall-time expiry of a frozen ROS clock."""
import os
import sys

from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as harness  # noqa: E402


class TestStopBarrier(harness.TestGoalManagerLocalizationEpoch):
    @classmethod
    def setUpClass(cls):
        harness.TOPIC_PREFIX = "/stop_reference_barrier_test"
        cls.extra_parameters = {"use_sim_time": True, "require_gimbal_status": False}
        super().setUpClass()
        cls.node.set_parameters([Parameter("use_sim_time", value=True)])
        cls.clock_pub = cls.node.create_publisher(Clock, "/clock", 10)
        cls.sim_ns = 10_000_000_000

    @classmethod
    def health(cls):
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = divmod(cls.sim_ns, 1_000_000_000)
        cls.clock_pub.publish(clock)
        cls.publish_health(1, 0.0)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered by test_goal_manager_epoch")

    def test_frozen_clock_does_not_release_replacement_after_stop(self):
        self.assertTrue(self.spin_until(
            lambda: self.node.get_clock().now().nanoseconds == self.sim_ns and
            self.goal_pub.get_subscription_count() == 1, periodic=self.health))
        self.spin_until(lambda: False, timeout=0.3, periodic=self.health)
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.orientation.w = 4.0, 1.0
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: bool(self.planner_goals), periodic=self.health))
        self.publish_candidate(self.planner_goals[-1], 1)
        # Covers a STOP on initial dispatch as well as startup STOP at time zero.
        type(self).sim_ns += 20_000_000
        self.assertTrue(self.spin_until(lambda: bool(self.references), periodic=self.health))
        self.assertTrue(self.spin_until(lambda: False in self.stop_states, periodic=self.health))

        type(self).sim_ns = 11_000_000_000
        self.spin_until(lambda: False, timeout=0.3, periodic=self.health)
        count, goals = len(self.references), len(self.planner_goals)
        goal.pose.position.x = 5.0
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > goals,
                                        periodic=self.health))
        self.publish_candidate(self.planner_goals[-1], 1)
        self.assertFalse(self.spin_until(lambda: len(self.references) > count,
                                         timeout=0.3, periodic=self.health),
                         "wall time released a reference while ROS time was frozen")
        type(self).sim_ns += 9_999_999
        self.assertFalse(self.spin_until(lambda: len(self.references) > count,
                                         timeout=0.2, periodic=self.health))
        type(self).sim_ns += 1
        self.assertTrue(self.spin_until(lambda: len(self.references) > count,
                                        periodic=self.health))
        committed = self.references[-1].header.stamp
        self.assertEqual(committed.sec * 1_000_000_000 + committed.nanosec, self.sim_ns)
