#!/usr/bin/env python3
"""A newer map needs a full candidate recheck, without starving submission."""
import os
import sys

from ats_navigation_interfaces.msg import PlanningMapSnapshot, PlannerStatus
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as harness  # noqa: E402


class TestFastMaps(harness.TestGoalManagerLocalizationEpoch):
    sequence = 1
    blocked = False

    @classmethod
    def setUpClass(cls):
        harness.TOPIC_PREFIX = "/s3_fast_map_contract"
        cls.extra_parameters = {"require_gimbal_status": False}
        super().setUpClass()

    @classmethod
    def health(cls):
        cls.publish_health(1, 0.0, cls.sequence)

    @classmethod
    def publish_planning_snapshot(cls, epoch, publication_sequence=None, occupied=False):
        if not cls.blocked:
            super().publish_planning_snapshot(epoch, publication_sequence, occupied)
        else:
            snapshot = PlanningMapSnapshot()
            snapshot.header.frame_id = "odom"
            snapshot.header.stamp = cls.node.get_clock().now().to_msg()
            snapshot.source_stamp = snapshot.header.stamp
            snapshot.ready, snapshot.unknown_is_obstacle = True, True
            snapshot.occupied_value_threshold = 50
            snapshot.localization_epoch = 1
            snapshot.source_generation = cls.sequence
            snapshot.publication_sequence = cls.sequence
            snapshot.info.resolution = 1.0
            snapshot.info.width = snapshot.info.height = 50
            snapshot.info.origin.position.x = snapshot.info.origin.position.y = -20.0
            snapshot.info.origin.orientation.w = 1.0
            snapshot.occupancy = [0] * 2500
            snapshot.signed_distance_m = [1.0] * 2500
            snapshot.gradient_x = snapshot.gradient_y = [0.0] * 2500
            # Between the two sparse path poses, not at either endpoint.
            snapshot.occupancy[20 * 50 + 22] = 100
            snapshot.signed_distance_m[20 * 50 + 22] = -0.1
            cls.planning_snapshot_pub.publish(snapshot)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered unchanged by test_goal_manager_epoch")

    def test_new_publication_requires_complete_reference_revalidation(self):
        self.assertTrue(self.spin_until(
            lambda: self.goal_pub.get_subscription_count() == 1, periodic=self.health))
        goal = PoseStamped()
        goal.header.frame_id, goal.pose.position.x = "map", 4.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: bool(self.planner_goals), periodic=self.health))
        first = self.planner_goals[-1]
        type(self).sequence = 2
        self.spin_until(lambda: False, timeout=0.2, periodic=self.health)
        self.publish_candidate(first, 1, publication_sequence=1)
        self.assertTrue(self.spin_until(lambda: bool(self.references), periodic=self.health),
                        "fresh known-free map publications starved candidate submission")
        self.assertTrue(self.spin_until(
            lambda: any(c.mode == c.MODE_EXECUTE for c in self.execution_commands),
            periodic=self.health))
        command = next(c for c in self.execution_commands if c.mode == c.MODE_EXECUTE)
        self.assertEqual(command.map_publication_sequence, 1, "MINCO provenance was relabelled")
        count = len(self.planner_goals)
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > count, periodic=self.health))
        second = self.planner_goals[-1]
        type(self).sequence, type(self).blocked = 3, True
        self.spin_until(lambda: False, timeout=0.2, periodic=self.health)
        count = len(self.references)
        self.publish_candidate(second, 1, publication_sequence=2)
        self.assertFalse(self.spin_until(lambda: len(self.references) > count, timeout=0.4,
                                         periodic=self.health),
                         "new obstacle between candidate endpoints escaped the commit gate")
        # MINCO may retain a candidate that this authority never committed. A
        # failed replacement cannot claim that nonexistent execution is safe.
        latest = self.planner_goals[-1]
        count = len(self.planner_goals)
        failure = PlannerStatus()
        failure.header.stamp = self.node.get_clock().now().to_msg()
        failure.goal_id, failure.localization_epoch = latest.goal_id, 1
        failure.plan_request_sequence = latest.plan_request_sequence
        failure.map_publication_sequence = latest.map_publication_sequence
        failure.state = failure.STATE_ACCEPTED
        failure.failure_reason = failure.FAILURE_START_OR_GOAL_OCCUPIED
        self.planner_status_pub.publish(failure)
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > count, periodic=self.health),
                        "uncommitted retained candidate stalled recovery")
