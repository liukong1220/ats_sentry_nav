#!/usr/bin/env python3
"""A newer map needs a full candidate recheck, without starving submission."""
import os
import sys
import math

from ats_navigation_interfaces.msg import PlanningMapSnapshot, PlannerStatus
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.duration import Duration

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as harness  # noqa: E402


class TestFastMaps(harness.TestGoalManagerLocalizationEpoch):
    sequence = 1
    blocked = False
    snapshot_frame = "odom"
    map_transform = (0.0, 0.0, 0.0)

    @classmethod
    def setUpClass(cls):
        harness.TOPIC_PREFIX = "/s3_fast_map_contract"
        cls.sequence, cls.blocked = 1, False
        cls.extra_parameters = {"require_gimbal_status": False}
        super().setUpClass()

    @classmethod
    def health(cls):
        cls.publish_health(1, 0.0, cls.sequence)

    @classmethod
    def publish_tf(cls, map_to_odom_x):
        transform = TransformStamped()
        transform.header.stamp = (cls.node.get_clock().now() + Duration(seconds=0.1)).to_msg()
        transform.header.frame_id, transform.child_frame_id = "map", "odom"
        x, y, yaw = cls.map_transform
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation.z = math.sin(yaw / 2)
        transform.transform.rotation.w = math.cos(yaw / 2)
        cls.tf_broadcaster.sendTransform(transform)

    @classmethod
    def publish_test_candidate(cls, goal, gate_collisions=0, frame="odom"):
        path = Path()
        path.header.stamp = cls.node.get_clock().now().to_msg()
        path.header.frame_id = frame
        for x in (0.0, goal.goal_pose.pose.position.x):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.orientation.w = x, 1.0
            path.poses.append(pose)
        status = PlannerStatus()
        status.header.stamp = path.header.stamp
        status.goal_id, status.localization_epoch = goal.goal_id, 1
        status.plan_request_sequence = goal.plan_request_sequence
        status.map_publication_sequence = cls.sequence
        status.map_generation = cls.sequence
        status.state = status.STATE_REFERENCE_READY
        status.reference_stamp = path.header.stamp
        status.gate_collision_count = gate_collisions
        status.yaw_authority = status.YAW_AUTHORITY_GIMBAL_COMPENSATED
        cls.candidate_pub.publish(path)
        cls.planner_status_pub.publish(status)

    @classmethod
    def publish_planning_snapshot(cls, epoch, publication_sequence=None, occupied=False):
        if cls.blocked or cls.snapshot_frame != "odom":
            snapshot = PlanningMapSnapshot()
            snapshot.header.frame_id = cls.snapshot_frame
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
            if cls.blocked:
                x, y = 2.0, 0.0
                if cls.snapshot_frame == "map":
                    tx, ty, yaw = cls.map_transform
                    x, y = tx + math.cos(yaw) * x, ty + math.sin(yaw) * x
                cell = math.floor(y + 20) * 50 + math.floor(x + 20)
                snapshot.occupancy[cell] = 100
                snapshot.signed_distance_m[cell] = -0.1
            cls.planning_snapshot_pub.publish(snapshot)
        else:
            super().publish_planning_snapshot(epoch, publication_sequence, occupied)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered unchanged by test_goal_manager_epoch")

    def test_new_publication_requires_complete_reference_revalidation(self):
        self.assertTrue(self.spin_until(
            lambda: self.goal_pub.get_subscription_count() == 1, periodic=self.health))
        goal = PoseStamped()
        tx, ty, yaw = self.map_transform
        goal.header.frame_id = "map"
        goal.pose.position.x = tx + 4.0 * math.cos(yaw)
        goal.pose.position.y = ty + 4.0 * math.sin(yaw)
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
        # Equal publication identity is not proof of collision freedom: MINCO's
        # experimental escape prefix can explicitly report an unsafe candidate.
        count = len(self.planner_goals)
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > count,
                                        periodic=self.health))
        same_map_goal = self.planner_goals[-1]
        count = len(self.references)
        self.publish_candidate(same_map_goal, 1, publication_sequence=3)
        self.assertFalse(self.spin_until(lambda: len(self.references) > count, timeout=0.4,
                                         periodic=self.health),
                         "same-publication collision escaped the final commit gate")
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
        # A free map cannot override the planner's nonzero collision declaration.
        type(self).sequence, type(self).blocked = 4, False
        self.spin_until(lambda: False, timeout=0.2, periodic=self.health)
        count = len(self.planner_goals)
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: len(self.planner_goals) > count,
                                        periodic=self.health))
        latest = self.planner_goals[-1]
        count = len(self.references)
        self.publish_test_candidate(latest, gate_collisions=1)
        self.assertFalse(self.spin_until(lambda: len(self.references) > count, timeout=0.4,
                                         periodic=self.health), "declared collision was committed")
        self.publish_test_candidate(self.planner_goals[-1], frame="missing_candidate_frame")
        self.assertFalse(self.spin_until(lambda: len(self.references) > count, timeout=0.4,
                                         periodic=self.health), "missing TF was accepted")
        stale = TransformStamped()
        stale.header.frame_id = self.snapshot_frame
        stale.child_frame_id = "stale_candidate_frame"
        stale.header.stamp = (self.node.get_clock().now() - Duration(seconds=5.0)).to_msg()
        stale.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(stale)
        self.spin_until(lambda: False, timeout=0.2, periodic=self.health)
        self.publish_test_candidate(self.planner_goals[-1], frame=stale.child_frame_id)
        self.assertFalse(self.spin_until(lambda: len(self.references) > count, timeout=0.4,
                                         periodic=self.health), "stale TF was accepted")
        # Rejected candidates do not poison healthy same-publication recovery.
        self.publish_test_candidate(self.planner_goals[-1])
        self.assertTrue(self.spin_until(lambda: len(self.references) > count,
                                        periodic=self.health), "safe recovery was starved")


class TestMapFrameFastMaps(TestFastMaps):
    """Production localization mode publishes map snapshots and odom references."""

    snapshot_frame = "map"
    map_transform = (2.0, -3.0, math.pi / 2)
