#!/usr/bin/env python3
"""Node-level regression: a no-stop replacement keeps the vehicle moving.

With require_gimbal_status=true the first commit consumes the pending yaw
authority request.  A replacement with unchanged authority skips the new
request, so it must be authorised by the active command's acknowledged lease;
before 2026-10-30 it demanded a pending request and emergency-stopped every
time (domain 111: reason=commit_gimbal_ack_missing).
"""

import os
import sys
import unittest

from ats_navigation_interfaces.msg import ExecutionCommand
from ats_navigation_interfaces.msg import PlannerStatus
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_goal_manager_epoch as epoch_harness  # noqa: E402

TOPIC_PREFIX = "/ats_goal_manager_replacement_test"


class TestGoalManagerReplacement(epoch_harness.TestGoalManagerLocalizationEpoch):
    @classmethod
    def setUpClass(cls):
        epoch_harness.TOPIC_PREFIX = TOPIC_PREFIX
        cls.extra_parameters = {
            "replan_without_stop_enabled": True,
            "require_gimbal_status": True,
        }
        super().setUpClass()

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.skipTest("covered by test_goal_manager_epoch")

    def test_no_stop_replacement_is_committed_without_emergency_stop(self):
        health = lambda: self.publish_health(1, 0.0)  # noqa: E731
        self.assertTrue(
            self.spin_until(
                lambda: self.goal_pub.get_subscription_count() == 1,
                periodic=health,
            )
        )
        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = 4.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.assertTrue(self.spin_until(lambda: bool(self.planner_goals), periodic=health))
        first_goal = self.planner_goals[-1]

        request_count = len(self.yaw_authority_requests)
        self.publish_candidate(first_goal, 1)
        self.assertTrue(
            self.spin_until(lambda: len(self.yaw_authority_requests) > request_count)
        )
        # publish_health acknowledges the latest yaw authority request.
        self.assertTrue(
            self.spin_until(lambda: len(self.references) == 1, periodic=health)
        )
        self.assertTrue(self.spin_until(lambda: False in self.stop_states, periodic=health))
        released_at = len(self.stop_states) - 1 - self.stop_states[::-1].index(False)
        requests_after_first_commit = len(self.yaw_authority_requests)

        advised = PlannerStatus()
        advised.header.stamp = self.node.get_clock().now().to_msg()
        advised.goal_id = first_goal.goal_id
        advised.localization_epoch = first_goal.localization_epoch
        advised.plan_request_sequence = first_goal.plan_request_sequence
        advised.map_publication_sequence = first_goal.map_publication_sequence
        advised.state = PlannerStatus.STATE_ACCEPTED
        advised.failure_reason = PlannerStatus.FAILURE_REPLAN_ADVISED
        goal_count = len(self.planner_goals)
        self.planner_status_pub.publish(advised)
        self.assertTrue(
            self.spin_until(lambda: len(self.planner_goals) > goal_count, periodic=health)
        )
        replacement_goal = self.planner_goals[-1]
        self.assertEqual(replacement_goal.goal_id, first_goal.goal_id)
        self.assertGreater(
            replacement_goal.plan_request_sequence, first_goal.plan_request_sequence
        )

        self.publish_candidate(replacement_goal, 1)
        self.assertTrue(
            self.spin_until(lambda: len(self.references) == 2, periodic=health)
        )
        self.spin_until(lambda: False, timeout=0.3, periodic=health)
        self.assertNotIn(True, self.stop_states[released_at:])
        self.assertEqual(len(self.yaw_authority_requests), requests_after_first_commit)
        executes = [
            command for command in self.execution_commands
            if command.mode == ExecutionCommand.MODE_EXECUTE
        ]
        self.assertEqual(executes[-1].goal_id, first_goal.goal_id)
        self.assertGreater(executes[-1].gimbal_request_sequence, 0)


if __name__ == "__main__":
    unittest.main()
