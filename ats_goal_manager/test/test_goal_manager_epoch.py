#!/usr/bin/env python3
"""Node-level regression for map-canonical goals across localization epochs."""

import time
import signal
import subprocess
import unittest

from ament_index_python.packages import get_package_prefix
from ats_navigation_interfaces.msg import LocalizationStatus
from ats_navigation_interfaces.msg import ExecutionCommand
from ats_navigation_interfaces.msg import GimbalYawStatus
from ats_navigation_interfaces.msg import PlannerGoal
from ats_navigation_interfaces.msg import PlannerStatus
from ats_navigation_interfaces.msg import PlanningMapStatus
from ats_navigation_interfaces.msg import YawAuthorityRequest
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.duration import Duration
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


TOPIC_PREFIX = "/ats_goal_manager_epoch_test"


class TestGoalManagerLocalizationEpoch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        executable = (
            get_package_prefix("ats_goal_manager")
            + "/lib/ats_goal_manager/ats_goal_manager_node"
        )
        parameters = {
            "input_goal_topic": f"{TOPIC_PREFIX}/goal",
            "planner_goal_topic": f"{TOPIC_PREFIX}/planner_goal",
            "planner_status_topic": f"{TOPIC_PREFIX}/planner_status",
            "candidate_reference_topic": f"{TOPIC_PREFIX}/candidate",
            "reference_path_topic": f"{TOPIC_PREFIX}/reference",
            "emergency_stop_topic": f"{TOPIC_PREFIX}/emergency_stop",
            "execution_command_topic": f"{TOPIC_PREFIX}/execution_command",
            "yaw_authority_request_topic": f"{TOPIC_PREFIX}/yaw_authority_request",
            "gimbal_status_topic": f"{TOPIC_PREFIX}/gimbal_status",
            "map_ready_topic": f"{TOPIC_PREFIX}/map_ready",
            "map_status_topic": f"{TOPIC_PREFIX}/map_status",
            "localization_status_topic": f"{TOPIC_PREFIX}/localization_status",
            "odom_topic": f"{TOPIC_PREFIX}/odometry",
            "action_name": f"{TOPIC_PREFIX}/navigate_to_pose",
            "require_localization_status": True,
            "require_map_status": True,
            "map_ready_timeout_sec": 5.0,
            "localization_status_timeout_sec": 5.0,
            "map_wait_timeout_sec": 5.0,
            "emergency_stop_heartbeat_period_sec": 0.05,
            "gimbal_status_timeout_sec": 5.0,
        }
        command = [
            executable,
            "--ros-args",
            "-r",
            "__node:=ats_goal_manager_epoch_test",
        ]
        for name, value in parameters.items():
            encoded = str(value).lower() if isinstance(value, bool) else str(value)
            command.extend(("-p", f"{name}:={encoded}"))
        cls.process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        rclpy.init()
        cls.node = rclpy.create_node("goal_manager_epoch_test_driver")
        transient_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.goal_pub = cls.node.create_publisher(
            PoseStamped, f"{TOPIC_PREFIX}/goal", 10
        )
        cls.localization_pub = cls.node.create_publisher(
            LocalizationStatus,
            f"{TOPIC_PREFIX}/localization_status",
            transient_qos,
        )
        cls.map_status_pub = cls.node.create_publisher(
            PlanningMapStatus, f"{TOPIC_PREFIX}/map_status", transient_qos
        )
        cls.candidate_pub = cls.node.create_publisher(
            Path, f"{TOPIC_PREFIX}/candidate", 1
        )
        cls.odometry_pub = cls.node.create_publisher(
            Odometry, f"{TOPIC_PREFIX}/odometry", 10
        )
        cls.planner_status_pub = cls.node.create_publisher(
            PlannerStatus, f"{TOPIC_PREFIX}/planner_status", 10
        )
        cls.gimbal_status_pub = cls.node.create_publisher(
            GimbalYawStatus, f"{TOPIC_PREFIX}/gimbal_status", transient_qos
        )
        cls.planner_goals = []
        cls.references = []
        cls.stop_states = []
        cls.execution_commands = []
        cls.yaw_authority_requests = []
        cls.gimbal_sequence = 0
        cls.node.create_subscription(
            PlannerGoal,
            f"{TOPIC_PREFIX}/planner_goal",
            cls.planner_goals.append,
            10,
        )
        cls.node.create_subscription(
            Path, f"{TOPIC_PREFIX}/reference", cls.references.append, 1
        )
        cls.node.create_subscription(
            Bool,
            f"{TOPIC_PREFIX}/emergency_stop",
            lambda message: cls.stop_states.append(message.data),
            transient_qos,
        )
        cls.node.create_subscription(
            ExecutionCommand,
            f"{TOPIC_PREFIX}/execution_command",
            cls.execution_commands.append,
            transient_qos,
        )
        cls.node.create_subscription(
            YawAuthorityRequest,
            f"{TOPIC_PREFIX}/yaw_authority_request",
            cls.yaw_authority_requests.append,
            transient_qos,
        )
        cls.tf_broadcaster = TransformBroadcaster(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        if cls.process.poll() is None:
            cls.process.send_signal(signal.SIGINT)
            try:
                cls.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                cls.process.terminate()
                try:
                    cls.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    cls.process.kill()
                    cls.process.wait(timeout=2.0)

    @classmethod
    def spin_until(cls, predicate, timeout=5.0, periodic=None):
        # 等待期间持续刷新 TF/状态，避免把 DDS 建链时序误判为节点行为。
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if periodic is not None:
                periodic()
            rclpy.spin_once(cls.node, timeout_sec=0.05)
            if predicate():
                return True
        return False

    @classmethod
    def publish_tf(cls, map_to_odom_x):
        transform = TransformStamped()
        transform.header.stamp = (
            cls.node.get_clock().now() + Duration(seconds=0.1)
        ).to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "odom"
        transform.transform.translation.x = map_to_odom_x
        transform.transform.rotation.w = 1.0
        cls.tf_broadcaster.sendTransform(transform)

    @classmethod
    def publish_health(cls, epoch, map_to_odom_x):
        if map_to_odom_x is not None:
            cls.publish_tf(map_to_odom_x)
        odometry = Odometry()
        odometry.header.stamp = cls.node.get_clock().now().to_msg()
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "gimbal_yaw_odom"
        odometry.pose.pose.orientation.w = 1.0
        cls.odometry_pub.publish(odometry)
        localization = LocalizationStatus()
        localization.header.stamp = cls.node.get_clock().now().to_msg()
        localization.header.frame_id = "map"
        localization.state = LocalizationStatus.STATE_TRACKING
        localization.epoch = epoch
        cls.localization_pub.publish(localization)
        map_status = PlanningMapStatus()
        map_status.header.stamp = cls.node.get_clock().now().to_msg()
        map_status.header.frame_id = "map"
        map_status.ready = True
        map_status.localization_epoch = epoch
        map_status.rog_generation = epoch
        map_status.publication_sequence = epoch
        cls.map_status_pub.publish(map_status)
        request = (
            cls.yaw_authority_requests[-1]
            if cls.yaw_authority_requests
            else None
        )
        cls.publish_gimbal_status(
            request.request_sequence if request else 0,
            request.yaw_authority
            if request
            else GimbalYawStatus.YAW_AUTHORITY_GIMBAL_COMPENSATED,
            request.require_gimbal_lock if request else False,
        )

    @classmethod
    def publish_gimbal_status(cls, request_sequence, yaw_authority, locked):
        gimbal = GimbalYawStatus()
        gimbal.header.stamp = cls.node.get_clock().now().to_msg()
        cls.gimbal_sequence += 1
        gimbal.sequence = cls.gimbal_sequence
        gimbal.request_sequence = request_sequence
        gimbal.yaw_authority = yaw_authority
        gimbal.locked = locked
        gimbal.tf_healthy = True
        cls.gimbal_status_pub.publish(gimbal)

    @classmethod
    def publish_candidate(
        cls,
        goal,
        epoch,
        publication_sequence=None,
        yaw_authority=PlannerStatus.YAW_AUTHORITY_GIMBAL_COMPENSATED,
        requires_gimbal_lock=False,
    ):
        stamp = cls.node.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "odom"
        for x in (0.0, goal.goal_pose.pose.position.x):
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "odom"
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        status = PlannerStatus()
        status.header.stamp = cls.node.get_clock().now().to_msg()
        status.goal_id = goal.goal_id
        status.localization_epoch = epoch
        status.map_generation = epoch
        status.map_publication_sequence = (
            epoch if publication_sequence is None else publication_sequence
        )
        status.state = PlannerStatus.STATE_REFERENCE_READY
        status.reference_stamp = stamp
        status.yaw_authority = yaw_authority
        status.requires_gimbal_lock = requires_gimbal_lock
        cls.candidate_pub.publish(path)
        cls.planner_status_pub.publish(status)

    def test_map_goal_is_replanned_and_old_epoch_reference_is_rejected(self):
        self.assertTrue(
            self.spin_until(
                lambda: self.goal_pub.get_subscription_count() == 1,
                periodic=lambda: self.publish_health(1, None),
            )
        )

        goal = PoseStamped()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = 4.0
        goal.pose.position.y = 1.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        self.assertFalse(
            self.spin_until(
                lambda: bool(self.planner_goals),
                timeout=0.3,
                periodic=lambda: self.publish_health(1, None),
            )
        )
        self.assertIn(True, self.stop_states)
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    item.localization_epoch == 1 for item in self.planner_goals
                ),
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )
        first_goal = next(
            item for item in self.planner_goals if item.localization_epoch == 1
        )
        self.assertEqual(first_goal.goal_pose.header.frame_id, "odom")
        self.assertAlmostEqual(first_goal.goal_pose.pose.position.x, 3.0, places=3)

        goal_count = len(self.planner_goals)
        transient_failure = PlannerStatus()
        transient_failure.header.stamp = self.node.get_clock().now().to_msg()
        transient_failure.goal_id = first_goal.goal_id
        transient_failure.localization_epoch = first_goal.localization_epoch
        transient_failure.state = PlannerStatus.STATE_FAILED
        transient_failure.failure_reason = PlannerStatus.FAILURE_MAP_UNREADY
        transient_failure.map_publication_sequence = first_goal.map_publication_sequence
        self.planner_status_pub.publish(transient_failure)
        self.assertTrue(
            self.spin_until(
                lambda: len(self.planner_goals) > goal_count,
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )
        retried_goal = self.planner_goals[-1]
        self.assertEqual(retried_goal.goal_id, first_goal.goal_id)
        self.assertEqual(retried_goal.localization_epoch, first_goal.localization_epoch)

        goal_count = len(self.planner_goals)
        blocked_snapshot_failure = PlannerStatus()
        blocked_snapshot_failure.header.stamp = self.node.get_clock().now().to_msg()
        blocked_snapshot_failure.goal_id = retried_goal.goal_id
        blocked_snapshot_failure.localization_epoch = retried_goal.localization_epoch
        blocked_snapshot_failure.state = PlannerStatus.STATE_FAILED
        blocked_snapshot_failure.failure_reason = (
            PlannerStatus.FAILURE_START_OR_GOAL_OCCUPIED
        )
        blocked_snapshot_failure.map_publication_sequence = (
            retried_goal.map_publication_sequence
        )
        self.planner_status_pub.publish(blocked_snapshot_failure)
        self.assertTrue(
            self.spin_until(
                lambda: len(self.planner_goals) > goal_count,
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )
        first_goal = self.planner_goals[-1]
        self.assertEqual(first_goal.goal_id, retried_goal.goal_id)
        self.assertEqual(first_goal.localization_epoch, retried_goal.localization_epoch)

        # A reference carrying an old adapter publication sequence cannot pass
        # the final Goal Manager commit, even when goal/epoch/timestamp match.
        # It must also clear the stale candidate and dispatch the same active
        # goal again, rather than silently remaining in planning until timeout.
        goal_count = len(self.planner_goals)
        self.publish_candidate(first_goal, 1, publication_sequence=0)
        self.assertFalse(
            self.spin_until(
                lambda: bool(self.references),
                timeout=0.4,
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: len(self.planner_goals) > goal_count,
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )
        first_goal = self.planner_goals[-1]
        request_count = len(self.yaw_authority_requests)
        self.publish_candidate(first_goal, 1)
        self.assertTrue(
            self.spin_until(
                lambda: len(self.yaw_authority_requests) > request_count
            )
        )
        request = self.yaw_authority_requests[-1]
        # A current status without this exact request acknowledgement cannot
        # authorize the candidate reference.
        self.publish_gimbal_status(
            request.request_sequence - 1, request.yaw_authority, False
        )
        self.assertFalse(
            self.spin_until(lambda: bool(self.references), timeout=0.3)
        )
        self.publish_gimbal_status(
            request.request_sequence,
            request.yaw_authority,
            request.require_gimbal_lock,
        )
        self.assertTrue(self.spin_until(lambda: len(self.references) == 1))
        self.assertTrue(self.spin_until(lambda: False in self.stop_states))
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    command.mode == ExecutionCommand.MODE_EXECUTE
                    and command.goal_id == first_goal.goal_id
                    and command.manager_incarnation > 0
                    and command.localization_epoch == 1
                    and command.map_publication_sequence == 1
                    and len(command.reference.poses) == 2
                    for command in self.execution_commands
                )
            )
        )

        # A mode change while tracking must first publish a structured STOP.
        # BODY_YAW_FOLLOW then waits for the matching *locked* acknowledgement.
        stop_event_count = len(self.stop_states)
        command_count = len(self.execution_commands)
        request_count = len(self.yaw_authority_requests)
        self.publish_candidate(
            first_goal,
            1,
            yaw_authority=PlannerStatus.YAW_AUTHORITY_BODY_YAW_FOLLOW,
            requires_gimbal_lock=True,
        )
        self.assertTrue(
            self.spin_until(
                lambda: len(self.yaw_authority_requests) > request_count
            )
        )
        switch_request = self.yaw_authority_requests[-1]
        self.assertTrue(
            self.spin_until(lambda: True in self.stop_states[stop_event_count:])
        )
        self.assertTrue(
            any(
                command.mode == ExecutionCommand.MODE_STOP
                for command in self.execution_commands[command_count:]
            )
        )
        self.publish_gimbal_status(
            switch_request.request_sequence,
            switch_request.yaw_authority,
            False,
        )
        self.assertFalse(
            self.spin_until(
                lambda: any(
                    command.mode == ExecutionCommand.MODE_EXECUTE
                    and command.yaw_authority
                    == ExecutionCommand.YAW_AUTHORITY_BODY_YAW_FOLLOW
                    for command in self.execution_commands[command_count:]
                ),
                timeout=0.3,
            )
        )
        self.publish_gimbal_status(
            switch_request.request_sequence,
            switch_request.yaw_authority,
            True,
        )
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    command.mode == ExecutionCommand.MODE_EXECUTE
                    and command.yaw_authority
                    == ExecutionCommand.YAW_AUTHORITY_BODY_YAW_FOLLOW
                    and command.gimbal_request_sequence
                    == switch_request.request_sequence
                    and command.gimbal_feedback_sequence > 0
                    for command in self.execution_commands[command_count:]
                ),
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )

        stop_event_count = len(self.stop_states)
        goal_count = len(self.planner_goals)
        localization_map_transition = PlanningMapStatus()
        localization_map_transition.header.stamp = self.node.get_clock().now().to_msg()
        localization_map_transition.header.frame_id = "map"
        localization_map_transition.ready = False
        localization_map_transition.localization_epoch = 1
        localization_map_transition.publication_sequence = 10
        localization_map_transition.message = "localization is not tracking"
        self.map_status_pub.publish(localization_map_transition)
        self.assertTrue(
            self.spin_until(lambda: True in self.stop_states[stop_event_count:])
        )
        self.assertTrue(
            self.spin_until(
                lambda: len(self.planner_goals) > goal_count,
                periodic=lambda: self.publish_health(1, 1.0),
            )
        )
        recovered_goal = self.planner_goals[-1]
        self.assertEqual(recovered_goal.goal_id, first_goal.goal_id)
        self.assertEqual(recovered_goal.localization_epoch, 1)
        reference_count = len(self.references)
        request_count = len(self.yaw_authority_requests)
        self.publish_candidate(recovered_goal, 1)
        self.assertTrue(
            self.spin_until(
                lambda: len(self.yaw_authority_requests) > request_count
            )
        )
        request = self.yaw_authority_requests[-1]
        self.publish_gimbal_status(
            request.request_sequence,
            request.yaw_authority,
            request.require_gimbal_lock,
        )
        self.assertTrue(
            self.spin_until(lambda: len(self.references) == reference_count + 1)
        )
        first_goal = recovered_goal

        stop_event_count = len(self.stop_states)
        self.localization_pub.publish(
            LocalizationStatus(
                header=first_goal.header,
                state=LocalizationStatus.STATE_TRACKING,
                epoch=2,
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: True in self.stop_states[stop_event_count:],
                periodic=lambda: self.publish_tf(2.0),
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: any(
                    command.mode == ExecutionCommand.MODE_STOP
                    and command.command_sequence > 0
                    for command in self.execution_commands
                )
            )
        )

        self.assertTrue(
            self.spin_until(
                lambda: any(
                    item.localization_epoch == 2 for item in self.planner_goals
                ),
                periodic=lambda: self.publish_health(2, 2.0),
            )
        )
        second_goal = next(
            item for item in self.planner_goals if item.localization_epoch == 2
        )
        self.assertEqual(second_goal.goal_id, first_goal.goal_id)
        self.assertAlmostEqual(second_goal.goal_pose.pose.position.x, 2.0, places=3)

        reference_count = len(self.references)
        false_count = self.stop_states.count(False)
        self.publish_candidate(first_goal, 1)
        self.assertFalse(
            self.spin_until(
                lambda: len(self.references) > reference_count,
                timeout=0.4,
                periodic=lambda: self.publish_health(2, 2.0),
            )
        )
        self.assertEqual(self.stop_states.count(False), false_count)

        request_count = len(self.yaw_authority_requests)
        self.publish_candidate(second_goal, 2)
        self.assertTrue(
            self.spin_until(
                lambda: len(self.yaw_authority_requests) > request_count
            )
        )
        request = self.yaw_authority_requests[-1]
        self.publish_gimbal_status(
            request.request_sequence,
            request.yaw_authority,
            request.require_gimbal_lock,
        )
        self.assertTrue(
            self.spin_until(lambda: len(self.references) == reference_count + 1)
        )
        self.assertTrue(
            self.spin_until(lambda: self.stop_states.count(False) > false_count)
        )
