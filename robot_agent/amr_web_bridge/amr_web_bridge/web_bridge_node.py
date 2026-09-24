#!/usr/bin/env python3

from __future__ import annotations

import asyncio
from collections import OrderedDict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path as FilePath
from queue import Empty, Queue
import threading
import time
from typing import Any
from uuid import uuid4

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap, LoadMap, ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty as EmptyService, SetBool, Trigger
from tf2_ros import Buffer, TransformListener
import websockets

from .agent_identity import (
    AgentCredential,
    AgentCredentialStore,
    EnrollmentClient,
    EnrollmentError,
    machine_fingerprint,
    verification_fingerprint,
)

from .map_catalog import (
    active_map_id_from_link,
    available_map_yaml,
    build_map_catalog,
    delete_map,
    MAP_ID_PATTERN,
    rename_map,
    set_active_map_link,
    update_map_metadata,
)
from .mapping_runtime import MappingRuntime
from .navigation_recovery import classify_plan_failure, NavigationRecoveryRunner
from .path_utils import (
    path_signature,
    serialize_path,
    serialize_preview_path,
)
from .profile_validator import validate_robot_profile
from .recovery_services import call_empty_service, call_service
from .stall_escape import choose_stall_escape, StallEscapeDecision


class WebBridgeNode(Node):

    def __init__(self) -> None:
        super().__init__('amr_web_bridge')

        self.declare_parameter('server_url', 'ws://localhost:8000')
        self.declare_parameter('robot_id', 'robot01')
        self.declare_parameter('robot_serial_number', 'SIM-0001')
        self.declare_parameter('robot_display_name', 'SCUTTLE-01 Simulator')
        self.declare_parameter('agent_version', '0.2.0')
        self.declare_parameter('profile_version', 'turtlebot3-waffle-sim-v1')
        self.declare_parameter(
            'agent_capabilities',
            'navigation,mapping,localization,diagnostics',
        )
        self.declare_parameter('heartbeat_period', 5.0)
        self.declare_parameter('telemetry_period', 1.0)
        self.declare_parameter('reconnect_delay', 3.0)
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'maps_directory',
            os.getenv('AMR_MAPS_DIRECTORY', ''),
        )
        self.declare_parameter(
            'active_map_id',
            os.getenv('AMR_ACTIVE_MAP_ID', 'warehouse_map'),
        )
        self.declare_parameter(
            'active_map_link',
            os.getenv('AMR_ACTIVE_MAP_LINK', ''),
        )
        self.declare_parameter('map_catalog_period', 10.0)
        self.declare_parameter('load_map_service', '/map_server/load_map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter(
            'physical_estop_topic',
            '/safety/physical_estop',
        )
        self.declare_parameter('physical_estop_stale_seconds', 2.0)
        self.declare_parameter('hardware_contract_mode', 'simulation')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('diagnostics_stale_after_seconds', 3.0)
        self.declare_parameter('diagnostics_expire_after_seconds', 60.0)
        self.declare_parameter('profile_data_freshness_seconds', 3.0)
        self.declare_parameter('profile_moving_linear_threshold', 0.02)
        self.declare_parameter('profile_moving_angular_threshold', 0.05)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('path_max_poses', 500)
        self.declare_parameter('path_publish_period', 0.5)
        self.declare_parameter('battery_percent', 100)
        self.declare_parameter(
            'navigate_action',
            '/navigate_to_pose',
        )
        self.declare_parameter(
            'compute_path_action',
            '/compute_path_to_pose',
        )
        self.declare_parameter('robot_ws_token', '')
        self.declare_parameter('emergency_stop_cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('emergency_stop_zero_rate', 10.0)
        self.declare_parameter('mapping_teleop_deadman_seconds', 0.35)
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('amcl_state_service', '/amcl/get_state')
        self.declare_parameter(
            'global_localization_service',
            '/reinitialize_global_localization',
        )
        self.declare_parameter('localization_scan_angular_speed', 0.26)
        self.declare_parameter('localization_scan_timeout_seconds', 28.0)
        self.declare_parameter('blocked_start_recovery_window_seconds', 90.0)
        self.declare_parameter('near_goal_acceptance_distance_m', 0.35)

        self.server_url = str(
            self.get_parameter('server_url').value
        ).rstrip('/')
        self.robot_id = str(
            self.get_parameter('robot_id').value
        )
        self.robot_serial_number = str(
            self.get_parameter('robot_serial_number').value
        ).strip()
        self.robot_display_name = str(
            self.get_parameter('robot_display_name').value
        ).strip()
        self.agent_version = str(
            self.get_parameter('agent_version').value
        ).strip()
        self.profile_version = str(
            self.get_parameter('profile_version').value
        ).strip()
        self.agent_capabilities = sorted({
            item.strip().lower()
            for item in str(
                self.get_parameter('agent_capabilities').value
            ).split(',')
            if item.strip()
        })
        self.agent_boot_id = str(uuid4())
        self.hardware_fingerprint = machine_fingerprint(
            self.robot_serial_number
        )
        default_credential_file = (
            FilePath.home()
            / '.config'
            / 'indoor-delivery-robot'
            / f'{self.robot_serial_number}.json'
        )
        self.credential_store = AgentCredentialStore(FilePath(os.getenv(
            'ROBOT_CREDENTIAL_FILE',
            str(default_credential_file),
        )))
        stored_credential = self.credential_store.load()
        self.robot_credential = os.getenv('ROBOT_AGENT_CREDENTIAL', '')
        self.robot_credential_version = int(
            os.getenv('ROBOT_AGENT_CREDENTIAL_VERSION', '0')
        )
        if stored_credential is not None:
            self.robot_id = stored_credential.robot_id
            self.robot_credential = stored_credential.credential
            self.robot_credential_version = stored_credential.credential_version
        self.robot_enrollment_token = os.getenv(
            'ROBOT_ENROLLMENT_TOKEN',
            '',
        )
        self.heartbeat_period = float(
            self.get_parameter('heartbeat_period').value
        )
        self.telemetry_period = float(
            self.get_parameter('telemetry_period').value
        )
        self.reconnect_delay = float(
            self.get_parameter('reconnect_delay').value
        )
        self.pose_topic = str(
            self.get_parameter('pose_topic').value
        )
        self.map_topic = str(
            self.get_parameter('map_topic').value
        )
        self.maps_directory = str(
            self.get_parameter('maps_directory').value
        )
        self.active_map_link = str(
            self.get_parameter('active_map_link').value
        ).strip()
        configured_active_map_id = str(
            self.get_parameter('active_map_id').value
        ).strip() or None
        linked_active_map_id = active_map_id_from_link(
            self.maps_directory,
            self.active_map_link,
        ) if self.active_map_link else None
        # A physical robot with an active-map link must never revive a stale
        # profile value. No valid link is an explicit no-map state.
        self.active_map_id = (
            linked_active_map_id
            if self.active_map_link else configured_active_map_id
        )
        self.map_catalog_period = max(
            2.0,
            float(self.get_parameter('map_catalog_period').value),
        )
        self.load_map_service = str(
            self.get_parameter('load_map_service').value
        )
        self.odom_topic = str(
            self.get_parameter('odom_topic').value
        )
        self.scan_topic = str(
            self.get_parameter('scan_topic').value
        )
        self.battery_topic = str(
            self.get_parameter('battery_topic').value
        )
        self.physical_estop_topic = str(
            self.get_parameter('physical_estop_topic').value
        )
        self.physical_estop_stale_seconds = max(
            0.1,
            float(
                self.get_parameter('physical_estop_stale_seconds').value
            ),
        )
        self.hardware_contract_mode = str(
            self.get_parameter('hardware_contract_mode').value
        ).strip().lower()
        if self.hardware_contract_mode not in {
            'physical',
            'prototype',
            'simulation',
        }:
            raise ValueError(
                'hardware_contract_mode must be physical, prototype, '
                'or simulation'
            )
        self.diagnostics_topic = str(
            self.get_parameter('diagnostics_topic').value
        )
        self.diagnostics_stale_after_seconds = max(
            1.0,
            float(
                self.get_parameter(
                    'diagnostics_stale_after_seconds'
                ).value
            ),
        )
        self.diagnostics_expire_after_seconds = max(
            self.diagnostics_stale_after_seconds,
            float(
                self.get_parameter(
                    'diagnostics_expire_after_seconds'
                ).value
            ),
        )
        self.path_topic = str(
            self.get_parameter('path_topic').value
        )
        self.path_max_poses = min(
            500,
            max(
                2,
                int(self.get_parameter('path_max_poses').value),
            ),
        )
        self.path_publish_period = max(
            0.1,
            float(
                self.get_parameter('path_publish_period').value
            ),
        )
        self.battery_percent = min(
            100,
            max(0, int(self.get_parameter('battery_percent').value)),
        )
        self.battery_source = (
            'SIMULATED'
            if self.hardware_contract_mode == 'simulation'
            else 'UNAVAILABLE'
        )
        self.navigate_action = str(
            self.get_parameter('navigate_action').value
        )
        self.compute_path_action = str(
            self.get_parameter('compute_path_action').value
        )
        parameter_token = str(
            self.get_parameter('robot_ws_token').value
        )
        self.robot_ws_token = os.getenv(
            'ROBOT_WS_TOKEN', parameter_token
        )
        self.emergency_stop_cmd_vel_topic = str(
            self.get_parameter(
                'emergency_stop_cmd_vel_topic'
            ).value
        )
        self.emergency_stop_zero_rate = min(
            20.0,
            max(
                5.0,
                float(
                    self.get_parameter(
                        'emergency_stop_zero_rate'
                    ).value
                ),
            ),
        )
        self.mapping_teleop_deadman_seconds = min(
            1.0,
            max(0.15, float(self.get_parameter('mapping_teleop_deadman_seconds').value)),
        )
        self.initial_pose_topic = str(
            self.get_parameter('initial_pose_topic').value
        )
        self.amcl_state_service = str(
            self.get_parameter('amcl_state_service').value
        )
        self.global_localization_service = str(
            self.get_parameter('global_localization_service').value
        )
        self.localization_scan_angular_speed = min(
            0.4,
            max(0.15, float(self.get_parameter('localization_scan_angular_speed').value)),
        )
        self.localization_scan_timeout_seconds = min(
            60.0,
            max(10.0, float(self.get_parameter('localization_scan_timeout_seconds').value)),
        )
        self.blocked_start_recovery_window_seconds = min(
            120.0,
            max(
                30.0,
                float(self.get_parameter(
                    'blocked_start_recovery_window_seconds'
                ).value),
            ),
        )
        self.near_goal_acceptance_distance_m = min(
            0.5,
            max(
                0.15,
                float(self.get_parameter(
                    'near_goal_acceptance_distance_m'
                ).value),
            ),
        )
        self.websocket_uri = (
            f'{self.server_url}/ws/robots/{self.robot_id}'
        )

        self.stop_requested = threading.Event()
        self.telemetry_lock = threading.Lock()
        self.map_lock = threading.Lock()
        self.velocity_lock = threading.Lock()
        self.scan_lock = threading.Lock()
        self.diagnostics_lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.path_lock = threading.Lock()
        self.preview_lock = threading.Lock()
        self.map_command_lock = threading.Lock()
        self.mapping_velocity_lock = threading.Lock()
        self.localization_lock = threading.Lock()
        self.emergency_stop_latched = threading.Event()
        self.physical_estop_latched = threading.Event()
        self.last_emergency_command_id: str | None = None
        self.latest_telemetry: dict[str, Any] | None = None
        self.latest_map: dict[str, Any] | None = None
        self.latest_diagnostics: dict[str, Any] | None = None
        self.latest_velocity: (
            dict[str, float] | None
        ) = None
        self.latest_scan: dict[str, Any] | None = None
        self.last_map_monotonic: float | None = None
        self.last_odom_monotonic: float | None = None
        self.last_scan_monotonic: float | None = None
        self.last_battery_monotonic: float | None = None
        self.last_physical_estop_monotonic: float | None = None
        self.last_diagnostics_monotonic: float | None = None
        self.map_revision = 0
        self.diagnostics_revision = 0
        self.node_started_monotonic = time.monotonic()
        self.latest_localization_pose: dict[str, Any] | None = None
        self.last_localization_monotonic: float | None = None
        self.amcl_state = 'UNKNOWN'
        self.amcl_state_future: Any = None
        self.amcl_state_request_started_monotonic: float | None = None
        self.amcl_state_last_success_monotonic: float | None = None
        self.localization_recovery_count = 0
        self.localization_recovery_active = False
        self.localization_recovery_started_monotonic: float | None = None
        self.localization_convergence_samples = 0
        self.localization_scan_active = False
        self.localization_scan_started_monotonic: float | None = None
        self.command_queue: Queue[
            dict[str, Any]
        ] = Queue()
        self.cancel_queue: Queue[
            dict[str, Any]
        ] = Queue()
        self.pending_command_ids: set[str] = set()
        self.processed_command_ids: OrderedDict[str, None] = OrderedDict()
        self.processed_command_limit = 512
        self.pending_cancel_requests: dict[
            str,
            dict[str, Any],
        ] = {}
        self.cancelled_task_ids: set[str] = set()
        self.active_command: (
            dict[str, Any] | None
        ) = None
        self.active_goal_handle: Any = None
        self.recovery_cancelled_command_ids: set[str] = set()
        self.robot_operation_lock = threading.Lock()
        self.robot_operation_active = False
        self.automatic_stall_recovery_lock = threading.Lock()
        self.automatic_stall_recovery_active = False
        self.automatic_stall_recovery_attempted_ids: set[str] = set()
        self.blocked_start_recovery_lock = threading.Lock()
        self.blocked_start_recovery_active = False
        self.blocked_start_recovery_attempted_ids: set[str] = set()
        self.navigation_recovery_runner = NavigationRecoveryRunner()
        self.last_feedback_log_ns = 0
        self.latest_path: dict[str, Any] | None = None
        self.path_revision = 0
        self.latest_path_signature: tuple[Any, ...] | None = None
        self.preview_queue: Queue[dict[str, Any]] = Queue()
        self.active_preview: dict[str, Any] | None = None
        self.map_command_queue: Queue[dict[str, Any]] = Queue()
        self.active_map_command: dict[str, Any] | None = None
        self.localization_command_queue: Queue[dict[str, Any]] = Queue()
        self.active_localization_command: dict[str, Any] | None = None
        mapping_params_file = os.path.join(
            get_package_share_directory('amr_web_bridge'),
            'config',
            'web_mapping.yaml',
        )
        self.mapping_runtime = MappingRuntime(
            self.maps_directory,
            slam_params_file=mapping_params_file,
            lifecycle_state=self.mapping_lifecycle_node_active,
            lifecycle_command=self.mapping_lifecycle_command,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.profile_data_freshness_seconds = max(
            0.5,
            float(self.get_parameter('profile_data_freshness_seconds').value),
        )
        self.profile_moving_linear_threshold = max(
            0.001,
            float(self.get_parameter('profile_moving_linear_threshold').value),
        )
        self.profile_moving_angular_threshold = max(
            0.001,
            float(self.get_parameter('profile_moving_angular_threshold').value),
        )
        self.base_frame = str(self.get_parameter('base_frame').value).strip() or 'base_footprint'
        self.mapping_velocity_deadline = 0.0
        self.mapping_velocity_active = False
        self.localization_velocity_deadline = 0.0
        self.localization_velocity_active = False
        self.mapping_pose_reference: tuple[
            tuple[float, float, float],
            tuple[float, float, float],
        ] | None = None

        self.asyncio_loop: asyncio.AbstractEventLoop | None = None
        self.send_lock: asyncio.Lock | None = None
        self.websocket: Any = None

        self.pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            self.pose_qos,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            self.pose_qos,
        )
        self.odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
            ),
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            self.odom_qos,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            self.odom_qos,
        )
        self.safety_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.battery_subscription = self.create_subscription(
            BatteryState,
            self.battery_topic,
            self.battery_callback,
            self.safety_qos,
        )
        self.physical_estop_subscription = self.create_subscription(
            Bool,
            self.physical_estop_topic,
            self.physical_estop_callback,
            self.safety_qos,
        )
        self.diagnostics_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.diagnostics_subscription = self.create_subscription(
            DiagnosticArray,
            self.diagnostics_topic,
            self.diagnostics_callback,
            self.diagnostics_qos,
        )
        self.path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.path_subscription = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            self.path_qos,
        )
        self.navigation_client = ActionClient(
            self,
            NavigateToPose,
            self.navigate_action,
        )
        self.preview_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_action,
        )
        # Recovery runs from a worker thread after Nav2 aborts a goal. Keep a
        # warm action client in this node's DDS participant so recovery never
        # launches a fresh ``ros2 action send_goal`` process under load.
        self.recovery_plan_client = ActionClient(
            self,
            ComputePathToPose,
            self.compute_path_action,
        )
        self.recovery_spin_client = ActionClient(self, Spin, '/spin')
        self.load_map_client = self.create_client(
            LoadMap,
            self.load_map_service,
        )
        self.amcl_state_client = self.create_client(
            GetState,
            self.amcl_state_service,
        )
        self.localization_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_localization/manage_nodes',
        )
        self.navigation_lifecycle_client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.global_localization_client = self.create_client(
            EmptyService,
            self.global_localization_service,
        )
        # These clients stay in the node's DDS participant.  They avoid
        # launching a fresh ``ros2 service call`` process at the exact moment
        # a pre-stall needs to be handled.
        self.recovery_lifecycle_clients = [
            (name, self.create_client(GetState, f'{name}/get_state'))
            for name in (
                '/planner_server',
                '/controller_server',
                '/behavior_server',
                '/bt_navigator',
            )
        ]
        self.mapping_lifecycle_clients = {
            name.removeprefix('/'): client
            for name, client in self.recovery_lifecycle_clients
        }
        self.mapping_lifecycle_clients['amcl'] = self.amcl_state_client
        self.mapping_lifecycle_manager_clients = {
            'lifecycle_manager_localization': (
                self.localization_lifecycle_client
            ),
            'lifecycle_manager_navigation': self.navigation_lifecycle_client,
        }
        self.recovery_costmap_clients = [
            (name, self.create_client(ClearEntireCostmap, name))
            for name in (
                '/local_costmap/clear_entirely_local_costmap',
                '/global_costmap/clear_entirely_global_costmap',
            )
        ]
        self.clear_motor_fault_client = self.create_client(
            Trigger,
            '/clear_motor_fault',
        )
        self.finish_motor_recovery_client = self.create_client(
            SetBool,
            '/finish_motor_recovery',
        )
        self.guarded_reverse_client = self.create_client(
            Trigger,
            '/guarded_reverse_escape',
        )
        self.command_timer = self.create_timer(
            0.1,
            self.process_command_queue,
        )
        self.preview_timer = self.create_timer(
            0.1,
            self.process_preview_queue,
        )
        self.map_command_timer = self.create_timer(
            0.1,
            self.process_map_command_queue,
        )
        self.localization_command_timer = self.create_timer(
            0.1,
            self.process_localization_command_queue,
        )
        self.amcl_state_timer = self.create_timer(
            1.0,
            self.poll_amcl_state,
        )
        self.emergency_velocity_publisher = self.create_publisher(
            Twist,
            self.emergency_stop_cmd_vel_topic,
            10,
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            10,
        )
        self.emergency_zero_timer = self.create_timer(
            1.0 / self.emergency_stop_zero_rate,
            self.publish_emergency_zero,
        )
        self.physical_estop_watchdog_timer = self.create_timer(
            0.2,
            self.enforce_physical_estop_watchdog,
        )
        self.mapping_deadman_timer = self.create_timer(
            0.05,
            self.enforce_mapping_deadman,
        )
        self.localization_scan_timer = self.create_timer(
            0.1,
            self.run_localization_scan,
        )
        self.mapping_pose_timer = self.create_timer(
            0.2,
            self.update_mapping_pose,
        )
        self.automatic_stall_recovery_timer = self.create_timer(
            0.10,
            self.maybe_start_automatic_stall_recovery,
        )

        self.worker_thread = threading.Thread(
            target=self.run_asyncio_thread,
            name='amr-websocket-thread',
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info('AMR Web Bridge started')
        self.get_logger().info(f'Robot ID: {self.robot_id}')
        self.get_logger().info(
            f'FastAPI WebSocket: {self.websocket_uri}'
        )
        self.get_logger().info(
            f'Pose telemetry topic: {self.pose_topic}'
        )
        self.get_logger().info(f'Map topic: {self.map_topic}')
        self.get_logger().info(
            f'Map catalog directory: {self.maps_directory or "not configured"}'
        )
        self.get_logger().info(
            'Active map: '
            f'{self.active_map_id or "not selected"}'
        )
        self.get_logger().info(
            f'Nav2 load map service: {self.load_map_service}'
        )
        self.get_logger().info(
            f'Nav2 action: {self.navigate_action}'
        )
        self.get_logger().info(
            f'Nav2 preview action: {self.compute_path_action}'
        )
        self.get_logger().info(
            f'Odometry topic: {self.odom_topic}'
        )
        self.get_logger().info(
            f'Diagnostics topic: {self.diagnostics_topic}'
        )
        self.get_logger().info(
            f'Nav2 global path topic: {self.path_topic}'
        )

    @staticmethod
    def utc_timestamp() -> str:
        return datetime.now(timezone.utc).isoformat()

    def run_asyncio_thread(self) -> None:
        try:
            asyncio.run(self.connection_supervisor())
        except Exception as error:
            if not self.stop_requested.is_set():
                self.get_logger().error(
                    f'WebSocket thread failed: {error}'
                )
        finally:
            self.asyncio_loop = None

    def pose_callback(
        self,
        message: PoseWithCovarianceStamped,
    ) -> None:
        pose = message.pose.pose
        orientation = pose.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        yaw = math.atan2(sin_yaw, cos_yaw)

        battery, battery_source = self.current_battery_state()
        telemetry = {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': float(yaw),
            'battery': battery,
            'battery_source': battery_source,
            'frame_id': message.header.frame_id or 'map',
            'timestamp': self.utc_timestamp(),
        }

        with self.telemetry_lock:
            self.latest_telemetry = telemetry
        covariance = message.pose.covariance
        with self.localization_lock:
            self.latest_localization_pose = {
                'frame_id': message.header.frame_id or 'map',
                'x': float(pose.position.x),
                'y': float(pose.position.y),
                'yaw': float(yaw),
                'position_uncertainty': math.sqrt(max(
                    0.0,
                    float(covariance[0]),
                    float(covariance[7]),
                )),
                'yaw_uncertainty': math.sqrt(max(0.0, float(covariance[35]))),
            }
            self.last_localization_monotonic = time.monotonic()

    def odom_callback(
        self,
        message: Odometry,
    ) -> None:
        twist = message.twist.twist

        linear_velocity = math.hypot(
            float(twist.linear.x),
            float(twist.linear.y),
        )
        angular_velocity = float(
            twist.angular.z
        )

        if not all(
            math.isfinite(value)
            for value in (
                linear_velocity,
                angular_velocity,
            )
        ):
            return

        with self.velocity_lock:
            self.latest_velocity = {
                'linear_velocity': linear_velocity,
                'angular_velocity': angular_velocity,
            }
            self.last_odom_monotonic = time.monotonic()

    def scan_callback(self, message: LaserScan) -> None:
        snapshot = {
            'ranges': tuple(float(value) for value in message.ranges),
            'angle_min': float(message.angle_min),
            'angle_increment': float(message.angle_increment),
            'range_min': float(message.range_min),
            'range_max': float(message.range_max),
        }
        with self.scan_lock:
            self.latest_scan = snapshot
            self.last_scan_monotonic = time.monotonic()

    def battery_callback(self, message: BatteryState) -> None:
        percentage = float(message.percentage)
        if not math.isfinite(percentage) or not 0.0 <= percentage <= 1.0:
            return
        with self.telemetry_lock:
            self.battery_percent = round(percentage * 100.0)
            self.battery_source = 'SENSOR'
            self.last_battery_monotonic = time.monotonic()

    def current_battery_state(self) -> tuple[int, str]:
        with self.telemetry_lock:
            return self.battery_percent, self.battery_source

    def physical_estop_callback(self, message: Bool) -> None:
        self.last_physical_estop_monotonic = time.monotonic()
        if bool(message.data):
            self.latch_physical_estop()
            return
        self.physical_estop_latched.clear()
        self.publish_zero_velocity()

    def latch_physical_estop(self) -> None:
        was_latched = self.physical_estop_latched.is_set()
        self.physical_estop_latched.set()
        self.publish_emergency_zero()
        if was_latched:
            return
        self.clear_command_queue()
        self.clear_navigation_path(send_clear=True, force=True)
        with self.command_lock:
            goal_handle = self.active_goal_handle
            self.active_command = None
            self.active_goal_handle = None
            self.pending_command_ids.clear()
            self.pending_cancel_requests.clear()
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as error:
                self.get_logger().error(
                    'Physical Emergency Stop Nav2 cancellation failed: '
                    f'{error}'
                )

    def enforce_physical_estop_watchdog(self) -> None:
        if self.hardware_contract_mode != 'physical':
            return
        last_update = self.last_physical_estop_monotonic
        if (
            last_update is not None
            and time.monotonic() - last_update
            <= self.physical_estop_stale_seconds
        ):
            return
        self.latch_physical_estop()

    def update_mapping_pose(self) -> None:
        snapshot = self.mapping_runtime.snapshot(self.map_revision)
        if snapshot.get('phase') not in {'MAPPING', 'STOPPING', 'REVIEW'}:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                Time(),
            )
        except Exception:
            return

        translation = transform.transform.translation
        orientation = transform.transform.rotation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        battery, battery_source = self.current_battery_state()
        telemetry = {
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': float(math.atan2(sin_yaw, cos_yaw)),
            'battery': battery,
            'battery_source': battery_source,
            'frame_id': 'map',
            'timestamp': self.utc_timestamp(),
        }
        with self.telemetry_lock:
            self.latest_telemetry = telemetry

    def lookup_planar_pose(
        self,
        target_frame: str,
        source_frame: str,
    ) -> tuple[float, float, float]:
        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(),
        ).transform
        orientation = transform.rotation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        return (
            float(transform.translation.x),
            float(transform.translation.y),
            float(math.atan2(sin_yaw, cos_yaw)),
        )

    def capture_mapping_pose_reference(self) -> None:
        try:
            self.mapping_pose_reference = (
                self.lookup_planar_pose('map', self.base_frame),
                self.lookup_planar_pose('odom', self.base_frame),
            )
        except Exception:
            self.mapping_pose_reference = None

    def restore_mapping_pose_estimate(self) -> None:
        reference = self.mapping_pose_reference
        self.mapping_pose_reference = None
        if reference is None:
            return
        try:
            (map_x, map_y, map_yaw), (
                odom_x0, odom_y0, odom_yaw0,
            ) = reference
            odom_x1, odom_y1, odom_yaw1 = self.lookup_planar_pose(
                'odom', 'base_footprint'
            )
        except Exception:
            return

        odom_dx = odom_x1 - odom_x0
        odom_dy = odom_y1 - odom_y0
        delta_x = (
            math.cos(odom_yaw0) * odom_dx
            + math.sin(odom_yaw0) * odom_dy
        )
        delta_y = (
            -math.sin(odom_yaw0) * odom_dx
            + math.cos(odom_yaw0) * odom_dy
        )
        pose_x = (
            map_x + math.cos(map_yaw) * delta_x
            - math.sin(map_yaw) * delta_y
        )
        pose_y = (
            map_y + math.sin(map_yaw) * delta_x
            + math.cos(map_yaw) * delta_y
        )
        pose_yaw = map_yaw + odom_yaw1 - odom_yaw0

        self.publish_initial_pose(
            pose_x,
            pose_y,
            pose_yaw,
            position_uncertainty=0.5,
            yaw_uncertainty=math.sqrt(0.0685),
        )

    def publish_initial_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        position_uncertainty: float,
        yaw_uncertainty: float,
    ) -> None:
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = position_uncertainty ** 2
        message.pose.covariance[7] = position_uncertainty ** 2
        message.pose.covariance[35] = yaw_uncertainty ** 2
        self.initial_pose_publisher.publish(message)

    def poll_amcl_state(self) -> None:
        now = time.monotonic()
        future = self.amcl_state_future
        if future is not None and not future.done():
            started = self.amcl_state_request_started_monotonic
            if started is None or now - started <= 3.0:
                return
            # A lost lifecycle response used to leave this future pending
            # forever, pinning readiness at UNKNOWN even after AMCL recovered.
            # Retire it and let the warm client issue a fresh request.
            try:
                future.cancel()
            except Exception:
                pass
            self.amcl_state_future = None
            self.amcl_state_request_started_monotonic = None
        if not self.amcl_state_client.service_is_ready():
            last_success = self.amcl_state_last_success_monotonic
            if last_success is None or now - last_success > 5.0:
                with self.localization_lock:
                    self.amcl_state = 'UNKNOWN'
            return
        future = self.amcl_state_client.call_async(GetState.Request())
        self.amcl_state_future = future
        self.amcl_state_request_started_monotonic = now
        future.add_done_callback(self.amcl_state_callback)

    def amcl_state_callback(self, future: Any) -> None:
        if future is not self.amcl_state_future:
            return
        try:
            lifecycle_state = future.result().current_state
            label = str(lifecycle_state.label).strip().lower()
            state = (
                'ACTIVE'
                if int(lifecycle_state.id) == 3 or label == 'active'
                else 'INACTIVE'
            )
            self.amcl_state_last_success_monotonic = time.monotonic()
        except Exception:
            last_success = self.amcl_state_last_success_monotonic
            if (
                last_success is not None
                and time.monotonic() - last_success <= 5.0
            ):
                return
            state = 'UNKNOWN'
        finally:
            if future is self.amcl_state_future:
                self.amcl_state_future = None
                self.amcl_state_request_started_monotonic = None
        with self.localization_lock:
            self.amcl_state = state

    def localization_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.localization_lock:
            last_pose = (
                dict(self.latest_localization_pose)
                if self.latest_localization_pose is not None else None
            )
            last_received = self.last_localization_monotonic
            amcl_state = self.amcl_state
            recovery_count = self.localization_recovery_count
            recovery_active = self.localization_recovery_active
            scan_active = self.localization_scan_active
            scan_started = self.localization_scan_started_monotonic
        with self.velocity_lock:
            velocity = (
                dict(self.latest_velocity)
                if self.latest_velocity is not None else None
            )
        moving = bool(
            velocity
            and (
                abs(velocity['linear_velocity']) > 0.02
                or abs(velocity['angular_velocity']) > 0.05
            )
        )
        pose_age = max(0.0, now - last_received) if last_received is not None else None
        tf_available = False
        pose = last_pose
        try:
            x, y, yaw = self.lookup_planar_pose('map', 'base_footprint')
            tf_available = True
            pose = {
                **(last_pose or {}),
                'frame_id': 'map',
                'x': x,
                'y': y,
                'yaw': yaw,
            }
        except Exception:
            pass

        mapping_active = self.mapping_runtime.snapshot(
            self.map_revision
        ).get('phase') != 'IDLE'
        position_uncertainty = (
            last_pose.get('position_uncertainty') if last_pose else None
        )
        yaw_uncertainty = (
            last_pose.get('yaw_uncertainty') if last_pose else None
        )
        if mapping_active:
            health, reason, detail = (
                'UNKNOWN', 'MAPPING_ACTIVE', 'Localization is paused during mapping'
            )
        elif amcl_state == 'INACTIVE':
            health, reason, detail = (
                'LOST', 'AMCL_INACTIVE', 'AMCL lifecycle node is not active'
            )
        elif pose is None:
            health = 'LOST' if now - self.node_started_monotonic > 5.0 else 'UNKNOWN'
            reason, detail = 'NO_POSE', 'No AMCL pose has been received'
        elif not tf_available:
            health, reason, detail = (
                'LOST', 'TF_UNAVAILABLE', 'map to base_footprint transform is unavailable'
            )
        elif moving and pose_age is not None and pose_age > 3.0:
            health, reason, detail = (
                'LOST', 'POSE_STALE', 'AMCL pose is stale while the robot is moving'
            )
        elif (
            position_uncertainty is not None
            and yaw_uncertainty is not None
            and (position_uncertainty > 1.0 or yaw_uncertainty > 0.75)
        ):
            health, reason, detail = (
                'DEGRADED', 'HIGH_UNCERTAINTY', 'AMCL pose uncertainty is high'
            )
        else:
            health, reason, detail = (
                'LOCALIZED', 'READY', 'AMCL pose and map transform are available'
            )
        recovery_finished = False
        with self.localization_lock:
            if self.localization_recovery_active:
                old_enough = (
                    self.localization_recovery_started_monotonic is not None
                    and now - self.localization_recovery_started_monotonic >= 5.0
                )
                converged = old_enough and health == 'LOCALIZED' and not moving
                self.localization_convergence_samples = (
                    self.localization_convergence_samples + 1 if converged else 0
                )
                if self.localization_convergence_samples >= 6:
                    self.localization_recovery_active = False
                    self.localization_recovery_started_monotonic = None
                    self.localization_convergence_samples = 0
                    self.localization_scan_active = False
                    self.localization_scan_started_monotonic = None
                    recovery_finished = True
            recovery_active = self.localization_recovery_active
            scan_active = self.localization_scan_active
            scan_started = self.localization_scan_started_monotonic
        if recovery_finished:
            self.publish_zero_velocity()
        clean_pose = None if pose is None else {
            'frame_id': str(pose.get('frame_id', 'map')),
            'x': float(pose['x']),
            'y': float(pose['y']),
            'yaw': float(pose['yaw']),
        }
        return {
            'type': 'localization_status',
            'robot_id': self.robot_id,
            'health': health,
            'reason': reason,
            'amcl_state': amcl_state,
            'map_id': self.active_map_id,
            'pose': clean_pose,
            'pose_age_seconds': pose_age,
            'position_uncertainty': position_uncertainty,
            'yaw_uncertainty': yaw_uncertainty,
            'tf_available': tf_available,
            'moving': moving,
            'recovery_count': recovery_count,
            'recovery_active': recovery_active,
            'automatic_scan_active': scan_active,
            'automatic_scan_progress': (
                min(1.0, max(0.0, (now - scan_started) / self.localization_scan_timeout_seconds))
                if scan_active and scan_started is not None else 0.0
            ),
            'detail': detail,
        }

    @staticmethod
    def normalize_diagnostic_level(level: Any) -> int | None:
        if isinstance(level, (bytes, bytearray, memoryview)):
            raw_bytes = bytes(level)
            return raw_bytes[0] if raw_bytes else None

        try:
            return int(level)
        except (TypeError, ValueError, OverflowError):
            return None

    def diagnostic_level_name(self, level: Any) -> str:
        normalized_level = WebBridgeNode.normalize_diagnostic_level(
            level
        )
        level_name = {
            0: 'OK',
            1: 'WARN',
            2: 'ERROR',
            3: 'STALE',
        }.get(normalized_level)

        if level_name is None:
            self.get_logger().warning(
                'Unknown or empty diagnostic level '
                f'{level!r}; using STALE'
            )
            return 'STALE'

        return level_name

    @staticmethod
    def diagnostic_timestamp(
        message: DiagnosticArray,
    ) -> str | None:
        stamp = message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return None

        try:
            timestamp = (
                float(stamp.sec)
                + float(stamp.nanosec) / 1e9
            )
            return datetime.fromtimestamp(
                timestamp,
                tz=timezone.utc,
            ).isoformat()
        except (OverflowError, OSError, ValueError):
            return None

    def diagnostics_callback(
        self,
        message: DiagnosticArray,
    ) -> None:
        received_at = time.monotonic()
        diagnostics = {
            'type': 'diagnostics',
            'timestamp': self.diagnostic_timestamp(message),
            'statuses': [
                {
                    'name': status.name,
                    'level': self.diagnostic_level_name(
                        status.level
                    ),
                    'message': status.message,
                    'hardware_id': status.hardware_id,
                    'values': [
                        {
                            'key': value.key,
                            'value': value.value,
                        }
                        for value in status.values
                    ],
                    '_received_at': received_at,
                }
                for status in message.status
            ],
        }

        with self.diagnostics_lock:
            merged_statuses = {
                (
                    status['name'],
                    status['hardware_id'],
                ): status
                for status in (
                    self.latest_diagnostics['statuses']
                    if self.latest_diagnostics is not None
                    else []
                )
            }
            for status in diagnostics['statuses']:
                merged_statuses[
                    (status['name'], status['hardware_id'])
                ] = status
            diagnostics['statuses'] = list(
                merged_statuses.values()
            )
            self.latest_diagnostics = diagnostics
            self.diagnostics_revision += 1
            self.last_diagnostics_monotonic = received_at

    def diagnostics_snapshot(
        self,
        now: float | None = None,
    ) -> dict[str, Any] | None:
        if self.latest_diagnostics is None:
            return None

        current_time = time.monotonic() if now is None else now
        active_statuses = []
        retained_statuses = []
        for stored_status in self.latest_diagnostics['statuses']:
            received_at = float(
                stored_status.get('_received_at', current_time)
            )
            age_seconds = max(0.0, current_time - received_at)
            if age_seconds > self.diagnostics_expire_after_seconds:
                continue

            retained_statuses.append(stored_status)
            status = {
                key: value
                for key, value in stored_status.items()
                if key != '_received_at'
            }
            status['values'] = [
                dict(value) for value in stored_status['values']
            ]
            if age_seconds > self.diagnostics_stale_after_seconds:
                status['level'] = 'STALE'
                status['message'] = (
                    'Diagnostic publisher update is stale'
                )
            active_statuses.append(status)

        self.latest_diagnostics['statuses'] = retained_statuses
        return {
            'type': self.latest_diagnostics['type'],
            'timestamp': self.latest_diagnostics['timestamp'],
            'statuses': active_statuses,
        }

    def map_callback(self, message: OccupancyGrid) -> None:
        origin = message.info.origin
        orientation = origin.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        origin_yaw = math.atan2(sin_yaw, cos_yaw)
        stamp = message.header.stamp

        map_payload = {
            'frame_id': message.header.frame_id or 'map',
            'resolution': float(message.info.resolution),
            'width': int(message.info.width),
            'height': int(message.info.height),
            'origin_x': float(origin.position.x),
            'origin_y': float(origin.position.y),
            'origin_yaw': float(origin_yaw),
            'data': [int(value) for value in message.data],
            'timestamp': (
                f'{stamp.sec}.{stamp.nanosec:09d}'
            ),
        }

        with self.map_lock:
            self.latest_map = map_payload
            self.map_revision += 1
            self.last_map_monotonic = time.monotonic()

        self.get_logger().info(
            'Received ROS map '
            f'{message.info.width}x{message.info.height}'
        )

    def mapping_map_snapshot(self) -> dict[str, Any]:
        """Copy the latest live map so saving cannot observe later changes."""
        with self.map_lock:
            if self.latest_map is None:
                raise RuntimeError('Waiting for the live SLAM map')
            return {
                **self.latest_map,
                'data': list(self.latest_map['data']),
                'revision': self.map_revision,
            }

    def activate_saved_mapping_map(self, yaml_path: FilePath) -> None:
        """Persist and start Nav2 with a map produced by this SLAM session."""
        # Remove the prior transient-local map before Nav2 starts with the new
        # file, otherwise the dashboard can briefly revert to the old map.
        with self.map_lock:
            self.latest_map = None
            self.map_revision += 1
        succeeded, output = self.navigation_recovery_runner.activate_map(
            str(yaml_path)
        )
        if not succeeded:
            raise RuntimeError(
                output or 'Unable to activate the newly saved map'
            )
        self.active_map_id = yaml_path.stem

    @staticmethod
    def message_timestamp(message: Any) -> str:
        stamp = message.header.stamp
        try:
            seconds = (
                float(stamp.sec)
                + float(stamp.nanosec) / 1_000_000_000.0
            )
            if math.isfinite(seconds) and seconds >= 0.0:
                return datetime.fromtimestamp(
                    seconds,
                    tz=timezone.utc,
                ).isoformat()
        except (OverflowError, OSError, ValueError):
            pass
        return WebBridgeNode.utc_timestamp()

    def path_callback(self, message: Path) -> None:
        with self.command_lock:
            command = (
                dict(self.active_command)
                if self.active_command is not None
                else None
            )

        path = serialize_path(
            message,
            command,
            self.path_max_poses,
            self.message_timestamp(message),
        )
        if path is None:
            if command is not None:
                self.clear_navigation_path(
                    command,
                    force=False,
                )
            return

        with self.command_lock:
            if (
                self.active_command is None
                or self.active_command.get('command_id')
                != path['command_id']
            ):
                return

        signature = path_signature(path)
        with self.path_lock:
            if signature == self.latest_path_signature:
                return
            self.latest_path = path
            self.latest_path_signature = signature
            self.path_revision += 1

    async def connection_supervisor(self) -> None:
        self.asyncio_loop = asyncio.get_running_loop()

        while not self.stop_requested.is_set():
            try:
                await self.ensure_robot_identity()
                self.websocket_uri = (
                    f'{self.server_url}/ws/robots/{self.robot_id}'
                )
                self.get_logger().info('Connecting to FastAPI...')

                async with websockets.connect(
                    self.websocket_uri,
                    extra_headers=(
                        {
                            'Authorization': (
                                'Bearer '
                                f'{self.robot_credential or self.robot_ws_token}'
                            )
                        }
                        if self.robot_credential or self.robot_ws_token
                        else None
                    ),
                    open_timeout=5,
                    ping_interval=20,
                    ping_timeout=20,
                ) as websocket:
                    self.websocket = websocket
                    self.send_lock = asyncio.Lock()
                    if self.robot_credential:
                        await self.send_agent_hello(websocket)
                    self.get_logger().info(
                        'Connected to FastAPI WebSocket'
                    )

                    background_tasks = [
                        asyncio.create_task(
                            self.heartbeat_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.telemetry_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.map_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.map_catalog_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.diagnostics_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.path_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.mapping_status_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.localization_status_loop(websocket)
                        ),
                        asyncio.create_task(
                            self.agent_readiness_loop(websocket)
                        ),
                    ]

                    try:
                        await self.receive_loop(websocket)
                    finally:
                        for task in background_tasks:
                            task.cancel()

                        await asyncio.gather(
                            *background_tasks,
                            return_exceptions=True,
                        )

            except Exception as error:
                if not self.stop_requested.is_set():
                    self.get_logger().warning(
                        f'WebSocket disconnected: {error}'
                    )
            finally:
                self.clear_navigation_path(send_clear=False)
                self.websocket = None
                self.send_lock = None

            if not self.stop_requested.is_set():
                self.get_logger().info(
                    f'Reconnecting in {self.reconnect_delay:.1f} s'
                )
                await asyncio.sleep(self.reconnect_delay)

    async def ensure_robot_identity(self) -> None:
        if self.robot_credential or not self.robot_enrollment_token:
            return
        client = EnrollmentClient(
            self.server_url,
            self.robot_enrollment_token,
        )
        payload = {
            'serial_number': self.robot_serial_number,
            'hardware_fingerprint': self.hardware_fingerprint,
            'display_name': self.robot_display_name,
            'agent_version': self.agent_version,
            'ros_distro': os.getenv('ROS_DISTRO', 'unknown'),
            'profile_version': self.profile_version,
            'capabilities': self.agent_capabilities,
        }
        while not self.stop_requested.is_set():
            try:
                enrollment = await asyncio.to_thread(client.create, payload)
            except EnrollmentError as error:
                if error.status == 429:
                    delay = max(1, error.retry_after_seconds or 3)
                    self.get_logger().warning(
                        'Robot enrollment is rate limited; retrying the same '
                        f'identity in {delay} s'
                    )
                    await asyncio.sleep(delay)
                    continue
                raise
            self.get_logger().warning(
                'Robot pairing required. Code: '
                f'{enrollment.pairing_code} | Serial: '
                f'{self.robot_serial_number} | Fingerprint: '
                f'{verification_fingerprint(self.hardware_fingerprint)}'
            )
            while not self.stop_requested.is_set():
                try:
                    credential = await asyncio.to_thread(
                        client.claim,
                        enrollment.enrollment_id,
                        enrollment.pairing_code,
                        self.hardware_fingerprint,
                    )
                except EnrollmentError as error:
                    if error.status == 409:
                        await asyncio.sleep(enrollment.poll_after_seconds)
                        continue
                    if error.status == 429:
                        delay = max(
                            enrollment.poll_after_seconds,
                            error.retry_after_seconds or 0,
                        )
                        self.get_logger().warning(
                            'Pairing claim is rate limited; keeping the current '
                            f'code and retrying in {delay} s'
                        )
                        await asyncio.sleep(delay)
                        continue
                    if error.status == 410:
                        self.get_logger().warning(
                            'Pairing code expired; requesting a new code'
                        )
                        break
                    raise
                await asyncio.to_thread(
                    self.credential_store.save,
                    credential,
                )
                self.robot_id = credential.robot_id
                self.robot_credential = credential.credential
                self.robot_credential_version = credential.credential_version
                self.get_logger().info(
                    'Robot paired successfully as '
                    f'{credential.robot_id}; credential stored securely'
                )
                return

    async def send_agent_hello(self, websocket: Any) -> None:
        await self.send_json(
            websocket,
            {
                'type': 'agent_hello',
                'protocol_version': '1.0',
                'robot_id': self.robot_id,
                'boot_id': self.agent_boot_id,
                'agent_version': self.agent_version,
                'ros_distro': os.getenv('ROS_DISTRO', 'unknown'),
                'profile_version': self.profile_version,
                'capabilities': self.agent_capabilities,
                'serial_number': self.robot_serial_number,
                'hardware_fingerprint': self.hardware_fingerprint,
            },
        )

    def refresh_active_map_id(self) -> None:
        """Keep runtime state aligned with the persistent robot-side link."""
        if self.active_map_link:
            self.active_map_id = active_map_id_from_link(
                self.maps_directory,
                self.active_map_link,
            )

    def agent_readiness_snapshot(self) -> dict[str, Any]:
        self.refresh_active_map_id()
        now = time.monotonic()
        mapping_active = self.mapping_runtime.snapshot(
            self.map_revision
        ).get('phase') not in {'IDLE', 'REVIEW', 'FAILED'}
        with self.map_lock:
            map_age = (
                now - self.last_map_monotonic
                if self.last_map_monotonic is not None else None
            )
        with self.velocity_lock:
            odom_age = (
                now - self.last_odom_monotonic
                if self.last_odom_monotonic is not None else None
            )
            velocity = dict(self.latest_velocity) if self.latest_velocity else None
        robot_moving = self.velocity_indicates_motion(
            velocity,
            linear_threshold=self.profile_moving_linear_threshold,
            angular_threshold=self.profile_moving_angular_threshold,
        )
        scan_age = (
            now - self.last_scan_monotonic
            if self.last_scan_monotonic is not None else None
        )
        with self.telemetry_lock:
            battery_age = (
                now - self.last_battery_monotonic
                if self.last_battery_monotonic is not None else None
            )
        physical_estop_age = (
            now - self.last_physical_estop_monotonic
            if self.last_physical_estop_monotonic is not None else None
        )
        with self.localization_lock:
            pose_age = (
                now - self.last_localization_monotonic
                if self.last_localization_monotonic is not None else None
            )
            amcl_state = self.amcl_state
        with self.diagnostics_lock:
            diagnostics_age = (
                now - self.last_diagnostics_monotonic
                if self.last_diagnostics_monotonic is not None else None
            )

        def transform_available(target: str, source: str) -> bool:
            try:
                return self.tf_buffer.can_transform(target, source, Time())
            except Exception:
                return False

        report = validate_robot_profile(
            self.agent_capabilities,
            {
                'navigate_action': self.navigation_client.server_is_ready(),
                'navigate_action_name': self.navigate_action,
                'compute_path_action': self.preview_client.server_is_ready(),
                'compute_path_action_name': self.compute_path_action,
                'odom_age_seconds': odom_age,
                'scan_age_seconds': scan_age,
                'amcl_pose_age_seconds': pose_age,
                'robot_moving': robot_moving,
                'map_age_seconds': map_age,
                'diagnostics_age_seconds': diagnostics_age,
                'tf_map_to_odom': transform_available('map', 'odom'),
                'tf_odom_to_base': transform_available(
                    'odom', self.base_frame
                ),
                'amcl_state': 'INACTIVE' if mapping_active else amcl_state,
                'global_localization_service': (
                    self.global_localization_client.service_is_ready()
                ),
                'global_localization_service_name': (
                    self.global_localization_service
                ),
                'maps_directory': self.maps_directory,
                'load_map_service': self.load_map_client.service_is_ready(),
                'load_map_service_name': self.load_map_service,
                'hardware_contract_mode': self.hardware_contract_mode,
                'battery_age_seconds': battery_age,
                'physical_estop_age_seconds': physical_estop_age,
                'physical_estop_latched': (
                    self.physical_estop_latched.is_set()
                ),
            },
            freshness_seconds=self.profile_data_freshness_seconds,
        )
        checks = {
            item['check_id']: item['status'] == 'PASS'
            for item in report['checks']
        }
        return {
            'type': 'agent_readiness',
            'protocol_version': '1.0',
            'robot_id': self.robot_id,
            'status': report['status'],
            'checks': checks,
            'validation_results': report['checks'],
            'active_map_id': self.active_map_id,
            'detail': report['detail'],
            'timestamp': self.utc_timestamp(),
        }

    @staticmethod
    def velocity_indicates_motion(
        velocity: dict[str, float] | None,
        *,
        linear_threshold: float,
        angular_threshold: float,
    ) -> bool:
        """Reject stationary EKF noise without masking deliberate motion."""
        if not velocity:
            return False
        return (
            abs(float(velocity.get('linear_velocity', 0.0)))
            > linear_threshold
            or abs(float(velocity.get('angular_velocity', 0.0)))
            > angular_threshold
        )

    async def agent_readiness_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await self.send_json(
                websocket,
                self.agent_readiness_snapshot(),
            )
            await asyncio.sleep(2.0)

    async def heartbeat_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await asyncio.sleep(self.heartbeat_period)
            await self.send_json(
                websocket,
                {
                    'type': 'heartbeat',
                    'timestamp': self.utc_timestamp(),
                },
            )

    async def telemetry_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            mapping_phase = self.mapping_runtime.snapshot(
                self.map_revision
            ).get('phase')
            await asyncio.sleep(
                0.2 if mapping_phase == 'MAPPING' else self.telemetry_period
            )

            with self.telemetry_lock:
                telemetry = (
                    dict(self.latest_telemetry)
                    if self.latest_telemetry is not None
                    else None
                )

            if telemetry is None:
                continue

            await self.send_json(
                websocket,
                {
                    'type': 'telemetry',
                    'data': telemetry,
                },
            )

    async def map_loop(self, websocket: Any) -> None:
        sent_revision = -1

        while not self.stop_requested.is_set():
            await asyncio.sleep(0.5)

            with self.map_lock:
                revision = self.map_revision
                map_payload = (
                    dict(self.latest_map)
                    if self.latest_map is not None
                    else None
                )

            if (
                map_payload is None
                or revision == sent_revision
            ):
                continue

            await self.send_json(
                websocket,
                {
                    'type': 'map',
                    'data': map_payload,
                },
            )
            sent_revision = revision

    async def send_map_catalog(self, websocket: Any) -> None:
        self.refresh_active_map_id()
        catalog = build_map_catalog(
            self.maps_directory,
            self.active_map_id,
            self.robot_id,
        )
        await self.send_json(websocket, catalog)

    async def map_catalog_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await self.send_map_catalog(websocket)
            await asyncio.sleep(self.map_catalog_period)

    async def diagnostics_loop(
        self,
        websocket: Any,
    ) -> None:
        sent_revision = -1

        while not self.stop_requested.is_set():
            await asyncio.sleep(1.0)

            with self.diagnostics_lock:
                revision = self.diagnostics_revision
                diagnostics = self.diagnostics_snapshot()

            if (
                diagnostics is None
                or revision == sent_revision
            ):
                continue

            await self.send_json(
                websocket,
                diagnostics,
            )
            sent_revision = revision

    async def path_loop(self, websocket: Any) -> None:
        sent_revision = -1
        while not self.stop_requested.is_set():
            await asyncio.sleep(self.path_publish_period)
            with self.path_lock:
                revision = self.path_revision
                path = (
                    {
                        **self.latest_path,
                        'poses': [
                            dict(pose)
                            for pose in self.latest_path['poses']
                        ],
                    }
                    if self.latest_path is not None
                    else None
                )

            if path is None or revision == sent_revision:
                continue
            await self.send_json(websocket, path)
            sent_revision = revision

    async def mapping_status_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await self.send_mapping_status(websocket)
            await asyncio.sleep(0.25)

    async def send_mapping_status(
        self,
        websocket: Any,
        *,
        command_id: str | None = None,
        accepted: bool = True,
        detail: str | None = None,
    ) -> None:
        status = self.mapping_runtime.snapshot(self.map_revision)
        status['robot_id'] = self.robot_id
        status['command_id'] = command_id
        status['accepted'] = accepted
        if detail is not None:
            status['detail'] = detail
        await self.send_json(websocket, status)

    async def localization_status_loop(self, websocket: Any) -> None:
        while not self.stop_requested.is_set():
            await self.send_localization_status(websocket)
            await asyncio.sleep(0.5)

    async def send_localization_status(
        self,
        websocket: Any,
        *,
        command_id: str | None = None,
        command_action: str | None = None,
        accepted: bool = True,
        detail: str | None = None,
    ) -> None:
        status = self.localization_snapshot()
        status['command_id'] = command_id
        status['command_action'] = command_action
        status['accepted'] = accepted
        if detail is not None:
            status['detail'] = detail
        await self.send_json(websocket, status)

    async def send_json(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        if self.send_lock is None:
            return

        async with self.send_lock:
            await websocket.send(json.dumps(message))

    async def send_current_json(
        self,
        message: dict[str, Any],
    ) -> None:
        websocket = self.websocket
        if websocket is None:
            return

        await self.send_json(websocket, message)

    def send_from_ros(
        self,
        message: dict[str, Any],
    ) -> None:
        loop = self.asyncio_loop
        if loop is None or self.websocket is None:
            self.get_logger().warning(
                'Cannot send message: WebSocket is disconnected'
            )
            return

        future = asyncio.run_coroutine_threadsafe(
            self.send_current_json(message),
            loop,
        )
        future.add_done_callback(self.send_future_callback)

    def send_future_callback(self, future: Any) -> None:
        try:
            future.result()
        except Exception as error:
            if not self.stop_requested.is_set():
                self.get_logger().warning(
                    f'Failed to send WebSocket message: {error}'
                )

    async def receive_loop(self, websocket: Any) -> None:
        async for raw_message in websocket:
            try:
                message = json.loads(raw_message)
            except json.JSONDecodeError:
                self.get_logger().warning('Received invalid JSON')
                continue

            if not isinstance(message, dict):
                self.get_logger().warning(
                    'Received WebSocket message is not an object'
                )
                continue

            await self.handle_message(websocket, message)

    async def handle_message(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        message_type = message.get('type')

        if message_type == 'connection_ack':
            self.get_logger().info(
                'FastAPI acknowledged connection '
                f"for {message.get('robot_id')}"
            )
        elif message_type == 'heartbeat_ack':
            self.get_logger().debug('Heartbeat acknowledged')
        elif message_type == 'telemetry_ack':
            self.get_logger().debug('Telemetry acknowledged')
        elif message_type == 'map_ack':
            self.get_logger().info(
                'Map acknowledged by FastAPI: '
                f"revision {message.get('revision')}"
            )
        elif message_type == 'map_catalog_ack':
            self.get_logger().info(
                'Map catalog acknowledged by FastAPI: '
                f"{message.get('map_count', 0)} maps"
            )
        elif message_type == 'map_catalog_request':
            await self.send_map_catalog(websocket)
        elif message_type == 'credential_rotation':
            await self.handle_credential_rotation(websocket, message)
        elif message_type == 'map_command':
            await self.handle_map_command(websocket, message)
        elif message_type == 'map_catalog_command':
            await self.handle_map_catalog_command(websocket, message)
        elif message_type == 'mapping_command':
            await self.handle_mapping_command(websocket, message)
        elif message_type == 'mapping_teleop':
            await self.handle_mapping_teleop(websocket, message)
        elif message_type == 'localization_teleop':
            await self.handle_localization_teleop(websocket, message)
        elif message_type == 'localization_scan':
            await self.handle_localization_scan(websocket, message)
        elif message_type == 'localization_command':
            await self.handle_localization_command(websocket, message)
        elif message_type == 'command':
            if message.get('action'):
                await self.handle_robot_operation(websocket, message)
            else:
                await self.queue_navigation_command(websocket, message)
        elif message_type == 'route_preview_request':
            await self.queue_route_preview(websocket, message)
        elif message_type == 'cancel_navigation':
            await self.queue_navigation_cancel(
                websocket,
                message,
            )
        elif message_type == 'emergency_command':
            await self.handle_emergency_command(
                websocket,
                message,
            )
        elif message_type == 'command_ack_received':
            self.get_logger().info(
                'FastAPI received command acknowledgement'
            )
        elif message_type == 'command_status_received':
            self.get_logger().debug(
                'FastAPI received command lifecycle status '
                f"{message.get('lifecycle')}"
            )
        elif message_type == 'navigation_result_received':
            self.get_logger().info(
                'FastAPI applied navigation result: '
                f"{message.get('task_status')}"
            )
        elif (
            message_type
            == 'navigation_cancelled_received'
        ):
            self.get_logger().info(
                'FastAPI confirmed task cancellation: '
                f"{message.get('task_id')}"
            )
        elif message_type == 'error':
            self.get_logger().warning(
                'FastAPI error: '
                f"{message.get('code')} - "
                f"{message.get('detail')}"
            )
        else:
            self.get_logger().debug(
                f'Unhandled WebSocket message: {message_type}'
            )

    async def handle_credential_rotation(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        credential = message.get('credential')
        version = message.get('credential_version')
        valid = (
            message.get('protocol_version') == '1.0'
            and message.get('robot_id') == self.robot_id
            and isinstance(credential, str)
            and len(credential) >= 32
            and isinstance(version, int)
            and version > self.robot_credential_version
        )
        if not valid:
            await self.send_json(
                websocket,
                {
                    'type': 'credential_rotation_failed',
                    'protocol_version': '1.0',
                    'robot_id': self.robot_id,
                    'credential_version': version if isinstance(version, int) else 0,
                    'detail': 'Invalid credential rotation command',
                    'timestamp': datetime.now(timezone.utc).isoformat(),
                },
            )
            return
        replacement = AgentCredential(
            robot_id=self.robot_id,
            credential=credential,
            credential_version=version,
            protocol_version='1.0',
        )
        try:
            await asyncio.to_thread(self.credential_store.save, replacement)
        except OSError as error:
            self.get_logger().error(
                f'Credential rotation could not be persisted: {error}'
            )
            await self.send_json(
                websocket,
                {
                    'type': 'credential_rotation_failed',
                    'protocol_version': '1.0',
                    'robot_id': self.robot_id,
                    'credential_version': version,
                    'detail': 'Credential file could not be updated',
                    'timestamp': datetime.now(timezone.utc).isoformat(),
                },
            )
            return
        self.robot_credential = credential
        self.robot_credential_version = version
        await self.send_json(
            websocket,
            {
                'type': 'credential_rotated',
                'protocol_version': '1.0',
                'robot_id': self.robot_id,
                'credential_version': version,
                'timestamp': datetime.now(timezone.utc).isoformat(),
            },
        )
        self.get_logger().info(
            f'Robot credential rotated to version {version}'
        )

    async def handle_mapping_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        action = message.get('action')
        command_id = message.get('command_id')
        session_id = message.get('session_id')
        valid = (
            message.get('robot_id') == self.robot_id
            and action in {'START', 'STOP', 'SAVE', 'DISCARD'}
            and isinstance(command_id, str)
            and command_id.startswith('mapping-')
            and len(command_id) <= 100
            and isinstance(session_id, str)
            and session_id.startswith(f'mapping:{self.robot_id}:')
            and len(session_id) <= 100
        )
        if not valid:
            await self.send_mapping_status(
                websocket, command_id=command_id if isinstance(command_id, str) else None,
                accepted=False, detail='Invalid mapping command',
            )
            return
        try:
            if action == 'START':
                with self.command_lock:
                    navigation_active = self.active_command is not None
                with self.preview_lock:
                    preview_active = (
                        self.active_preview is not None
                        or not self.preview_queue.empty()
                    )
                with self.map_command_lock:
                    map_operation_active = (
                        self.active_map_command is not None
                        or not self.map_command_queue.empty()
                    )
                if self.motion_stop_latched():
                    raise ValueError('Emergency Stop is latched')
                if (
                    navigation_active
                    or preview_active
                    or map_operation_active
                    or self.active_localization_command is not None
                ):
                    raise ValueError('Navigation, route preview, or map switching is active')
                self.capture_mapping_pose_reference()
                await asyncio.to_thread(
                    self.mapping_runtime.start,
                    session_id,
                    self.map_revision,
                )
            elif action == 'STOP':
                self.publish_zero_velocity()
                await asyncio.to_thread(self.mapping_runtime.stop_capture)
            elif action == 'SAVE':
                map_id = message.get('map_id')
                metadata = message.get('metadata')
                if not isinstance(map_id, str) or not isinstance(metadata, dict):
                    raise ValueError('Map ID and metadata are required')
                self.publish_zero_velocity()
                occupancy_map = self.mapping_map_snapshot()
                activate_saved_map = (
                    self.activate_saved_mapping_map
                    if self.active_map_link else None
                )
                await asyncio.to_thread(
                    self.mapping_runtime.save,
                    map_id,
                    metadata,
                    occupancy_map,
                    activate_saved_map,
                )
                self.mapping_pose_reference = None
                await self.send_map_catalog(websocket)
            else:
                self.publish_zero_velocity()
                await asyncio.to_thread(self.mapping_runtime.discard)
                self.restore_mapping_pose_estimate()
            await self.send_mapping_status(websocket, command_id=command_id)
        except Exception as error:
            self.get_logger().error(f'Mapping command {action} failed: {error}')
            await self.send_mapping_status(
                websocket,
                command_id=command_id,
                accepted=False,
                detail=str(error)[:500],
            )

    async def handle_mapping_teleop(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        snapshot = self.mapping_runtime.snapshot(self.map_revision)
        linear_x = message.get('linear_x')
        angular_z = message.get('angular_z')
        valid = (
            message.get('robot_id') == self.robot_id
            and message.get('session_id') == snapshot.get('session_id')
            and snapshot.get('phase') == 'MAPPING'
            and not self.motion_stop_latched()
            and isinstance(linear_x, (int, float))
            and not isinstance(linear_x, bool)
            and isinstance(angular_z, (int, float))
            and not isinstance(angular_z, bool)
            and math.isfinite(float(linear_x))
            and math.isfinite(float(angular_z))
            and abs(float(linear_x)) <= 0.22
            and abs(float(angular_z)) <= 1.0
        )
        if not valid:
            self.get_logger().warning(
                'Rejected invalid mapping teleoperation command'
            )
            return
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.emergency_velocity_publisher.publish(twist)
        with self.mapping_velocity_lock:
            self.mapping_velocity_deadline = time.monotonic() + self.mapping_teleop_deadman_seconds
            self.mapping_velocity_active = bool(linear_x or angular_z)

    async def handle_localization_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        action = message.get('action')
        command_id = message.get('command_id')
        valid = (
            message.get('robot_id') == self.robot_id
            and action in {'SET_INITIAL_POSE', 'GLOBAL_LOCALIZATION'}
            and isinstance(command_id, str)
            and command_id.startswith('localization-')
            and len(command_id) <= 100
        )
        if action == 'SET_INITIAL_POSE':
            pose = message.get('pose')
            valid = valid and self.valid_localization_pose(pose) and all(
                isinstance(message.get(key), (int, float))
                and not isinstance(message.get(key), bool)
                and math.isfinite(float(message[key]))
                for key in ('position_uncertainty', 'yaw_uncertainty')
            )
        with self.command_lock:
            navigation_active = self.active_command is not None
        with self.preview_lock:
            preview_active = (
                self.active_preview is not None or not self.preview_queue.empty()
            )
        with self.map_command_lock:
            map_operation_active = (
                self.active_map_command is not None
                or not self.map_command_queue.empty()
            )
        mapping_active = self.mapping_runtime.snapshot(
            self.map_revision
        ).get('phase') != 'IDLE'
        blocked = (
            navigation_active
            or preview_active
            or map_operation_active
            or mapping_active
            or self.active_localization_command is not None
            or self.motion_stop_latched()
        )
        if not valid or blocked:
            detail = (
                'Invalid localization command' if not valid
                else 'Robot navigation or another maintenance operation is active'
            )
            await self.send_localization_status(
                websocket,
                command_id=command_id if isinstance(command_id, str) else None,
                command_action=action if action in {
                    'SET_INITIAL_POSE', 'GLOBAL_LOCALIZATION'
                } else None,
                accepted=False,
                detail=detail,
            )
            return
        self.active_localization_command = dict(message)
        self.localization_command_queue.put(dict(message))

    async def handle_localization_teleop(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        del websocket
        linear_x = message.get('linear_x')
        angular_z = message.get('angular_z')
        with self.localization_lock:
            recovery_active = self.localization_recovery_active
            scan_active = self.localization_scan_active
        with self.command_lock:
            navigation_active = self.active_command is not None
        mapping_active = self.mapping_runtime.snapshot(
            self.map_revision
        ).get('phase') != 'IDLE'
        valid = (
            message.get('robot_id') == self.robot_id
            and recovery_active
            and not scan_active
            and not navigation_active
            and not mapping_active
            and not self.motion_stop_latched()
            and isinstance(linear_x, (int, float))
            and not isinstance(linear_x, bool)
            and isinstance(angular_z, (int, float))
            and not isinstance(angular_z, bool)
            and math.isfinite(float(linear_x))
            and math.isfinite(float(angular_z))
            and abs(float(linear_x)) <= 0.12
            and abs(float(angular_z)) <= 0.4
        )
        if not valid:
            self.get_logger().warning(
                'Rejected invalid localization recovery command'
            )
            return
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.emergency_velocity_publisher.publish(twist)
        with self.mapping_velocity_lock:
            self.localization_velocity_deadline = (
                time.monotonic() + self.mapping_teleop_deadman_seconds
            )
            self.localization_velocity_active = bool(linear_x or angular_z)

    async def handle_localization_scan(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        del websocket
        action = message.get('action')
        with self.localization_lock:
            recovery_active = self.localization_recovery_active
        with self.command_lock:
            navigation_active = self.active_command is not None
        mapping_active = self.mapping_runtime.snapshot(
            self.map_revision
        ).get('phase') != 'IDLE'
        valid = (
            message.get('robot_id') == self.robot_id
            and action in {'START', 'STOP'}
        )
        if not valid:
            self.get_logger().warning('Rejected invalid localization scan command')
            return
        if action == 'STOP':
            with self.localization_lock:
                self.localization_scan_active = False
                self.localization_scan_started_monotonic = None
            self.publish_zero_velocity()
            return
        if (
            not recovery_active
            or navigation_active
            or mapping_active
            or self.motion_stop_latched()
        ):
            self.get_logger().warning('Rejected unsafe localization scan command')
            return
        with self.localization_lock:
            self.localization_scan_active = True
            self.localization_scan_started_monotonic = time.monotonic()

    def run_localization_scan(self) -> None:
        with self.localization_lock:
            active = self.localization_scan_active
            recovery_active = self.localization_recovery_active
            started = self.localization_scan_started_monotonic
        if not active:
            return
        with self.command_lock:
            navigation_active = self.active_command is not None
        mapping_active = self.mapping_runtime.snapshot(
            self.map_revision
        ).get('phase') != 'IDLE'
        timed_out = bool(
            started is not None
            and time.monotonic() - started >= self.localization_scan_timeout_seconds
        )
        if (
            not recovery_active
            or navigation_active
            or mapping_active
            or timed_out
            or self.motion_stop_latched()
        ):
            with self.localization_lock:
                self.localization_scan_active = False
                self.localization_scan_started_monotonic = None
            self.publish_zero_velocity()
            return
        twist = Twist()
        twist.angular.z = self.localization_scan_angular_speed
        self.emergency_velocity_publisher.publish(twist)

    @staticmethod
    def valid_localization_pose(value: Any) -> bool:
        return (
            isinstance(value, dict)
            and value.get('frame_id') == 'map'
            and all(
                isinstance(value.get(key), (int, float))
                and not isinstance(value.get(key), bool)
                and math.isfinite(float(value[key]))
                for key in ('x', 'y', 'yaw')
            )
            and abs(float(value['x'])) <= 10000.0
            and abs(float(value['y'])) <= 10000.0
            and abs(float(value['yaw'])) <= math.pi + 1e-6
        )

    def process_localization_command_queue(self) -> None:
        try:
            command = self.localization_command_queue.get_nowait()
        except Empty:
            return
        action = str(command['action'])
        if action == 'SET_INITIAL_POSE':
            with self.localization_lock:
                amcl_state = self.amcl_state
            if amcl_state == 'INACTIVE':
                if not self.localization_lifecycle_client.service_is_ready():
                    self.finish_localization_command(
                        command,
                        False,
                        'AMCL is inactive and the localization lifecycle manager is unavailable',
                    )
                    return
                request = ManageLifecycleNodes.Request()
                request.command = ManageLifecycleNodes.Request.RESUME
                future = self.localization_lifecycle_client.call_async(request)
                future.add_done_callback(
                    lambda result: self.localization_resume_callback(result, command)
                )
                return
            self.publish_initial_pose_command(command)
            return
        if not self.global_localization_client.service_is_ready():
            self.finish_localization_command(
                command,
                False,
                'AMCL global localization service is unavailable',
            )
            return
        future = self.global_localization_client.call_async(EmptyService.Request())
        future.add_done_callback(
            lambda result: self.global_localization_callback(result, command)
        )

    def localization_resume_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError('localization lifecycle manager rejected resume')
            with self.localization_lock:
                self.amcl_state = 'UNKNOWN'
            self.publish_initial_pose_command(command)
        except Exception as error:
            self.finish_localization_command(
                command,
                False,
                f'Unable to activate AMCL before setting the initial pose: {error}',
            )

    def publish_initial_pose_command(self, command: dict[str, Any]) -> None:
        pose = command['pose']
        self.publish_initial_pose(
            float(pose['x']),
            float(pose['y']),
            float(pose['yaw']),
            position_uncertainty=float(command['position_uncertainty']),
            yaw_uncertainty=float(command['yaw_uncertainty']),
        )
        with self.localization_lock:
            self.localization_recovery_count += 1
            self.localization_recovery_active = True
            self.localization_recovery_started_monotonic = time.monotonic()
            self.localization_convergence_samples = 0
            self.localization_scan_active = False
            self.localization_scan_started_monotonic = None
        self.finish_localization_command(
            command,
            True,
            'Initial pose published; waiting for AMCL pose and map transform',
        )

    def global_localization_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        try:
            future.result()
            with self.localization_lock:
                self.localization_recovery_count += 1
                self.localization_recovery_active = True
                self.localization_recovery_started_monotonic = time.monotonic()
                self.localization_convergence_samples = 0
                self.localization_scan_active = False
                self.localization_scan_started_monotonic = None
            self.finish_localization_command(
                command,
                True,
                'AMCL global localization started',
            )
        except Exception as error:
            self.finish_localization_command(
                command,
                False,
                f'AMCL global localization failed: {error}',
            )

    def finish_localization_command(
        self,
        command: dict[str, Any],
        accepted: bool,
        detail: str,
    ) -> None:
        payload = self.localization_snapshot()
        payload.update({
            'command_id': command['command_id'],
            'command_action': command['action'],
            'accepted': accepted,
            'detail': detail,
        })
        self.active_localization_command = None
        self.send_from_ros(payload)

    def enforce_mapping_deadman(self) -> None:
        should_stop = False
        with self.mapping_velocity_lock:
            if self.mapping_velocity_active and time.monotonic() >= self.mapping_velocity_deadline:
                self.mapping_velocity_active = False
                should_stop = True
            if (
                self.localization_velocity_active
                and time.monotonic() >= self.localization_velocity_deadline
            ):
                self.localization_velocity_active = False
                should_stop = True
        if should_stop:
            self.publish_zero_velocity()

    async def handle_map_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        command_id = message.get('command_id')
        map_id = message.get('map_id')
        valid = (
            message.get('command') == 'switch_map'
            and message.get('robot_id') == self.robot_id
            and isinstance(command_id, str)
            and command_id.startswith(f'map-switch:{self.robot_id}:')
            and len(command_id) <= 100
            and isinstance(map_id, str)
        )
        if not valid:
            await self.send_json(
                websocket,
                {
                    'type': 'map_switch_result',
                    'command_id': command_id if isinstance(command_id, str) else '',
                    'robot_id': self.robot_id,
                    'map_id': map_id if isinstance(map_id, str) else '',
                    'accepted': False,
                    'detail': 'Invalid map switch command',
                },
            )
            return

        yaml_path = available_map_yaml(self.maps_directory, map_id)
        with self.command_lock:
            navigation_active = self.active_command is not None
        with self.preview_lock:
            preview_active = (
                self.active_preview is not None
                or not self.preview_queue.empty()
            )
        with self.map_command_lock:
            map_switch_active = (
                self.active_map_command is not None
                or not self.map_command_queue.empty()
            )
        if (
            navigation_active
            or preview_active
            or map_switch_active
            or yaml_path is None
        ):
            detail = (
                'Navigation is active'
                if navigation_active else
                'Route preview is active'
                if preview_active else
                'Another map switch is active'
                if map_switch_active else
                'Map is unavailable on the robot'
            )
            await self.send_json(
                websocket,
                {
                    'type': 'map_switch_result',
                    'command_id': command_id,
                    'robot_id': self.robot_id,
                    'map_id': map_id,
                    'accepted': False,
                    'detail': detail,
                },
            )
            return

        queued = dict(message)
        queued['yaml_path'] = str(yaml_path)
        self.map_command_queue.put(queued)
        self.get_logger().info(
            f'Queued Nav2 map switch {command_id} to {map_id}'
        )

    async def handle_map_catalog_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        command_id = message.get('command_id')
        map_id = message.get('map_id')
        action = message.get('action')
        valid = (
            message.get('robot_id') == self.robot_id
            and isinstance(command_id, str)
            and command_id.startswith(f'map-catalog:{self.robot_id}:')
            and len(command_id) <= 100
            and isinstance(map_id, str)
            and MAP_ID_PATTERN.fullmatch(map_id) is not None
            and action in {'UPDATE_METADATA', 'RENAME', 'DELETE'}
        )
        result_map_id = None
        accepted = False
        detail = 'Invalid map catalog command'
        if valid:
            with self.command_lock:
                navigation_active = self.active_command is not None
            with self.preview_lock:
                preview_active = (
                    self.active_preview is not None
                    or not self.preview_queue.empty()
                )
            with self.map_command_lock:
                switch_active = (
                    self.active_map_command is not None
                    or not self.map_command_queue.empty()
                )
            try:
                if switch_active:
                    raise ValueError('A map switch is active')
                if action in {'RENAME', 'DELETE'} and (
                    navigation_active or preview_active
                ):
                    raise ValueError('Navigation or route preview is active')
                if action in {'RENAME', 'DELETE'} and map_id == self.active_map_id:
                    raise ValueError('The active map cannot be changed')
                if action == 'UPDATE_METADATA':
                    metadata = message.get('metadata')
                    if not isinstance(metadata, dict):
                        raise ValueError('Map metadata is invalid')
                    update_map_metadata(self.maps_directory, map_id, metadata)
                    result_map_id = map_id
                    detail = 'Map metadata updated'
                elif action == 'RENAME':
                    new_map_id = message.get('new_map_id')
                    if not isinstance(new_map_id, str):
                        raise ValueError('New map ID is invalid')
                    rename_map(
                        self.maps_directory,
                        map_id,
                        new_map_id,
                        self.active_map_link,
                    )
                    result_map_id = new_map_id
                    detail = 'Map renamed'
                else:
                    delete_map(
                        self.maps_directory,
                        map_id,
                        self.active_map_link,
                    )
                    detail = 'Map deleted'
                accepted = True
            except (OSError, ValueError) as error:
                detail = str(error)

        await self.send_json(websocket, {
            'type': 'map_catalog_operation_result',
            'command_id': command_id if isinstance(command_id, str) else '',
            'robot_id': self.robot_id,
            'map_id': map_id if isinstance(map_id, str) else '',
            'action': action if action in {
                'UPDATE_METADATA', 'RENAME', 'DELETE'
            } else 'UPDATE_METADATA',
            'accepted': accepted,
            'result_map_id': result_map_id,
            'detail': detail,
        })
        if accepted:
            await self.send_map_catalog(websocket)

    async def queue_route_preview(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        request_id = message.get('request_id')

        with self.map_command_lock:
            map_switch_active = (
                self.active_map_command is not None
                or not self.map_command_queue.empty()
            )
        if map_switch_active:
            await self.send_json(
                websocket,
                {
                    'type': 'route_preview_result',
                    'request_id': (
                        request_id if isinstance(request_id, str) else ''
                    ),
                    'status': 'unavailable',
                    'detail': 'Map switch is in progress',
                    'pickup_path': [],
                    'delivery_path': [],
                },
            )
            return

        def valid_pose(value: Any) -> bool:
            return (
                isinstance(value, dict)
                and isinstance(value.get('frame_id'), str)
                and bool(value.get('frame_id'))
                and all(
                    isinstance(value.get(key), (int, float))
                    and not isinstance(value.get(key), bool)
                    and math.isfinite(float(value[key]))
                    for key in ('x', 'y', 'yaw')
                )
            )

        valid = (
            isinstance(request_id, str)
            and bool(request_id)
            and len(request_id) <= 100
            and valid_pose(message.get('start'))
            and valid_pose(message.get('pickup'))
            and valid_pose(message.get('destination'))
        )
        if not valid:
            await self.send_json(
                websocket,
                {
                    'type': 'route_preview_result',
                    'request_id': (
                        request_id
                        if isinstance(request_id, str)
                        else ''
                    ),
                    'status': 'unavailable',
                    'detail': 'Invalid route preview request',
                    'pickup_path': [],
                    'delivery_path': [],
                },
            )
            return

        self.preview_queue.put(message)
        self.get_logger().info(
            f'Queued Nav2 route preview {request_id}'
        )

    async def queue_navigation_cancel(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        cancel_id = message.get('cancel_id')
        task_id = message.get('task_id')

        valid_request = (
            isinstance(cancel_id, str)
            and bool(cancel_id)
            and isinstance(task_id, str)
            and bool(task_id)
            and cancel_id.startswith(
                f'{task_id}:cancel:'
            )
        )

        if not valid_request:
            await self.send_json(
                websocket,
                {
                    'type': 'navigation_cancelled',
                    'cancel_id': cancel_id,
                    'task_id': task_id,
                    'cancelled': False,
                    'detail': (
                        'Invalid navigation '
                        'cancellation request'
                    ),
                },
            )
            return

        self.cancel_queue.put(message)

        self.get_logger().info(
            'Queued Nav2 cancellation '
            f'{cancel_id}'
        )

    async def handle_emergency_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        command_id = message.get('command_id')
        command = message.get('command')
        valid = (
            isinstance(command_id, str)
            and bool(command_id)
            and command in {
                'emergency_stop',
                'emergency_stop_reset',
            }
        )
        if not valid:
            await self.send_json(
                websocket,
                {
                    'type': 'emergency_ack',
                    'command_id': command_id,
                    'command': command,
                    'accepted': False,
                    'detail': 'Invalid Emergency Stop command',
                },
            )
            return

        if command == 'emergency_stop':
            self.emergency_stop_latched.set()
            self.last_emergency_command_id = command_id
            self.clear_command_queue()
            self.clear_navigation_path(
                send_clear=True,
                force=True,
            )
            self.publish_emergency_zero()
            with self.command_lock:
                goal_handle = self.active_goal_handle
                self.active_command = None
                self.active_goal_handle = None
                self.pending_command_ids.clear()
                self.pending_cancel_requests.clear()
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as error:
                    self.get_logger().error(
                        'Emergency Nav2 cancellation request failed: '
                        f'{error}'
                    )
            detail = (
                'Emergency Stop latched; navigation cleared '
                'and zero velocity enforced'
            )
        else:
            self.emergency_stop_latched.clear()
            self.last_emergency_command_id = command_id
            self.publish_zero_velocity()
            detail = (
                'Emergency Stop reset; previous navigation '
                'will not be resumed'
            )

        await self.send_json(
            websocket,
            {
                'type': 'emergency_ack',
                'command_id': command_id,
                'command': command,
                'accepted': True,
                'detail': detail,
            },
        )

    def clear_command_queue(self) -> None:
        while True:
            try:
                self.command_queue.get_nowait()
            except Empty:
                break
        with self.command_lock:
            self.pending_command_ids.clear()

    def publish_zero_velocity(self) -> None:
        self.emergency_velocity_publisher.publish(Twist())

    def motion_stop_latched(self) -> bool:
        physical_latch = getattr(self, 'physical_estop_latched', None)
        return self.emergency_stop_latched.is_set() or (
            physical_latch is not None and physical_latch.is_set()
        )

    def publish_emergency_zero(self) -> None:
        if self.motion_stop_latched():
            self.publish_zero_velocity()

    async def handle_robot_operation(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        action = message.get('action')
        supported = {
            'navigation.recover',
            'motor.reset_stall',
            'navigation.restart_if_broken',
            'system.start_navigation',
            'system.stop_navigation',
            'system.shutdown',
        }
        rejection = self.navigation_command_rejection(message)
        valid = (
            isinstance(message.get('command_id'), str)
            and bool(message.get('command_id'))
            and message.get('robot_id') == self.robot_id
            and action in supported
            and isinstance(message.get('payload', {}), dict)
        )
        with self.robot_operation_lock:
            busy = self.robot_operation_active
            if valid and rejection is None and not busy:
                self.robot_operation_active = True
        if not valid or rejection is not None or busy:
            detail = rejection or (
                'Another robot operation is already running'
                if busy else 'Invalid robot operation command'
            )
            await self.send_json(
                websocket,
                self.command_status_payload(message, 'rejected', detail),
            )
            return

        await self.send_json(
            websocket,
            self.command_status_payload(
                message,
                'accepted',
                'Robot operation accepted',
            ),
        )
        threading.Thread(
            target=self.run_robot_operation,
            args=(dict(message),),
            name=f'robot-operation-{action}',
            daemon=True,
        ).start()

    def run_robot_operation(self, command: dict[str, Any]) -> None:
        action = str(command['action'])
        self.send_command_status(command, 'started', f'Starting {action}')
        try:
            if action == 'navigation.recover':
                succeeded, detail = self.recover_navigation(command)
            elif action == 'motor.reset_stall':
                succeeded, detail = self.reset_motor_stall()
            elif action == 'navigation.restart_if_broken':
                manager_available, lifecycle_detail = (
                    self.navigation_recovery_runner.lifecycle_manager_available()
                )
                if manager_available:
                    succeeded = False
                    detail = (
                        'Nav2 lifecycle server is available; restart was refused'
                    )
                else:
                    self.cancel_active_navigation_for_operation(resume=False)
                    succeeded, output = self.navigation_recovery_runner.system_action(
                        'restart-navigation'
                    )
                    detail = output or (
                        'Nav2 restarted; set Initial Pose before navigation'
                        if succeeded else lifecycle_detail
                    )
            elif action == 'system.start_navigation':
                succeeded, output = self.navigation_recovery_runner.system_action(
                    'start-navigation'
                )
                detail = output or (
                    'Robot ROS and Nav2 stack start requested; set Initial Pose '
                    'if localization is not ready'
                )
            elif action == 'system.stop_navigation':
                self.cancel_active_navigation_for_operation(resume=False)
                succeeded, output = self.navigation_recovery_runner.system_action(
                    'stop-navigation'
                )
                detail = output or 'Robot ROS and Nav2 stack stopped; Agent remains online'
            else:
                self.cancel_active_navigation_for_operation(resume=False)
                succeeded, output = self.navigation_recovery_runner.system_action(
                    'poweroff'
                )
                detail = output or 'ODROID shutdown scheduled'
        except Exception as error:
            succeeded, detail = False, f'Robot operation failed: {error}'
        finally:
            with self.robot_operation_lock:
                self.robot_operation_active = False
        self.send_command_status(
            command,
            'succeeded' if succeeded else 'failed',
            detail[:500],
        )

    def cancel_active_navigation_for_operation(
        self,
        *,
        resume: bool,
    ) -> dict[str, Any] | None:
        self.publish_zero_velocity()
        with self.command_lock:
            active = dict(self.active_command) if self.active_command else None
            goal_handle = self.active_goal_handle
            if active is not None and resume:
                self.recovery_cancelled_command_ids.add(str(active['command_id']))
        if goal_handle is not None:
            future = goal_handle.cancel_goal_async()
            deadline = time.monotonic() + 4.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
        elif active is not None:
            with self.command_lock:
                self.recovery_cancelled_command_ids.discard(
                    str(active['command_id'])
                )
        if active is not None:
            self.clear_active_command(str(active['command_id']))
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                with self.command_lock:
                    pending_result = (
                        str(active['command_id'])
                        in self.recovery_cancelled_command_ids
                    )
                if not pending_result:
                    break
                time.sleep(0.05)
        self.publish_zero_velocity()
        return active if resume else None

    def motor_fault_snapshot(self) -> dict[str, Any] | None:
        with self.diagnostics_lock:
            diagnostics = self.latest_diagnostics
            if diagnostics is None:
                return None
            statuses = list(diagnostics.get('statuses', []))
        for status in statuses:
            if status.get('name') != 'ESP32 base controller':
                continue
            values = {
                str(item.get('key')): str(item.get('value', ''))
                for item in status.get('values', [])
            }
            try:
                mcu_fault = int(values.get('mcu_fault', '-1'))
            except ValueError:
                mcu_fault = -1
            return {
                'mcu_fault': mcu_fault,
                'host_motion_fault': values.get('host_motion_fault', '').strip(),
                'host_motion_state': values.get('host_motion_state', '').strip(),
                'host_motion_reason': values.get('host_motion_reason', '').strip(),
                'motors_enabled': values.get('motors_enabled', '').lower() == 'true',
            }
        return None

    @staticmethod
    def resettable_wheel_fault(fault: dict[str, Any]) -> bool:
        host_fault = str(fault.get('host_motion_fault', '')).strip().upper()
        host_state = str(fault.get('host_motion_state', '')).strip().upper()
        host_reason = str(fault.get('host_motion_reason', '')).strip().upper()
        return int(fault.get('mcu_fault', -1)) in {2, 3} or host_fault in {
            'LEFT_ENCODER_STALL',
            'RIGHT_ENCODER_STALL',
            'LEFT_NO_WHEEL_FEEDBACK',
            'RIGHT_NO_WHEEL_FEEDBACK',
        } or (
            host_state == 'PRE_STALL'
            and host_reason in {
                'LEFT_NO_WHEEL_FEEDBACK',
                'RIGHT_NO_WHEEL_FEEDBACK',
            }
        )

    def maybe_start_automatic_stall_recovery(self) -> None:
        """Start one bounded escape attempt for an active navigation goal."""
        if self.motion_stop_latched():
            return
        fault = self.motor_fault_snapshot()
        if fault is None or not self.resettable_wheel_fault(fault):
            return
        with self.command_lock:
            active = dict(self.active_command) if self.active_command else None
        if active is None:
            return
        command_id = str(active.get('command_id', ''))
        if not command_id:
            return
        with self.automatic_stall_recovery_lock:
            if (
                self.automatic_stall_recovery_active
                or command_id in self.automatic_stall_recovery_attempted_ids
            ):
                return
            with self.robot_operation_lock:
                if self.robot_operation_active:
                    return
                self.robot_operation_active = True
            if len(self.automatic_stall_recovery_attempted_ids) >= 512:
                self.automatic_stall_recovery_attempted_ids.clear()
            self.automatic_stall_recovery_active = True
            self.automatic_stall_recovery_attempted_ids.add(command_id)
        threading.Thread(
            target=self.run_automatic_stall_recovery,
            args=(command_id,),
            name=f'automatic-stall-recovery-{command_id}',
            daemon=True,
        ).start()

    def run_automatic_stall_recovery(self, command_id: str) -> None:
        try:
            succeeded, detail = self.reset_motor_stall()
            log = self.get_logger().warning if succeeded else self.get_logger().error
            log(f'Automatic wheel-stall recovery for {command_id}: {detail}')
        except Exception as error:
            self.get_logger().error(
                f'Automatic wheel-stall recovery for {command_id} failed: '
                f'{error}'
            )
        finally:
            with self.automatic_stall_recovery_lock:
                self.automatic_stall_recovery_active = False
            with self.robot_operation_lock:
                self.robot_operation_active = False

    def robot_is_stationary(self) -> tuple[bool, str]:
        now = time.monotonic()
        with self.velocity_lock:
            velocity = dict(self.latest_velocity) if self.latest_velocity else None
            last_odom = self.last_odom_monotonic
        if velocity is None or last_odom is None or now - last_odom > 1.5:
            return False, 'Fresh odometry is unavailable; motor stall reset was refused'
        linear = abs(float(velocity.get('linear_velocity', 0.0)))
        angular = abs(float(velocity.get('angular_velocity', 0.0)))
        if linear > 0.02 or angular > 0.05:
            return False, (
                'Robot is still moving; motor stall reset was refused '
                f'(linear={linear:.3f} m/s, angular={angular:.3f} rad/s)'
            )
        return True, 'Robot is stationary'

    def stall_escape_decision(
        self,
        *,
        prefer_turn: bool = False,
    ) -> StallEscapeDecision:
        now = time.monotonic()
        with self.scan_lock:
            scan = dict(self.latest_scan) if self.latest_scan else None
            last_scan = self.last_scan_monotonic
        if scan is None or last_scan is None or now - last_scan > 1.0:
            return StallEscapeDecision(
                action='none',
                detail='fresh LaserScan is unavailable',
            )
        return choose_stall_escape(
            scan['ranges'],
            angle_min=float(scan['angle_min']),
            angle_increment=float(scan['angle_increment']),
            range_min=float(scan['range_min']),
            range_max=float(scan['range_max']),
            rear_required=(math.inf if prefer_turn else 0.45),
        )

    @staticmethod
    def _service_response(
        client: Any,
        request: Any,
        *,
        timeout_seconds: float = 1.0,
    ) -> tuple[bool, str]:
        """Wait for a pre-created ROS service client without spinning this node."""
        if not client.wait_for_service(timeout_sec=0.25):
            return False, 'service is unavailable'
        completed = threading.Event()
        result: dict[str, Any] = {}
        try:
            future = client.call_async(request)
        except Exception as error:
            return False, str(error)

        def done_callback(response_future: Any) -> None:
            try:
                result['response'] = response_future.result()
            except Exception as error:  # ROS future errors become a safe refusal.
                result['error'] = str(error)
            finally:
                completed.set()

        future.add_done_callback(done_callback)
        if not completed.wait(timeout_seconds):
            return False, f'service response timed out after {timeout_seconds:.1f}s'
        if result.get('error'):
            return False, str(result['error'])
        response = result.get('response')
        if response is None:
            return False, 'service returned no response'
        return bool(response.success), str(response.message)

    def clear_motor_fault_for_recovery(self) -> tuple[bool, str]:
        client = getattr(self, 'clear_motor_fault_client', None)
        if client is None:
            return self.navigation_recovery_runner.clear_motor_fault()
        return self._service_response(client, Trigger.Request())

    def finish_motor_recovery_for_recovery(
        self,
        success: bool,
    ) -> tuple[bool, str]:
        client = getattr(self, 'finish_motor_recovery_client', None)
        if client is None:
            return self.navigation_recovery_runner.finish_motor_recovery(success)
        request = SetBool.Request()
        request.data = bool(success)
        return self._service_response(client, request)

    def reset_motor_stall(self) -> tuple[bool, str]:
        initial_fault = self.motor_fault_snapshot()
        if initial_fault is None:
            return False, 'ESP32 diagnostics are unavailable; motor stall reset was refused'
        initial_pre_stall = (
            str(initial_fault.get('host_motion_state', '')).strip().upper()
            == 'PRE_STALL'
        )
        self.publish_zero_velocity()
        # The base bridge has already sent STOP upon a pre-stall.  Do not use
        # the longer manual-reset settle delay and lose the recovery window.
        time.sleep(0.10 if initial_pre_stall else 0.35)
        stationary, detail = self.robot_is_stationary()
        if not stationary:
            return False, detail

        fault = self.motor_fault_snapshot() or initial_fault
        mcu_fault = int(fault['mcu_fault'])
        host_fault = str(fault['host_motion_fault'])
        host_state = str(fault.get('host_motion_state', ''))
        if mcu_fault == 0 and not host_fault and host_state != 'PRE_STALL':
            return True, 'No motor stall is currently latched; motors remain enabled'
        if not self.resettable_wheel_fault(fault):
            return False, (
                'The active motor fault is not a resettable wheel feedback fault '
                'and was not cleared '
                f'(mcu_fault={mcu_fault}, host_motion_fault={host_fault or "none"}, '
                f'host_motion_state={host_state or "unknown"})'
            )

        active = self.cancel_active_navigation_for_operation(resume=True)
        escape = (
            self.stall_escape_decision()
            if active is not None
            else None
        )
        if escape is not None and escape.action == 'none':
            if host_state == 'PRE_STALL':
                self.clear_motor_fault_for_recovery()
                self.finish_motor_recovery_for_recovery(False)
            detail = (
                'Motor stall recovery kept the robot stopped because no safe '
                f'escape was found: {escape.detail}'
            )
            self.send_navigation_result(active, 'aborted', detail)
            return False, detail

        cleared, output = self.clear_motor_fault_for_recovery()
        if not cleared:
            if host_state == 'PRE_STALL':
                self.finish_motor_recovery_for_recovery(False)
            detail = f'Motor stall reset service failed: {output or "no response"}'
            if active is not None:
                self.send_navigation_result(active, 'aborted', detail)
            return False, detail

        pre_stall_recovery_pending = host_state == 'PRE_STALL'

        def finish_pre_stall(success: bool) -> tuple[bool, str]:
            nonlocal pre_stall_recovery_pending
            if not pre_stall_recovery_pending:
                return True, ''
            finished, finish_output = self.finish_motor_recovery_for_recovery(success)
            if finished:
                pre_stall_recovery_pending = False
            return finished, finish_output

        deadline = time.monotonic() + 2.5
        latest = fault
        try:
            while time.monotonic() < deadline:
                time.sleep(0.1)
                latest = self.motor_fault_snapshot() or latest
                if (
                    int(latest['mcu_fault']) == 0
                    and not str(latest['host_motion_fault'])
                ):
                    if active is not None:
                        if escape is None:
                            detail = (
                                'Motor stall cleared, but the safe escape decision '
                                'was unavailable; the current goal was not resumed'
                            )
                            self.send_navigation_result(active, 'aborted', detail)
                            return False, detail
                        if escape.action == 'back_up':
                            escaped, escape_output = (
                                self.navigation_recovery_runner.back_up(escape.value)
                            )
                            maneuver = 'short reverse'
                            # A caught wheel can make straight reverse time out.
                            # Re-read the live scan and try exactly one
                            # collision-checked turn toward the clearer side.
                            if not escaped:
                                turn = self.stall_escape_decision(prefer_turn=True)
                                if turn.action == 'spin':
                                    turned, turn_output = (
                                        self.navigation_recovery_runner.spin(
                                            turn.value
                                        )
                                    )
                                    escape_output = '\n'.join(filter(None, (
                                        escape_output,
                                        turn_output,
                                    )))
                                    if turned:
                                        escaped = True
                                        escape = turn
                                        maneuver = (
                                            'short left turn'
                                            if turn.value > 0.0
                                            else 'short right turn'
                                        )
                        else:
                            escaped, escape_output = (
                                self.navigation_recovery_runner.spin(escape.value)
                            )
                            maneuver = (
                                'short left turn'
                                if escape.value > 0.0
                                else 'short right turn'
                            )
                        if not escaped:
                            detail = (
                                'Motor stall cleared, but Nav2 refused every '
                                'collision-checked escape; the current goal was '
                                f'not resumed: {escape_output or "no response"}'
                            )
                            self.send_navigation_result(active, 'aborted', detail)
                            return False, detail
                        finished, finish_output = finish_pre_stall(True)
                        if not finished:
                            detail = (
                                'Nav2 completed the escape, but the base driver '
                                'could not verify the recovery state: '
                                f'{finish_output or "no response"}'
                            )
                            self.send_navigation_result(active, 'aborted', detail)
                            return False, detail
                        self.command_queue.put(active)
                        return True, (
                            'Motor stall reset confirmed by ESP32 diagnostics; '
                            f'Nav2 completed a {maneuver} ({escape.detail}); '
                            'motors remain enabled and the current goal was resumed'
                        )
                    finished, finish_output = finish_pre_stall(True)
                    if not finished:
                        return False, (
                            'Pre-stall was cleared but recovery completion failed: '
                            f'{finish_output or "no response"}'
                        )
                    return True, (
                        'Motor stall reset confirmed by ESP32 diagnostics; '
                        'motors remain enabled and no navigation goal was active'
                    )
            detail = (
                'Motor stall reset was sent but the fault is still latched '
                f'(mcu_fault={latest["mcu_fault"]}, '
                f'host_motion_fault={latest["host_motion_fault"] or "none"})'
            )
            if active is not None:
                self.send_navigation_result(active, 'aborted', detail)
            return False, detail
        finally:
            # Covers action timeout, subprocess failure, unexpected exception,
            # and every early return after begin_recovery().
            finish_pre_stall(False)

    def recovery_lifecycle_healthy(self) -> tuple[bool, str]:
        """Worker-thread only; accept the numeric ACTIVE lifecycle state."""
        for name, client in self.recovery_lifecycle_clients:
            response, detail = call_service(client, GetState.Request())
            if response is None:
                return False, f'{name}: {detail}'
            if response.current_state.id != 3:
                return False, f'{name}: {response.current_state.label}'
        return True, 'Navigation lifecycle nodes are active'

    def mapping_lifecycle_node_active(self, node: str) -> bool:
        """Read a mapping prerequisite through this node's warm ROS client."""
        clients = getattr(self, 'mapping_lifecycle_clients', {})
        client = clients.get(str(node).removeprefix('/'))
        if client is None:
            raise RuntimeError(f'Lifecycle client is unavailable for {node}')
        response, detail = call_service(
            client,
            GetState.Request(),
            timeout_seconds=4.0,
        )
        if response is None:
            raise RuntimeError(f'Unable to read {node} lifecycle state: {detail}')
        return int(response.current_state.id) == 3

    def mapping_lifecycle_command(self, manager: str, command: int) -> None:
        """Transition a lifecycle manager without spawning a ROS CLI process."""
        clients = getattr(self, 'mapping_lifecycle_manager_clients', {})
        client = clients.get(str(manager).removeprefix('/'))
        if client is None:
            raise RuntimeError(f'Lifecycle manager is unavailable: {manager}')
        request = ManageLifecycleNodes.Request()
        request.command = int(command)
        response, detail = call_service(
            client,
            request,
            timeout_seconds=15.0,
        )
        if response is None:
            raise RuntimeError(f'{manager} transition failed: {detail}')
        if not bool(response.success):
            raise RuntimeError(
                f'{manager} did not confirm the lifecycle transition'
            )

    def wait_for_recovery_lifecycle(
        self,
        timeout_seconds: float = 18.0,
        poll_seconds: float = 0.25,
    ) -> tuple[bool, str]:
        """Wait through one bounded Nav2 lifecycle-manager recovery cycle."""
        deadline = time.monotonic() + max(0.1, float(timeout_seconds))
        detail = 'Nav2 lifecycle state is unavailable'
        while time.monotonic() < deadline:
            healthy, detail = self.recovery_lifecycle_healthy()
            if healthy:
                return True, detail
            time.sleep(max(0.0, float(poll_seconds)))
        return False, detail

    def clear_recovery_costmaps(self) -> tuple[bool, str]:
        """Worker-thread only: reuse discovered clients instead of spawning CLI."""
        for name, client in self.recovery_costmap_clients:
            ok, detail = call_empty_service(client, ClearEntireCostmap.Request())
            if not ok:
                return False, f'{name}: {detail}'
        return True, 'Local and global costmaps cleared'

    def guarded_reverse_for_recovery(self) -> tuple[bool, str]:
        """Run the independent scan-guarded reverse without a CLI process."""
        client = getattr(self, 'guarded_reverse_client', None)
        if client is None:
            return self.navigation_recovery_runner.guarded_reverse()
        return WebBridgeNode._service_response(
            client,
            Trigger.Request(),
            timeout_seconds=6.0,
        )

    def spin_for_recovery(
        self,
        yaw: float,
        timeout_seconds: float = 8.0,
    ) -> tuple[bool, str]:
        """Run one short collision-checked Nav2 turn from a worker thread."""
        client = getattr(self, 'recovery_spin_client', None)
        if client is None:
            return self.navigation_recovery_runner.spin(yaw)
        if not client.wait_for_server(timeout_sec=1.0):
            return False, 'Spin action is unavailable'

        completed = threading.Event()
        result: dict[str, Any] = {
            'success': False,
            'detail': 'Spin returned no result',
            'closed': False,
        }
        result_lock = threading.Lock()

        def finish(success: bool, detail: str) -> None:
            with result_lock:
                if result['closed']:
                    return
                result['success'] = success
                result['detail'] = detail
                result['closed'] = True
            completed.set()

        def receive_result(future: Any) -> None:
            try:
                wrapped = future.result()
                action_result = wrapped.result
            except Exception as error:
                finish(False, f'Spin result failed: {error}')
                return
            error_code = int(getattr(action_result, 'error_code', 0))
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
                finish(True, f'Completed a bounded {float(yaw):.2f} rad turn')
                return
            finish(
                False,
                f'Spin stopped (error_code: {error_code}, '
                f'goal_status: {wrapped.status})',
            )

        def receive_goal(future: Any) -> None:
            try:
                goal_handle = future.result()
            except Exception as error:
                finish(False, f'Spin request failed: {error}')
                return
            if not goal_handle.accepted:
                finish(False, 'Spin request was rejected')
                return
            with result_lock:
                result['goal_handle'] = goal_handle
            goal_handle.get_result_async().add_done_callback(receive_result)

        goal = Spin.Goal()
        goal.target_yaw = float(yaw)
        goal.time_allowance.sec = 5
        try:
            client.send_goal_async(goal).add_done_callback(receive_goal)
        except Exception as error:
            return False, f'Spin request failed: {error}'

        if not completed.wait(timeout=max(0.1, float(timeout_seconds))):
            with result_lock:
                result['closed'] = True
                goal_handle = result.get('goal_handle')
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
            return False, f'Spin timed out after {float(timeout_seconds):.1f} seconds'
        return bool(result['success']), str(result['detail'])

    def recover_navigation(self, operation: dict[str, Any]) -> tuple[bool, str]:
        mapping_phase = self.mapping_runtime.snapshot(self.map_revision).get('phase')
        if mapping_phase != 'IDLE':
            return False, 'No connected path: recovery is unavailable during mapping'

        active = self.cancel_active_navigation_for_operation(resume=True)
        payload = operation.get('payload', {})
        target = payload.get('target') if isinstance(payload, dict) else None
        if target is None and active is not None:
            target = active.get('target')
        if not isinstance(target, dict):
            return False, 'No connected path: no current navigation goal is available'

        localization = self.localization_snapshot()
        if (
            localization.get('health') != 'LOCALIZED'
            or localization.get('amcl_state') != 'ACTIVE'
            or not localization.get('tf_available')
        ):
            return False, (
                'Localization: '
                f"{localization.get('detail') or 'AMCL is not localized'}"
            )

        lifecycle_ok, lifecycle_detail = self.recovery_lifecycle_healthy()
        if not lifecycle_ok:
            return False, (
                'No connected path: Nav2 lifecycle is not active; '
                'use the Admin restart control only if the lifecycle server is broken. '
                f'{lifecycle_detail}'
            )

        cleared, clear_detail = self.clear_recovery_costmaps()
        if not cleared:
            return False, f'No connected path: costmap clear failed: {clear_detail}'

        planned, output = self.compute_recovery_path(target)
        if not planned:
            failure = classify_plan_failure(output)
            return False, f'{failure}: {output or "planner returned no path"}'

        if active is not None:
            # Re-run the same delivery stage after the one explicit planning
            # check succeeds. The task identity remains unchanged.
            self.command_queue.put(active)
        return True, 'Navigation recovered: costmaps cleared and route recomputed'

    def maybe_start_blocked_pose_recovery(
        self,
        command: dict[str, Any],
        original_detail: str,
    ) -> bool:
        """Try one scan-guarded reverse after an early collision-like abort."""
        command_id = str(command.get('command_id', 'unknown'))

        def refuse(reason: str) -> bool:
            self.get_logger().warning(
                f'Blocked-pose recovery not started for {command_id}: {reason}'
            )
            return False

        if self.motion_stop_latched():
            return refuse('an emergency stop is latched')
        started = float(command.get('_navigation_started_monotonic', 0.0))
        elapsed = time.monotonic() - started if started > 0.0 else math.inf
        if started <= 0.0 or elapsed > self.blocked_start_recovery_window_seconds:
            return refuse(
                'navigation elapsed time '
                f'{elapsed:.1f}s is outside the '
                f'{self.blocked_start_recovery_window_seconds:.1f}s window'
            )
        escape = self.stall_escape_decision()
        if escape.action != 'back_up':
            return refuse(f'LaserScan did not authorize reverse: {escape.detail}')
        fault = self.motor_fault_snapshot()
        if fault is None or not fault.get('motors_enabled'):
            return refuse('ESP32 diagnostics are unavailable or motors are disabled')
        if (
            int(fault.get('mcu_fault', -1)) != 0
            or str(fault.get('host_motion_fault', '')).strip()
            or str(fault.get('host_motion_state', '')).strip().upper()
            not in {'', 'NORMAL'}
        ):
            return refuse(
                'motor diagnostics are not normal '
                f'(mcu_fault={fault.get("mcu_fault")}, '
                f'host_fault={fault.get("host_motion_fault") or "none"}, '
                f'host_state={fault.get("host_motion_state") or "unknown"})'
            )
        localization = self.localization_snapshot()
        if (
            localization.get('health') != 'LOCALIZED'
            or localization.get('amcl_state') != 'ACTIVE'
            or not localization.get('tf_available')
        ):
            return refuse(
                'localization is not ready '
                f'({localization.get("detail") or localization.get("health")})'
            )

        command_id = str(command.get('command_id', ''))
        if not command_id:
            return refuse('command ID is missing')
        with self.blocked_start_recovery_lock:
            if (
                self.blocked_start_recovery_active
                or command_id in self.blocked_start_recovery_attempted_ids
            ):
                return refuse('this command was already attempted or recovery is busy')
            with self.robot_operation_lock:
                if self.robot_operation_active:
                    return refuse('another robot operation is active')
                self.robot_operation_active = True
            if len(self.blocked_start_recovery_attempted_ids) >= 512:
                self.blocked_start_recovery_attempted_ids.clear()
            self.blocked_start_recovery_active = True
            self.blocked_start_recovery_attempted_ids.add(command_id)

        self.clear_active_command(command_id)
        threading.Thread(
            target=self.run_blocked_pose_recovery,
            args=(dict(command), original_detail, escape.detail),
            name=f'blocked-pose-recovery-{command_id}',
            daemon=True,
        ).start()
        return True

    def accept_aborted_goal_when_near(
        self,
        command: dict[str, Any],
    ) -> tuple[bool, str]:
        """Accept a blocked Nav2 goal only when the robot is safely nearby."""
        if self.motion_stop_latched():
            return False, 'an emergency stop is latched'

        target = command.get('target')
        if not isinstance(target, dict) or str(target.get('frame_id')) != 'map':
            return False, 'the navigation target is not in the map frame'
        try:
            target_x = float(target['x'])
            target_y = float(target['y'])
        except (KeyError, TypeError, ValueError):
            return False, 'the navigation target is invalid'
        if not math.isfinite(target_x) or not math.isfinite(target_y):
            return False, 'the navigation target is invalid'

        localization = self.localization_snapshot()
        pose = localization.get('pose')
        if (
            localization.get('health') != 'LOCALIZED'
            or localization.get('amcl_state') != 'ACTIVE'
            or not localization.get('tf_available')
            or not isinstance(pose, dict)
            or str(pose.get('frame_id')) != 'map'
        ):
            return False, (
                'localization is not ready '
                f'({localization.get("detail") or localization.get("health")})'
            )

        fault = self.motor_fault_snapshot()
        if fault is None or not fault.get('motors_enabled'):
            return False, 'ESP32 diagnostics are unavailable or motors are disabled'
        if (
            int(fault.get('mcu_fault', -1)) != 0
            or str(fault.get('host_motion_fault', '')).strip()
            or str(fault.get('host_motion_state', '')).strip().upper()
            not in {'', 'NORMAL'}
        ):
            return False, 'motor diagnostics are not normal'

        try:
            distance = math.hypot(
                float(pose['x']) - target_x,
                float(pose['y']) - target_y,
            )
        except (KeyError, TypeError, ValueError):
            return False, 'the localized robot position is invalid'
        if not math.isfinite(distance):
            return False, 'the localized robot position is invalid'
        if distance > self.near_goal_acceptance_distance_m:
            return False, (
                f'robot is {distance:.2f} m from the goal, outside the '
                f'{self.near_goal_acceptance_distance_m:.2f} m acceptance radius'
            )

        return True, (
            f'Accepted arrival {distance:.2f} m from the goal after Nav2 was '
            'blocked near the destination'
        )

    def run_blocked_pose_recovery(
        self,
        command: dict[str, Any],
        original_detail: str,
        scan_detail: str,
    ) -> None:
        command_id = str(command.get('command_id', ''))
        try:
            self.send_command_status(
                command,
                'started',
                'Nav2 reported a blocked start; checking a guarded reverse escape',
            )
            escaped, escape_output = self.guarded_reverse_for_recovery()
            if not escaped:
                detail = (
                    f'{original_detail}; guarded reverse was stopped or refused '
                    f'({scan_detail}): {escape_output or "no response"}'
                )
                self.get_logger().error(
                    f'Blocked-pose recovery for {command_id} failed: {detail}'
                )
                self.send_navigation_result(command, 'aborted', detail[:500])
                return

            lifecycle_ok, lifecycle_detail = self.wait_for_recovery_lifecycle()
            if not lifecycle_ok:
                detail = (
                    'Guarded reverse completed, but Nav2 did not return to an '
                    f'active state: {lifecycle_detail}'
                )
                self.get_logger().error(
                    f'Blocked-pose recovery for {command_id} failed: {detail}'
                )
                self.send_navigation_result(command, 'aborted', detail[:500])
                return

            turn = self.stall_escape_decision(prefer_turn=True)
            if turn.action == 'spin':
                turned, turn_output = self.spin_for_recovery(turn.value)
                if turned:
                    self.get_logger().warning(
                        f'Blocked-pose recovery for {command_id} completed '
                        f'a scan-gated turn: {turn.detail}'
                    )
                else:
                    # The turn is an additional escape opportunity. A refused
                    # turn must remain stopped, but a fresh route may still be
                    # safe after the completed guarded reverse.
                    self.get_logger().warning(
                        f'Optional turn for {command_id} was not completed: '
                        f'{turn_output}'
                    )

            cleared, clear_output = self.clear_recovery_costmaps()
            if not cleared:
                detail = (
                    'Guarded reverse completed, but Nav2 costmaps could not be '
                    f'cleared: {clear_output or "no response"}'
                )
                self.get_logger().error(
                    f'Blocked-pose recovery for {command_id} failed after reverse: '
                    f'{detail}'
                )
                self.send_navigation_result(command, 'aborted', detail[:500])
                return

            planned, plan_output = self.compute_recovery_path(command['target'])
            if not planned and 'request was rejected' in plan_output.lower():
                lifecycle_ok, lifecycle_detail = self.wait_for_recovery_lifecycle()
                if lifecycle_ok:
                    cleared, clear_output = self.clear_recovery_costmaps()
                    if cleared:
                        planned, plan_output = self.compute_recovery_path(
                            command['target']
                        )
                else:
                    plan_output = (
                        f'{plan_output}; Nav2 lifecycle did not recover: '
                        f'{lifecycle_detail}'
                    )
            if not planned:
                failure = classify_plan_failure(plan_output)
                detail = (
                    'Guarded reverse completed, but the route is still unavailable: '
                    f'{failure}: {plan_output or "planner returned no path"}'
                )
                self.get_logger().error(
                    f'Blocked-pose recovery for {command_id} failed after reverse: '
                    f'{detail}'
                )
                self.send_navigation_result(command, 'aborted', detail[:500])
                return

            command['_navigation_started_monotonic'] = 0.0
            self.command_queue.put(command)
            self.get_logger().warning(
                f'Blocked-pose recovery for {command_id} completed; retrying goal once'
            )
        except Exception as error:
            detail = f'Blocked-pose recovery failed: {error}'
            self.get_logger().error(f'{command_id}: {detail}')
            self.send_navigation_result(command, 'aborted', detail[:500])
        finally:
            with self.blocked_start_recovery_lock:
                self.blocked_start_recovery_active = False
            with self.robot_operation_lock:
                self.robot_operation_active = False

    async def queue_navigation_command(
        self,
        websocket: Any,
        message: dict[str, Any],
    ) -> None:
        command_id = message.get('command_id')
        command_name = message.get('command')
        task_id = message.get('task_id')
        stage = message.get('stage')
        target = message.get('target')

        rejection = self.navigation_command_rejection(message)
        if rejection is not None:
            await self.send_json(
                websocket,
                self.command_status_payload(
                    message,
                    'rejected',
                    rejection,
                ),
            )
            await self.send_json(
                websocket,
                {
                    'type': 'command_ack',
                    'command_id': command_id,
                    'accepted': False,
                    'detail': rejection,
                },
            )
            return

        if self.motion_stop_latched():
            await self.send_json(
                websocket,
                self.command_status_payload(
                    message,
                    'rejected',
                    'Emergency Stop is latched',
                ),
            )
            await self.send_json(
                websocket,
                {
                    'type': 'command_ack',
                    'command_id': command_id,
                    'accepted': False,
                    'detail': 'Emergency Stop is latched',
                },
            )
            return

        valid_target = (
            isinstance(target, dict)
            and isinstance(target.get('frame_id'), str)
            and bool(target.get('frame_id'))
            and isinstance(target.get('x'), (int, float))
            and isinstance(target.get('y'), (int, float))
            and isinstance(target.get('yaw'), (int, float))
        )
        valid_command = (
            isinstance(command_id, str)
            and bool(command_id)
            and command_name == 'navigate_to_pose'
            and isinstance(task_id, str)
            and bool(task_id)
            and stage in {'pickup', 'destination'}
            and valid_target
        )

        if not valid_command:
            await self.send_json(
                websocket,
                self.command_status_payload(
                    message,
                    'rejected',
                    'Invalid navigate_to_pose command',
                ),
            )
            await self.send_json(
                websocket,
                {
                    'type': 'command_ack',
                    'command_id': command_id,
                    'accepted': False,
                    'detail': 'Invalid navigate_to_pose command',
                },
            )
            return

        with self.command_lock:
            active_id = (
                self.active_command.get('command_id')
                if self.active_command is not None
                else None
            )
            duplicate = (
                command_id == active_id
                or command_id in self.pending_command_ids
                or command_id in self.processed_command_ids
            )

            if not duplicate:
                self.pending_command_ids.add(command_id)
                self.processed_command_ids[command_id] = None
                while (
                    len(self.processed_command_ids)
                    > self.processed_command_limit
                ):
                    self.processed_command_ids.popitem(last=False)

        if duplicate:
            await self.send_json(
                websocket,
                self.command_status_payload(
                    message,
                    'accepted',
                    'Command was already processed',
                ),
            )
            await self.send_json(
                websocket,
                {
                    'type': 'command_ack',
                    'command_id': command_id,
                    'accepted': True,
                    'detail': 'Command is already queued or active',
                },
            )
            return

        self.command_queue.put(message)
        await self.send_json(
            websocket,
            self.command_status_payload(
                message,
                'accepted',
                'Command accepted into the local execution queue',
            ),
        )
        self.get_logger().info(
            f'Queued Nav2 command {command_id}'
        )

    def navigation_command_rejection(
        self,
        message: dict[str, Any],
    ) -> str | None:
        addressed_robot = message.get('robot_id')
        if (
            addressed_robot is not None
            and addressed_robot != self.robot_id
        ):
            return 'Command is addressed to another robot'

        expected_profile = message.get('expected_profile_version')
        if (
            expected_profile is not None
            and expected_profile != self.profile_version
        ):
            return 'Robot profile version does not match the command'

        expected_map_revision = message.get('expected_map_revision')
        if (
            expected_map_revision is not None
            and expected_map_revision != self.map_revision
        ):
            return 'Map revision does not match the command'

        expires_at = message.get('expires_at')
        if expires_at is None:
            return None
        if not isinstance(expires_at, str):
            return 'Command expiry is invalid'

        try:
            normalized = expires_at.replace('Z', '+00:00')
            expiry = datetime.fromisoformat(normalized)
            if expiry.tzinfo is None:
                return 'Command expiry must include a timezone'
            if expiry <= datetime.now(timezone.utc):
                return 'Command has expired'
        except ValueError:
            return 'Command expiry is invalid'

        return None

    def command_status_payload(
        self,
        command: dict[str, Any],
        lifecycle: str,
        detail: str,
    ) -> dict[str, Any]:
        return {
            'type': 'command_status',
            'protocol_version': '1.0',
            'command_id': command.get('command_id') or 'unknown',
            'robot_id': self.robot_id,
            'lifecycle': lifecycle,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'detail': detail,
        }

    def process_cancel_queue(self) -> bool:
        try:
            cancel_request = (
                self.cancel_queue.get_nowait()
            )
        except Empty:
            return False

        cancel_id = str(
            cancel_request['cancel_id']
        )
        task_id = str(
            cancel_request['task_id']
        )

        with self.command_lock:
            self.pending_cancel_requests[
                task_id
            ] = cancel_request
            active_command = self.active_command
            active_goal_handle = (
                self.active_goal_handle
            )

        if active_command is None:
            with self.command_lock:
                self.cancelled_task_ids.add(
                    task_id
                )
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.get_logger().info(
                'No active Nav2 goal for '
                f'{task_id}; cancellation confirmed'
            )
            self.send_navigation_cancelled(
                cancel_request,
                True,
                'No active Nav2 goal remains',
            )
            return True

        active_task_id = str(
            active_command.get('task_id')
        )

        if active_task_id != task_id:
            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.get_logger().warning(
                f'Cannot cancel {task_id}; '
                f'active task is {active_task_id}'
            )
            self.send_navigation_cancelled(
                cancel_request,
                False,
                (
                    'Cancellation task does not match '
                    'the active Nav2 goal'
                ),
            )
            return True

        if active_goal_handle is None:
            self.get_logger().info(
                'Waiting for Nav2 goal handle before '
                f'cancelling {cancel_id}'
            )
            return True

        self.request_goal_cancellation(
            active_command,
            active_goal_handle,
            cancel_request,
        )
        return True

    def request_goal_cancellation(
        self,
        command: dict[str, Any],
        goal_handle: Any,
        cancel_request: dict[str, Any],
    ) -> None:
        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )
        cancel_id = str(
            cancel_request['cancel_id']
        )

        self.get_logger().info(
            'Requesting Nav2 cancellation '
            f'{cancel_id} for {command_id}'
        )

        try:
            cancel_future = (
                goal_handle.cancel_goal_async()
            )
        except Exception as error:
            detail = (
                'Failed to request Nav2 '
                f'cancellation: {error}'
            )
            self.get_logger().error(detail)

            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.send_navigation_cancelled(
                cancel_request,
                False,
                detail,
            )
            return

        cancel_future.add_done_callback(
            lambda future: (
                self.cancel_response_callback(
                    future,
                    command,
                    cancel_request,
                )
            )
        )

    def cancel_response_callback(
        self,
        future: Any,
        command: dict[str, Any],
        cancel_request: dict[str, Any],
    ) -> None:
        task_id = str(
            command['task_id']
        )
        cancel_id = str(
            cancel_request['cancel_id']
        )

        try:
            response = future.result()
        except Exception as error:
            detail = (
                'Failed to receive Nav2 '
                f'cancellation response: {error}'
            )
            self.get_logger().error(detail)

            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.send_navigation_cancelled(
                cancel_request,
                False,
                detail,
            )
            return

        if not response.goals_canceling:
            detail = (
                'Nav2 did not accept the '
                'cancellation request'
            )
            self.get_logger().warning(
                f'{detail}: {cancel_id}'
            )

            with self.command_lock:
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )

            self.send_navigation_cancelled(
                cancel_request,
                False,
                detail,
            )
            return

        self.get_logger().info(
            'Nav2 accepted cancellation request '
            f'{cancel_id}; waiting for final result'
        )

    def process_preview_queue(self) -> None:
        with self.preview_lock:
            if self.active_preview is not None:
                return

        try:
            request = self.preview_queue.get_nowait()
        except Empty:
            return

        state = {
            'request': request,
            'pickup_path': None,
            'frame_id': None,
        }
        with self.preview_lock:
            self.active_preview = state

        if not self.preview_client.wait_for_server(timeout_sec=1.0):
            self.finish_route_preview(
                request,
                'unavailable',
                'Nav2 ComputePathToPose server is unavailable',
            )
            return

        self.send_preview_leg(
            state,
            'pickup',
            request['start'],
            request['pickup'],
        )

    def build_preview_goal(
        self,
        start: dict[str, Any],
        target: dict[str, Any],
        *,
        use_start: bool,
    ) -> ComputePathToPose.Goal:
        goal = ComputePathToPose.Goal()
        stamp = self.get_clock().now().to_msg()
        goal.start.header.stamp = stamp
        goal.start.header.frame_id = str(start['frame_id'])
        goal.start.pose.position.x = float(start['x'])
        goal.start.pose.position.y = float(start['y'])
        goal.start.pose.orientation.z = math.sin(float(start['yaw']) / 2.0)
        goal.start.pose.orientation.w = math.cos(float(start['yaw']) / 2.0)
        goal.goal.header.stamp = stamp
        goal.goal.header.frame_id = str(target['frame_id'])
        goal.goal.pose.position.x = float(target['x'])
        goal.goal.pose.position.y = float(target['y'])
        goal.goal.pose.orientation.z = math.sin(float(target['yaw']) / 2.0)
        goal.goal.pose.orientation.w = math.cos(float(target['yaw']) / 2.0)
        goal.planner_id = ''
        goal.use_start = use_start
        return goal

    def build_recovery_plan_goal(
        self,
        target: dict[str, Any],
    ) -> ComputePathToPose.Goal:
        goal = ComputePathToPose.Goal()
        yaw = float(target['yaw'])
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.header.frame_id = str(target['frame_id'])
        goal.goal.pose.position.x = float(target['x'])
        goal.goal.pose.position.y = float(target['y'])
        goal.goal.pose.position.z = 0.0
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        goal.planner_id = 'GridBased'
        goal.use_start = False
        return goal

    def compute_recovery_path(
        self,
        target: dict[str, Any],
        timeout_seconds: float = 15.0,
    ) -> tuple[bool, str]:
        """
        Verify a route synchronously from a recovery worker thread.

        The node's executor continues spinning on the main thread and resolves
        these action futures. Reusing this node's DDS participant avoids the
        discovery delay and CPU burst caused by a new ``ros2`` CLI process.
        """
        if not self.recovery_plan_client.wait_for_server(timeout_sec=1.0):
            return False, 'Planner unavailable: ComputePathToPose action is unavailable'

        completed = threading.Event()
        result: dict[str, Any] = {
            'success': False,
            'detail': 'Planner unavailable: no recovery result was received',
            'closed': False,
        }
        result_lock = threading.Lock()

        def finish(success: bool, detail: str) -> None:
            with result_lock:
                if result['closed']:
                    return
                result['success'] = success
                result['detail'] = detail
                result['closed'] = True
            completed.set()

        def receive_result(future: Any) -> None:
            try:
                wrapped = future.result()
                action_result = wrapped.result
            except Exception as error:
                finish(False, f'Planner unavailable: failed to receive result: {error}')
                return

            error_code = int(getattr(action_result, 'error_code', 0))
            if (
                wrapped.status != GoalStatus.STATUS_SUCCEEDED
                or error_code != 0
            ):
                error_message = str(
                    getattr(action_result, 'error_msg', '')
                    or 'Nav2 could not compute a connected recovery path'
                )
                finish(
                    False,
                    f'{error_message} (error_code: {error_code}, '
                    f'goal_status: {wrapped.status})',
                )
                return

            path = getattr(action_result, 'path', None)
            pose_count = len(getattr(path, 'poses', [])) if path is not None else 0
            if pose_count == 0:
                finish(False, 'Nav2 planner returned an empty recovery path')
                return
            finish(True, f'Nav2 computed a connected path with {pose_count} poses')

        def receive_goal(future: Any) -> None:
            try:
                goal_handle = future.result()
            except Exception as error:
                finish(False, f'Planner unavailable: failed to send goal: {error}')
                return
            if not goal_handle.accepted:
                finish(False, 'Planner unavailable: recovery path request was rejected')
                return
            with result_lock:
                result['goal_handle'] = goal_handle
            goal_handle.get_result_async().add_done_callback(receive_result)

        try:
            goal_future = self.recovery_plan_client.send_goal_async(
                self.build_recovery_plan_goal(target)
            )
            goal_future.add_done_callback(receive_goal)
        except Exception as error:
            return False, f'Planner unavailable: failed to request route: {error}'

        if not completed.wait(timeout=max(0.1, float(timeout_seconds))):
            with result_lock:
                result['closed'] = True
                goal_handle = result.get('goal_handle')
            if goal_handle is not None:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
            return False, (
                'Planner timeout: ComputePathToPose did not return within '
                f'{float(timeout_seconds):.1f} seconds'
            )

        return bool(result['success']), str(result['detail'])

    def send_preview_leg(
        self,
        state: dict[str, Any],
        leg: str,
        start: dict[str, Any],
        target: dict[str, Any],
    ) -> None:
        request = state['request']
        future = self.preview_client.send_goal_async(
            self.build_preview_goal(
                start,
                target,
                use_start=leg != 'pickup',
            )
        )
        future.add_done_callback(
            lambda response: self.preview_goal_response(
                response,
                state,
                leg,
            )
        )
        self.get_logger().info(
            'Requesting Nav2 route preview '
            f"{request['request_id']} ({leg})"
        )

    def preview_goal_response(
        self,
        future: Any,
        state: dict[str, Any],
        leg: str,
    ) -> None:
        request = state['request']
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_route_preview(
                request,
                'unavailable',
                f'Failed to request Nav2 route preview: {error}',
            )
            return

        if not goal_handle.accepted:
            self.finish_route_preview(
                request,
                'unavailable',
                'Nav2 rejected the route preview request',
            )
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.preview_result_callback(
                result,
                state,
                leg,
            )
        )

    def preview_result_callback(
        self,
        future: Any,
        state: dict[str, Any],
        leg: str,
    ) -> None:
        request = state['request']
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            self.finish_route_preview(
                request,
                'unavailable',
                f'Failed to receive Nav2 route preview: {error}',
            )
            return

        error_code = int(getattr(result, 'error_code', 0))
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or error_code != 0
        ):
            detail = getattr(result, 'error_msg', '') or (
                f'Nav2 could not compute the {leg} route'
            )
            self.finish_route_preview(
                request,
                'unreachable',
                str(detail),
            )
            return

        serialized = serialize_preview_path(
            result.path,
            self.path_max_poses,
        )
        if serialized is None:
            self.finish_route_preview(
                request,
                'unreachable',
                f'Nav2 returned no {leg} path',
            )
            return

        frame_id, poses = serialized
        if leg == 'pickup':
            state['pickup_path'] = poses
            state['frame_id'] = frame_id
            self.send_preview_leg(
                state,
                'destination',
                request['pickup'],
                request['destination'],
            )
            return

        if state.get('frame_id') != frame_id:
            self.finish_route_preview(
                request,
                'unavailable',
                'Nav2 preview paths use incompatible frames',
            )
            return

        self.finish_route_preview(
            request,
            'available',
            'Both route segments are reachable',
            frame_id=frame_id,
            pickup_path=state['pickup_path'],
            delivery_path=poses,
        )

    def finish_route_preview(
        self,
        request: dict[str, Any],
        status: str,
        detail: str,
        *,
        frame_id: str | None = None,
        pickup_path: list[dict[str, float]] | None = None,
        delivery_path: list[dict[str, float]] | None = None,
    ) -> None:
        request_id = str(request.get('request_id', ''))
        with self.preview_lock:
            active = self.active_preview
            if (
                active is None
                or active['request'].get('request_id') != request_id
            ):
                return
            self.active_preview = None

        self.send_from_ros(
            {
                'type': 'route_preview_result',
                'request_id': request_id,
                'status': status,
                'frame_id': frame_id,
                'pickup_path': pickup_path or [],
                'delivery_path': delivery_path or [],
                'detail': detail,
            }
        )
        self.get_logger().info(
            f'Nav2 route preview {request_id}: {status}'
        )

    def process_map_command_queue(self) -> None:
        with self.map_command_lock:
            if self.active_map_command is not None:
                return
        try:
            command = self.map_command_queue.get_nowait()
        except Empty:
            return

        with self.command_lock:
            if self.active_command is not None:
                self.finish_map_switch(
                    command,
                    False,
                    'Navigation became active before the map switch',
                )
                return
        with self.preview_lock:
            if (
                self.active_preview is not None
                or not self.preview_queue.empty()
            ):
                self.finish_map_switch(
                    command,
                    False,
                    'Route preview became active before the map switch',
                )
                return
        with self.map_command_lock:
            self.active_map_command = command

        if not self.load_map_client.wait_for_service(timeout_sec=1.0):
            if self.active_map_link:
                succeeded, output = (
                    self.navigation_recovery_runner.activate_map(
                        str(command['yaml_path'])
                    )
                )
                if succeeded:
                    self.active_map_id = str(command['map_id'])
                    self.finish_map_switch(
                        command,
                        True,
                        'Active map saved; localization and Nav2 are starting. '
                        'Set Initial Pose after the map is ready.',
                    )
                    return
                self.finish_map_switch(
                    command,
                    False,
                    output or 'Unable to start navigation with the selected map',
                )
                return
            self.finish_map_switch(
                command,
                False,
                'No active map is running. Select a valid map to start '
                'localization and Nav2.',
            )
            return

        request = LoadMap.Request()
        request.map_url = str(command['yaml_path'])
        future = self.load_map_client.call_async(request)
        future.add_done_callback(
            lambda result: self.map_switch_result_callback(result, command)
        )
        self.get_logger().info(
            f"Requesting Nav2 map switch to {command['map_id']}"
        )

    def map_switch_result_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        try:
            response = future.result()
            result_code = int(response.result)
        except Exception as error:
            self.finish_map_switch(
                command,
                False,
                f'Nav2 LoadMap request failed: {error}',
            )
            return

        if result_code != LoadMap.Response.RESULT_SUCCESS:
            details = {
                LoadMap.Response.RESULT_MAP_DOES_NOT_EXIST: 'Map does not exist',
                LoadMap.Response.RESULT_INVALID_MAP_DATA: 'Map data is invalid',
                LoadMap.Response.RESULT_INVALID_MAP_METADATA: 'Map metadata is invalid',
                LoadMap.Response.RESULT_UNDEFINED_FAILURE: 'Nav2 map server failed',
            }
            self.finish_map_switch(
                command,
                False,
                details.get(result_code, f'Nav2 returned result {result_code}'),
            )
            return

        self.active_map_id = str(command['map_id'])
        detail = 'Nav2 map server loaded the map'
        if self.active_map_link:
            try:
                set_active_map_link(
                    self.maps_directory,
                    self.active_map_link,
                    self.active_map_id,
                )
            except (OSError, ValueError) as error:
                self.get_logger().error(
                    f'Nav2 loaded {self.active_map_id}, but it could not be '
                    f'persisted as the active map: {error}'
                )
                detail = (
                    'Nav2 map server loaded the map, but it was not saved '
                    'as the boot map'
                )
        if int(response.map.info.width) > 0 and int(response.map.info.height) > 0:
            self.map_callback(response.map)
        self.finish_map_switch(command, True, detail)

    def finish_map_switch(
        self,
        command: dict[str, Any],
        accepted: bool,
        detail: str,
    ) -> None:
        command_id = str(command.get('command_id', ''))
        with self.map_command_lock:
            active = self.active_map_command
            if active is not None and active.get('command_id') == command_id:
                self.active_map_command = None
        self.send_from_ros(
            {
                'type': 'map_switch_result',
                'command_id': command_id,
                'robot_id': self.robot_id,
                'map_id': str(command.get('map_id', '')),
                'accepted': accepted,
                'detail': detail,
            }
        )
        if accepted:
            self.send_from_ros(
                build_map_catalog(
                    self.maps_directory,
                    self.active_map_id,
                    self.robot_id,
                )
            )
            self.send_from_ros(self.agent_readiness_snapshot())
        self.get_logger().info(
            f"Map switch {command_id}: {'succeeded' if accepted else 'failed'}"
        )

    def process_command_queue(self) -> None:
        # Cancellation always takes priority over
        # starting another navigation command.
        if self.process_cancel_queue():
            return

        with self.map_command_lock:
            if (
                self.active_map_command is not None
                or not self.map_command_queue.empty()
            ):
                return

        if self.motion_stop_latched():
            self.clear_command_queue()
            return

        with self.command_lock:
            if self.active_command is not None:
                return

        try:
            command = self.command_queue.get_nowait()
        except Empty:
            return

        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )

        with self.command_lock:
            skip_cancelled_command = (
                task_id in self.cancelled_task_ids
            )
            self.pending_command_ids.discard(
                command_id
            )

            if skip_cancelled_command:
                self.cancelled_task_ids.discard(
                    task_id
                )
            else:
                self.active_command = command

        self.clear_navigation_path(send_clear=False)

        if skip_cancelled_command:
            self.get_logger().info(
                'Skipping queued Nav2 command '
                f'{command_id} because task '
                f'{task_id} was cancelled'
            )
            return

        if not self.navigation_client.wait_for_server(
            timeout_sec=1.0
        ):
            detail = 'Nav2 NavigateToPose server is unavailable'
            self.get_logger().error(detail)
            self.send_command_ack(command, False, detail)
            self.send_navigation_result(
                command,
                'aborted',
                detail,
            )
            self.clear_active_command(command_id)
            return

        goal = self.build_navigation_goal(command)
        command['_navigation_started_monotonic'] = time.monotonic()
        self.get_logger().info(
            'Sending Nav2 goal '
            f"{command_id} to {command['target']}"
        )

        goal_future = self.navigation_client.send_goal_async(
            goal,
            feedback_callback=(
                lambda feedback_message: (
                    self.navigation_feedback_callback(
                        feedback_message,
                        command,
                    )
                )
            ),
        )

        goal_future.add_done_callback(
            lambda future: self.goal_response_callback(
                future,
                command,
            )
        )

    def build_navigation_goal(
        self,
        command: dict[str, Any],
    ) -> NavigateToPose.Goal:
        target = command['target']
        yaw = float(target['yaw'])

        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = str(target['frame_id'])
        goal.pose.pose.position.x = float(target['x'])
        goal.pose.pose.position.y = float(target['y'])
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def goal_response_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )

        try:
            goal_handle = future.result()
        except Exception as error:
            detail = (
                f'Failed to send Nav2 goal: {error}'
            )
            self.get_logger().error(detail)
            self.send_command_ack(
                command,
                False,
                detail,
            )

            with self.command_lock:
                cancel_request = (
                    self.pending_cancel_requests.pop(
                        task_id,
                        None,
                    )
                )

            if cancel_request is not None:
                self.send_navigation_cancelled(
                    cancel_request,
                    True,
                    (
                        'Nav2 goal never became active: '
                        f'{detail}'
                    ),
                )
            else:
                self.send_navigation_result(
                    command,
                    'aborted',
                    detail,
                )

            self.clear_active_command(
                command_id
            )
            return

        if not goal_handle.accepted:
            detail = (
                'Nav2 rejected the navigation goal'
            )
            self.get_logger().warning(detail)
            self.send_command_ack(
                command,
                False,
                detail,
            )

            with self.command_lock:
                cancel_request = (
                    self.pending_cancel_requests.pop(
                        task_id,
                        None,
                    )
                )

            if cancel_request is not None:
                self.send_navigation_cancelled(
                    cancel_request,
                    True,
                    (
                        'Nav2 goal was not active '
                        'because it was rejected'
                    ),
                )
            else:
                self.send_navigation_result(
                    command,
                    'aborted',
                    detail,
                )

            self.clear_active_command(
                command_id
            )
            return

        with self.command_lock:
            self.active_goal_handle = goal_handle

        self.get_logger().info(
            f'Nav2 accepted command {command_id}'
        )
        self.send_command_ack(
            command,
            True,
            'Nav2 accepted the navigation goal',
        )
        self.send_command_status(
            command,
            'started',
            'Nav2 started the navigation goal',
        )

        result_future = (
            goal_handle.get_result_async()
        )
        result_future.add_done_callback(
            lambda result: (
                self.navigation_result_callback(
                    result,
                    command,
                )
            )
        )

        with self.command_lock:
            cancel_request = (
                self.pending_cancel_requests.get(
                    task_id
                )
            )

        if cancel_request is not None:
            self.request_goal_cancellation(
                command,
                goal_handle,
                cancel_request,
            )

    def navigation_feedback_callback(
        self,
        feedback_message: Any,
        command: dict[str, Any],
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if (
            now_ns - self.last_feedback_log_ns
            < 2_000_000_000
        ):
            return

        self.last_feedback_log_ns = now_ns

        feedback = feedback_message.feedback
        current_pose = feedback.current_pose
        pose = current_pose.pose
        orientation = pose.orientation

        sin_yaw = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        yaw = math.atan2(
            sin_yaw,
            cos_yaw,
        )

        navigation_duration = (
            feedback.navigation_time
        )
        navigation_time_seconds = (
            float(navigation_duration.sec)
            + (
                float(navigation_duration.nanosec)
                / 1_000_000_000.0
            )
        )

        estimated_duration = (
            feedback.estimated_time_remaining
        )
        estimated_time_remaining_seconds = (
            float(estimated_duration.sec)
            + (
                float(estimated_duration.nanosec)
                / 1_000_000_000.0
            )
        )

        distance_remaining = max(
            0.0,
            float(feedback.distance_remaining),
        )
        navigation_time_seconds = max(
            0.0,
            navigation_time_seconds,
        )
        estimated_time_remaining_seconds = max(
            0.0,
            estimated_time_remaining_seconds,
        )

        x = float(pose.position.x)
        y = float(pose.position.y)

        numeric_values = (
            distance_remaining,
            navigation_time_seconds,
            estimated_time_remaining_seconds,
            x,
            y,
            yaw,
        )

        if not all(
            math.isfinite(value)
            for value in numeric_values
        ):
            self.get_logger().warning(
                'Ignoring non-finite Nav2 feedback'
            )
            return

        with self.velocity_lock:
            velocity = (
                dict(self.latest_velocity)
                if self.latest_velocity is not None
                else None
            )

        self.get_logger().debug(
            'Nav2 feedback: '
            f'{distance_remaining:.2f} m remaining, '
            'ETA '
            f'{estimated_time_remaining_seconds:.1f} s'
        )

        self.send_from_ros(
            {
                'type': 'navigation_feedback',
                'command_id': command.get(
                    'command_id'
                ),
                'task_id': command.get(
                    'task_id'
                ),
                'stage': command.get(
                    'stage'
                ),
                'distance_remaining': (
                    distance_remaining
                ),
                'navigation_time_seconds': (
                    navigation_time_seconds
                ),
                (
                    'estimated_time_'
                    'remaining_seconds'
                ): (
                    estimated_time_remaining_seconds
                ),
                'number_of_recoveries': max(
                    0,
                    int(feedback.number_of_recoveries),
                ),
                'linear_velocity': (
                    velocity['linear_velocity']
                    if velocity is not None
                    else None
                ),
                'angular_velocity': (
                    velocity['angular_velocity']
                    if velocity is not None
                    else None
                ),
                'current_pose': {
                    'frame_id': (
                        current_pose.header.frame_id
                        or 'map'
                    ),
                    'x': x,
                    'y': y,
                    'yaw': yaw,
                },
                'timestamp': self.utc_timestamp(),
            }
        )

    def navigation_result_callback(
        self,
        future: Any,
        command: dict[str, Any],
    ) -> None:
        command_id = str(
            command['command_id']
        )
        task_id = str(
            command['task_id']
        )

        with self.command_lock:
            recovery_cancelled = command_id in self.recovery_cancelled_command_ids
            if recovery_cancelled:
                self.recovery_cancelled_command_ids.discard(command_id)
        if recovery_cancelled:
            self.get_logger().info(
                f'Suppressed navigation cancellation result during recovery: {command_id}'
            )
            self.clear_active_command(command_id)
            return

        try:
            wrapped_result = future.result()
            status = wrapped_result.status
            result = wrapped_result.result
        except Exception as error:
            detail = (
                'Failed to receive Nav2 result: '
                f'{error}'
            )
            self.get_logger().error(detail)

            with self.command_lock:
                cancel_request = (
                    self.pending_cancel_requests.pop(
                        task_id,
                        None,
                    )
                )

            if cancel_request is not None:
                self.send_navigation_cancelled(
                    cancel_request,
                    False,
                    detail,
                )
            else:
                self.send_navigation_result(
                    command,
                    'aborted',
                    detail,
                )

            self.clear_active_command(
                command_id
            )
            return

        with self.command_lock:
            cancel_request = (
                self.pending_cancel_requests.pop(
                    task_id,
                    None,
                )
            )

        if cancel_request is not None:
            if (
                status
                == GoalStatus.STATUS_CANCELED
            ):
                detail = (
                    'Nav2 goal cancellation completed'
                )
            elif (
                status
                == GoalStatus.STATUS_SUCCEEDED
            ):
                detail = (
                    'Nav2 goal finished before the '
                    'cancellation took effect'
                )
            else:
                error_message = getattr(
                    result,
                    'error_msg',
                    '',
                )
                detail = (
                    error_message
                    or (
                        'Nav2 goal stopped before '
                        'cancellation completed'
                    )
                )

            self.get_logger().info(
                'Navigation stopped for cancelled '
                f'task {task_id}: {detail}'
            )
            self.send_navigation_cancelled(
                cancel_request,
                True,
                detail,
            )
            self.clear_active_command(
                command_id
            )
            return

        if (
            status
            == GoalStatus.STATUS_SUCCEEDED
        ):
            navigation_status = 'succeeded'
            detail = 'Nav2 goal succeeded'
            self.get_logger().info(
                'Navigation succeeded for '
                f'{command_id}'
            )

        elif (
            status
            == GoalStatus.STATUS_CANCELED
        ):
            navigation_status = 'canceled'
            detail = 'Nav2 goal was canceled'
            self.get_logger().warning(
                'Navigation canceled for '
                f'{command_id}'
            )

        else:
            navigation_status = 'aborted'
            error_message = getattr(
                result,
                'error_msg',
                '',
            )
            detail = (
                error_message
                or 'Nav2 goal was aborted'
            )
            self.get_logger().error(
                'Navigation aborted for '
                f'{command_id}: {detail}'
            )

        if navigation_status == 'aborted':
            accepted_near_goal, near_goal_detail = (
                self.accept_aborted_goal_when_near(command)
            )
            if accepted_near_goal:
                navigation_status = 'succeeded'
                detail = near_goal_detail
                self.get_logger().warning(
                    f'Navigation accepted near goal for {command_id}: {detail}'
                )
            elif self.maybe_start_blocked_pose_recovery(command, detail):
                return

        self.send_navigation_result(
            command,
            navigation_status,
            detail,
        )
        self.clear_active_command(
            command_id
        )

    def send_command_ack(
        self,
        command: dict[str, Any],
        accepted: bool,
        detail: str,
    ) -> None:
        self.send_from_ros(
            {
                'type': 'command_ack',
                'command_id': command.get('command_id'),
                'accepted': accepted,
                'detail': detail,
            }
        )

    def send_command_status(
        self,
        command: dict[str, Any],
        lifecycle: str,
        detail: str,
    ) -> None:
        self.send_from_ros(
            self.command_status_payload(
                command,
                lifecycle,
                detail,
            )
        )

    def send_navigation_result(
        self,
        command: dict[str, Any],
        status: str,
        detail: str,
    ) -> None:
        self.send_command_status(
            command,
            'succeeded' if status == 'succeeded' else 'failed',
            detail,
        )
        self.send_from_ros(
            {
                'type': 'navigation_result',
                'command_id': command.get('command_id'),
                'task_id': command.get('task_id'),
                'stage': command.get('stage'),
                'status': status,
                'detail': detail,
            }
        )

    def send_navigation_cancelled(
        self,
        cancel_request: dict[str, Any],
        cancelled: bool,
        detail: str,
    ) -> None:
        self.send_from_ros(
            {
                'type': 'navigation_cancelled',
                'cancel_id': (
                    cancel_request.get(
                        'cancel_id'
                    )
                ),
                'task_id': (
                    cancel_request.get(
                        'task_id'
                    )
                ),
                'cancelled': cancelled,
                'detail': detail,
            }
        )

    def clear_active_command(self, command_id: str) -> None:
        cleared_command = None
        with self.command_lock:
            if (
                self.active_command is not None
                and self.active_command.get('command_id')
                == command_id
            ):
                cleared_command = dict(self.active_command)
                self.active_command = None
                self.active_goal_handle = None

        if cleared_command is not None:
            self.clear_navigation_path(
                cleared_command,
                send_clear=False,
                force=True,
            )

    def clear_navigation_path(
        self,
        command: dict[str, Any] | None = None,
        *,
        send_clear: bool = True,
        force: bool = False,
    ) -> None:
        if command is None:
            with self.command_lock:
                command = (
                    dict(self.active_command)
                    if self.active_command is not None
                    else None
                )

        with self.path_lock:
            had_path = self.latest_path is not None
            self.latest_path = None
            self.latest_path_signature = None
            self.path_revision += 1

        if (
            send_clear
            and command is not None
            and (had_path or force)
        ):
            self.send_from_ros(
                {
                    'type': 'navigation_path_clear',
                    'command_id': command.get('command_id'),
                    'task_id': command.get('task_id'),
                    'stage': command.get('stage'),
                }
            )

    def destroy_node(self) -> bool:
        self.stop_requested.set()
        self.emergency_stop_latched.clear()
        self.physical_estop_latched.clear()
        self.mapping_runtime.shutdown()
        self.destroy_timer(self.preview_timer)
        self.destroy_timer(self.emergency_zero_timer)
        self.destroy_timer(self.physical_estop_watchdog_timer)
        self.destroy_timer(self.mapping_deadman_timer)
        self.destroy_timer(self.mapping_pose_timer)
        self.destroy_publisher(
            self.emergency_velocity_publisher
        )

        if self.asyncio_loop is not None and self.websocket is not None:
            try:
                close_future = asyncio.run_coroutine_threadsafe(
                    self.websocket.close(),
                    self.asyncio_loop,
                )
                close_future.result(timeout=2.0)
            except Exception:
                pass

        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=3.0)

        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
