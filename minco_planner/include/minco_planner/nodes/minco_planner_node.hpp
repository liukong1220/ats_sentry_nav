// Copyright 2026

#ifndef MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "minco_planner/debug/planner_debug_visualizer.hpp"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/planning/grid_search.hpp"
#include "minco_planner/planning/local_route.hpp"
#include "minco_planner/safety/escape_prefix.hpp"
#include "minco_planner/safety/goal_pose_admission.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
#include "minco_planner/trajectory/replan_start.hpp"
#include "minco_planner/trajectory/spin_yaw.hpp"
#include "minco_planner/trajectory/terminal_yaw_relocation.hpp"
#include "minco_planner/trajectory/trajectory_quality_evaluator.hpp"
#include "minco_planner/trajectory/wheel_speed_time_scaling.hpp"
#include "minco_planner/trajectory/yaw_authority_policy.hpp"
#include "minco_planner/trajectory/yaw_spline_planner.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace minco_planner
{

struct PlannerCommitTelemetry
{
  std::string grid_topic;
  std::string validation_frame;
  std::uint32_t gate_collision_count{0};
  std::string gate_collision_indices;
  std::string occupancy_digest;
  std::string content_digest;
  std::uint32_t reference_point_count{0};
  std::vector<double> reference_vx;
  std::vector<double> reference_vy;
  std::vector<double> reference_ax;
  std::vector<double> reference_ay;
  std::vector<double> reference_yaw_rate;
  bool is_partial_subgoal{false};
  bool has_spliced_tail{false};
  float search_wall_ms{0.0F};
  float minco_wall_ms{0.0F};
  float total_plan_wall_ms{0.0F};
};

class MincoPlannerNode : public rclcpp::Node
{
public:
  explicit MincoPlannerNode(const rclcpp::NodeOptions & options);

private:
  using GoalUuid = std::array<std::uint8_t, 16>;

  /// Run the graph search across the graduated clearance ladder.
  ///
  /// Returns the first successful attempt and reports the clearance it used;
  /// on total failure it returns the last attempt so the caller can classify it.
  /// \param goal_pose_footprint_verified True only when the goal pose已经通过同一套
  ///   矩形足迹门禁。为真时才允许最后一档把"目标格净空下限"降到栅格量化余量。
  GridSearchResult runGraphSearch(
    const nav_msgs::msg::OccupancyGrid & planning_grid,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    bool goal_pose_footprint_verified,
    double & used_clearance,
    const std::vector<double> * blocked_squared = nullptr,
    bool allow_partial_subgoal = false);
  GridSearchResult planRoute(
    const std::shared_ptr<const PlanningMapSnapshot> & snapshot,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const GoalUuid & goal_uuid,
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    bool goal_pose_footprint_verified, double & used_clearance,
    nav_msgs::msg::Path * global_guide_out = nullptr,
    std::optional<geometry_msgs::msg::PoseStamped> * local_anchor_out = nullptr);
  void publishRollingAnchor(
    const std::optional<geometry_msgs::msg::PoseStamped> & anchor,
    const std::string & frame_id, const rclcpp::Time & stamp, const char * label);
  // Offline/sandbox input: builds the local snapshot (and its distance fields) from a grid.
  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  // Production input: the map owner's numeric snapshot; no distance transform runs here.
  void onPlanningSnapshot(
    const ats_navigation_interfaces::msg::PlanningMapSnapshot::SharedPtr msg);
  using SnapshotBuilder =
    std::function<std::shared_ptr<const PlanningMapSnapshot>(std::uint64_t)>;
  // Shared commit path for both inputs. `build` receives the candidate local generation.
  void ingestMapSnapshot(
    const SnapshotBuilder & build, const builtin_interfaces::msg::Time & source_stamp,
    std::int64_t received_steady_ns);
  void onMapReady(const std_msgs::msg::Bool::SharedPtr msg);
  void onMapReadyWatchdog();
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPlannerGoal(const ats_navigation_interfaces::msg::PlannerGoal::SharedPtr msg);
  void planGoal(const geometry_msgs::msg::PoseStamped &goal,
                std::uint64_t goal_id, std::uint64_t localization_epoch,
                std::uint64_t plan_request_sequence,
                std::uint64_t map_publication_sequence,
                bool report_status,
                const geometry_msgs::msg::PoseStamped * frozen_start = nullptr,
                const InitialKinematicState * initial_state = nullptr,
                double position_tolerance = 0.0, const GoalUuid & goal_uuid = {},
                bool preserve_active_reference = false);
  struct PlanGoalCommitContext;
  void applyTerminalYawRelocationAndSidestep(
    const std::shared_ptr<const PlanningMapSnapshot> & map_snapshot,
    const nav_msgs::msg::OccupancyGrid & planning_grid,
    const std::shared_ptr<const ats_rc_esdf::RcTraversabilityEsdfProvider> & clearance_esdf,
    double active_goal_yaw, bool active_is_final_goal,
    ReferenceTrajectory & reference, FootprintSafetyResult & safety);
  void tryEscapePrefixAndLocalReverse(
    const std::shared_ptr<const PlanningMapSnapshot> & map_snapshot,
    const nav_msgs::msg::OccupancyGrid & planning_grid,
    const std::shared_ptr<const ats_rc_esdf::RcTraversabilityEsdfProvider> & clearance_esdf,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double start_yaw, double goal_yaw,
    double & active_goal_yaw, bool & active_is_final_goal,
    const std::optional<double> & exploration_lidar_fov,
    const InitialKinematicState * seed_ptr, double used_clearance,
    bool goal_pose_footprint_verified,
    const std::function<bool()> & rolling_budget_exceeded,
    ReferenceTrajectory & reference, FootprintSafetyResult & safety,
    MincoOptimizationTrace & selected_trace, bool & has_spliced_tail,
    EscapePrefixDecision & escape_decision);
  void finalizeAndCommitReference(
    PlanGoalCommitContext & ctx, const EscapePrefixDecision & escape_decision);
  bool lookupStartPose(
    const nav_msgs::msg::OccupancyGrid & grid, geometry_msgs::msg::PoseStamped & start);
  bool resolveStartPose(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped * frozen_start,
    geometry_msgs::msg::PoseStamped & start);
  bool transformGoalToGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::PoseStamped & input, geometry_msgs::msg::PoseStamped & output);
  bool transformPathToGlobal(
    const nav_msgs::msg::Path & input, nav_msgs::msg::Path & output,
    geometry_msgs::msg::TransformStamped * used_transform = nullptr);
  nav_msgs::msg::Path toPath(const ReferenceTrajectory & trajectory) const;
  void annotatePositionClearance(
    ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const;
  void planYaw(
    ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot,
    double start_yaw, double goal_yaw, bool is_final_goal = true,
    std::optional<double> preferred_lidar_fov_yaw_offset = std::nullopt,
    bool force_goal_yaw = false) const;
  void annotateClearance(
    ReferenceTrajectory & trajectory, const PlanningMapSnapshot & snapshot) const;
  std::vector<Eigen::Vector2d> footprintSamples(
    const nav_msgs::msg::OccupancyGrid & grid) const;
  void setPlanSafe(bool safe);
  bool publishReferenceIfCurrent(
      const std::shared_ptr<const PlanningMapSnapshot> &snapshot,
      std::uint64_t map_health_epoch, const nav_msgs::msg::Path &reference_path,
      const ReferenceTrajectory &safety_reference,
      std::uint64_t goal_id, std::uint64_t localization_epoch,
      std::uint64_t plan_request_sequence,
      std::uint64_t map_publication_sequence,
      std::uint8_t yaw_authority,
      bool report_status,
      const PlannerCommitTelemetry & telemetry = PlannerCommitTelemetry(),
      const geometry_msgs::msg::TransformStamped * control_from_grid = nullptr,
      double reference_origin_delay_sec = 0.0,
      bool preserve_active_reference = false,
      bool scan_turn = false);
  void onRuntimeSafetyRecheck();
  bool remainingReferenceSafeOn(
    const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid,
    std::string * reason = nullptr, geometry_msgs::msg::PoseStamped * progress_pose = nullptr,
    std::optional<geometry_msgs::msg::Point> * conflict = nullptr,
    std::size_t * collision_count = nullptr);
  // 车当前推进位置起 runtime_safety_horizon_sec 内的参考在 grid 上是否安全（与运行期复检同判据）。
  bool runtimeWindowSafeOn(
    const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid);
  // 保留检查与运行期复检的窗口锚点：车沿参考的推进（参考时间，秒）。
  double referenceProgress(
    const ReferenceTrajectory & trajectory, const nav_msgs::msg::OccupancyGrid & grid,
    double elapsed);
  void publishEmergencyStop(bool stop);
  void
  publishPlannerStatus(std::uint64_t goal_id, std::uint64_t localization_epoch,
                       std::uint64_t plan_request_sequence,
                       std::uint64_t map_generation,
                       std::uint64_t map_publication_sequence,
                       std::uint8_t state, std::uint8_t failure_reason,
                       const builtin_interfaces::msg::Time &reference_stamp =
                           builtin_interfaces::msg::Time(),
                       std::uint8_t yaw_authority = 0,
                       bool requires_gimbal_lock = false,
                       const PlannerCommitTelemetry & telemetry = PlannerCommitTelemetry());
  // Both helpers require map_mutex_; no nested lock acquisition.
  void publishPlannerStatusLocked(
    std::uint64_t goal_id, std::uint64_t localization_epoch,
    std::uint64_t plan_request_sequence, std::uint64_t map_generation,
    std::uint64_t map_publication_sequence, std::uint8_t state,
    std::uint8_t failure_reason,
    const builtin_interfaces::msg::Time & reference_stamp = builtin_interfaces::msg::Time(),
    std::uint8_t yaw_authority = 0, bool requires_gimbal_lock = false,
    const PlannerCommitTelemetry & telemetry = PlannerCommitTelemetry());
  void invalidatePlannerStatusLocked(std::uint8_t failure_reason, std::uint64_t generation);
  void invalidateMapLocked();
  void declareAndLoadParams();

  std::string grid_topic_ = "traversability_grid";
  // Non-empty: subscribe to the numeric planning snapshot and ignore grid_topic.
  std::string planning_snapshot_topic_ = "/rog_map_adapter/planning_snapshot";
  std::string prior_guide_map_topic_;
  std::string goal_topic_ = "goal_pose";
  std::string goal_request_topic_;
  std::string planner_status_topic_;
  std::string raw_path_topic_ = "minco/raw_path";
  std::string reference_path_topic_ = "minco/reference_path";
  std::string candidate_reference_path_topic_;
  std::string preprocessed_guide_topic_ = "/minco/preprocessed_guide";
  std::string esdf_refined_guide_topic_ = "/minco/esdf_refined_guide";
  std::string rolling_anchor_topic_ = "/minco/rolling_anchor";
  std::string debug_marker_topic_ = "minco/debug_markers";
  std::string map_ready_topic_;
  std::string emergency_stop_topic_ = "/planner/emergency_stop";
  std::string global_frame_ = "map";
  std::string robot_frame_ = "base_link";
  std::string lidar_frame_ = "front_mid360";
  std::string search_algorithm_ = "hybrid";
  bool publish_unsafe_trajectory_ = false;
  EscapePrefixParams escape_prefix_params_{};
  double inscribed_footprint_radius_{0.0};
  bool search_clearance_floor_configured_{false};
  bool planner_manages_emergency_stop_ = true;
  int obstacle_value_threshold_ = 50;
  bool unknown_is_obstacle_ = true;
  double footprint_length_ = 0.58;
  double footprint_width_ = 0.58;
  double footprint_safety_margin_ = 0.05;
  double map_ready_timeout_sec_ = 3.0;
  double tf_lookup_timeout_sec_ = 0.1;
  double runtime_tf_lookup_timeout_sec_ = 0.05;
  // 运行日志默认每 2 s 最多输出一次；结构化关键字仍保留，便于回归脚本解析。
  std::uint64_t log_throttle_ms_{2000};
  bool map_commit_trace_enabled_{false};
  double emergency_stop_heartbeat_period_sec_ = 0.1;
  double runtime_safety_recheck_hz_ = 10.0;
  double runtime_safety_horizon_sec_ = 1.0;
  bool retain_safe_reference_on_snapshot_change_ = false;
  // 保留判定的前向时间窗(s)。<=0 表示检查完整剩余段(原行为)。远端冲突留给
  // 10 Hz 运行期复检在逼近时作废,避免远处地图抖动让车每 2 s 急停重规划。
  double retain_reference_horizon_sec_ = 0.0;
  // 保留/复检窗口以车沿参考的实际推进为锚点（而非墙钟），默认关闭保持原行为。
  bool reference_window_from_robot_progress_{false};
  // 保留检查失败但运行期窗口仍安全时，继续执行并发 FAILURE_REPLAN_ADVISED 请 GoalManager
  // 无停车重规划；默认关闭保持原行为（作废并急停）。
  bool replan_advisory_enabled_{false};
  // 已发过建议的参考（以参考戳标识），每条参考只建议一次；对应的重规划请求保留旧参考守护。
  std::optional<builtin_interfaces::msg::Time> replan_advised_reference_stamp_;
  // 远端 1-3 格弱冲突或仅全局引导冲突需连续确认的独立快照数（近端安全窗口仍零延迟急停）。
  int far_conflict_confirm_snapshots_{2};
  int far_conflict_streak_{0};
  std::optional<builtin_interfaces::msg::Time> far_conflict_reference_stamp_;
  // 无停车重规划的新参考起点取自正在执行的已提交轨迹（调研报告附录 A.3），而不是请求时
  // 的实测位姿：规划耗时内车已前行，实测起点落在车后面。plan_lead / deviation_max 见参数。
  ReplanStartParams replan_start_params_{};
  double reference_window_backward_sec_{0.0};
  double body_yaw_follow_clearance_ = 0.55;
  bool force_body_yaw_follow_ = false;
  bool tunnel_lock_gimbal_ = true;
  SpinPolicy spin_policy_;
  SpinYawParams spin_yaw_params_;
  /// Preferred graph-search clearance (circumscribed all-yaw footprint radius).
  double preferred_search_clearance_ = 0.0;
  /// Lowest clearance the graduated ladder may fall back to (inscribed half-width).
  double search_clearance_floor_ = 0.0;
  bool clearance_relaxation_enabled_ = true;
  double clearance_detour_ratio_ = 0.0;
  /// 终点净空放宽档。目标格的净空是各向同性代理量,而目标是"一个已知 yaw 的位姿",
  /// 后者有精确的矩形判定。目标位姿已过足迹门禁时,再用 inscribed+量化余量的代理量
  /// 否决它就是用更粗的判据推翻更细的判据(domain 187 目标 9:目标格净空约 0.26 m,
  /// 下限 0.341 m,图搜索直接报 goal occupied,规划器一条路径都产不出来)。
  bool endpoint_clearance_relaxation_enabled_ = true;
  /// 终端原地转向重定位。窄通道 yaw 在目标处追加的原地转向没有 footprint 感知,
  /// 目标净空介于内切半宽与全 yaw 外接圆之间时必然扫过不可行 yaw 带,门禁正确
  /// 拒绝后规划器会每周期复现同一条被拒轨迹直到超时。开关关闭即完全保持旧行为。
  bool terminal_yaw_relocation_enabled_ = true;
  // 足迹优化 yaw 不动点补解轮数（0 = 关闭）。
  int footprint_yaw_refinement_rounds_ = 2;
  /// 逐个试的转向位置个数上界,只在轨迹已被判不安全时才会消耗。
  int terminal_yaw_relocation_max_candidates_ = 6;
  TerminalYawRelocationParams terminal_yaw_relocation_params_{};
  double terminal_rotation_comfortable_clearance_m_ = 0.06;
  double terminal_rotation_tight_yaw_span_rad_ = 0.35;
  double sidestep_min_esdf_margin_m_ = 0.04;
  double sidestep_desired_esdf_margin_m_ = 0.10;
  double sidestep_max_radius_m_ = 0.45;
  int sidestep_max_tries_ = 8;
  double sidestep_accept_clearance_m_ = 0.08;
  double sidestep_min_improvement_m_ = 0.03;
  // O2 局部滚动窗口与拼接参数。
  bool rolling_horizon_enabled_ = true;
  double rolling_horizon_length_m_ = 8.0;
  double rolling_min_tail_length_m_ = 2.0;
  // O1/A3 无先验远目标局部前沿接续参数。
  bool partial_subgoal_continuation_enabled_ = true;
  bool partial_subgoal_only_without_prior_ = true;
  double partial_subgoal_min_advance_m_ = 0.30;
  // 中心 ESDF 候选已过矩形足迹门且最小足迹净空 >= 该阈值时跳过冗余二次足迹联合优化。
  double footprint_refinement_skip_clearance_m_ = 0.04;
  // O2 Release 墙钟预算与统计参数。
  double minco_stage_budget_ms_ = 20.0;
  double plan_wall_budget_ms_ = 30.0;
  double rolling_replan_wall_budget_ms_ = 35.0;
  double plan_hard_wall_budget_ms_ = 0.0;
  double optimizer_params_cached_speed_ = 1.5;
  double optimizer_params_cached_max_accel_ = 2.5;
  double optimizer_params_cached_lat_accel_ = 1.5;
  struct PlanWallStats
  {
    std::mutex mutex;
    std::uint64_t calls{0};
    std::uint64_t minco_over_budget{0};
    std::uint64_t plan_over_budget{0};
    std::vector<double> minco_samples_ms;
    std::vector<double> total_samples_ms;
    std::size_t ring_cursor{0};
  };
  PlanWallStats plan_wall_stats_;
  LocalRouteParams local_route_params_{};

  GridSearch graph_search_;
  // Only the planning callback group changes the prior layer and route cache.
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr prior_guide_map_;
  std::vector<double> prior_blocked_squared_;
  struct GlobalRoute
  {
    GoalUuid goal_uuid;
    std::uint64_t goal_id;
    std::uint64_t localization_epoch;
    std::shared_ptr<const PlanningMapSnapshot> snapshot;
    geometry_msgs::msg::Pose goal;
    nav_msgs::msg::Path::ConstSharedPtr path;
    double clearance;
    bool optimistic{false};
  };
  std::optional<GlobalRoute> global_route_;
  struct FrontierTabooEntry
  {
    double x{0.0};
    double y{0.0};
    std::chrono::steady_clock::time_point added_at{};
  };
  std::uint64_t frontier_taboo_goal_id_{0};
  std::uint64_t frontier_taboo_epoch_{0};
  std::vector<FrontierTabooEntry> frontier_taboo_;
  std::optional<geometry_msgs::msg::Point> last_partial_anchor_;
  std::optional<geometry_msgs::msg::Point> last_partial_start_;
  std::optional<geometry_msgs::msg::Point> last_unknown_frontier_;
  std::optional<geometry_msgs::msg::Point> mapless_stall_origin_;
  std::optional<geometry_msgs::msg::Point> mapless_long_stall_origin_;
  double mapless_stall_yaw_{0.0};
  std::chrono::steady_clock::time_point last_partial_anchor_time_{};
  std::chrono::steady_clock::time_point mapless_stall_time_{};
  std::chrono::steady_clock::time_point mapless_long_stall_time_{};
  int mapless_consecutive_rejects_{0};
  double optimistic_global_unknown_cost_{1.35};
  double frontier_taboo_radius_m_{0.70};
  double frontier_taboo_ttl_sec_{180.0};
  std::optional<double> resolveLidarFovYawOffsetRad();
  MincoTrajectoryOptimizer optimizer_;
  YawSplinePlanner yaw_planner_;
  /// 同一套航向参数，只多一项 narrow_start_hold_length；planYaw 在两版之间按足迹冲突选。
  YawSplinePlanner narrow_start_hold_yaw_planner_;
  double narrow_start_hold_length_ = 0.0;
  TunnelYawParams tunnel_yaw_params_{};
  WheelSpeedTimeScalingParams wheel_speed_time_scaling_params_{};
  /// 提交门使用的矩形足迹参数副本。目标位姿准入必须用同一份参数，否则"终点可行"
  /// 与"轨迹可行"会用两套几何判定。
  FootprintSafetyParams footprint_params_{};
  /// 目标位姿准入：目标点足迹不可行时，在成功容差域内挑一个可行终点。
  GoalPoseAdmissionParams goal_pose_admission_params_{};
  /// Nominal commit geometry gate (disabled when both limits <= 0).
  CommitGeometryLimits commit_geometry_limits_{};
  FootprintSafetyChecker safety_checker_;
  LocalCollisionRepair collision_repair_;
  PlannerDebugVisualizer visualizer_;
  TrajectoryQualityEvaluator quality_evaluator_;

  std::mutex map_mutex_;
  std::shared_ptr<const PlanningMapSnapshot> latest_map_snapshot_;
  std::uint64_t next_map_generation_{0};
  std::uint64_t map_health_epoch_{0};
  PlannerSafetyState safety_state_;
  PlannerStatusState planner_status_state_;
  std::optional<std::chrono::steady_clock::time_point> last_map_ready_signal_;
  struct ActiveSafetyReference
  {
    ReferenceTrajectory trajectory;
    std::uint64_t goal_id{0};
    std::uint64_t localization_epoch{0};
    std::uint64_t plan_request_sequence{0};
    std::uint64_t map_generation{0};
    std::uint64_t map_publication_sequence{0};
    // 提交时 控制坐标系(global_frame_) <- 栅格坐标系 的变换。MPC 在控制坐标系里执行，
    // map->odom 修正后车实际走的是 当前(栅格<-控制) * 提交时(控制<-栅格) * trajectory。
    std::optional<geometry_msgs::msg::TransformStamped> control_from_grid;
    // Exact SE3 path sent to the planar controller. Its Z plane is needed when
    // expressing execution in a grid whose TF contains roll/pitch.
    std::optional<nav_msgs::msg::Path> control_reference;
    nav_msgs::msg::Path::ConstSharedPtr guide;
    std::shared_ptr<const PlanningMapSnapshot> guide_snapshot;
    bool guide_optimistic{false};
    bool scan_turn{false};
  };
  std::optional<ActiveSafetyReference> active_safety_reference_;
  struct ReferenceConflict
  {
    std::uint64_t goal_id;
    std::uint64_t localization_epoch;
    std::uint64_t snapshot_generation;
    geometry_msgs::msg::Point position;
  };
  std::optional<ReferenceConflict> reference_conflict_;
  // 按当前 TF 重表达已提交控制参考；跨系 TF 不可用时返回无效参考。
  ReferenceTrajectory alignedToCurrentTf(const ActiveSafetyReference & reference);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr prior_guide_map_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlanningMapSnapshot>::SharedPtr
    planning_snapshot_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::PlannerGoal>::SharedPtr goal_request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr map_ready_sub_;
  rclcpp::TimerBase::SharedPtr safety_watchdog_timer_;
  rclcpp::TimerBase::SharedPtr runtime_safety_recheck_timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr candidate_reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr preprocessed_guide_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr esdf_refined_guide_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rolling_anchor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::PlannerStatus>::SharedPtr planner_status_pub_;

  rclcpp::CallbackGroup::SharedPtr planning_callback_group_;
  rclcpp::CallbackGroup::SharedPtr map_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_NODE_HPP_
