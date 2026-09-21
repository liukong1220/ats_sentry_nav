// Copyright 2026 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
#define SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_

#include <Eigen/Geometry>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/relocalization_observation.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "small_gicp_relocalization/relocalization_candidate_core.hpp"
#include "small_gicp_relocalization/scan_input_core.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace small_gicp_relocalization
{

class SmallGicpRelocalizationNode : public rclcpp::Node
{
public:
  explicit SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options);
  ~SmallGicpRelocalizationNode() override;

private:
  friend class SmallGicpRelocalizationFrameTest;

  using PointCovarianceCloud = pcl::PointCloud<pcl::PointCovariance>;
  using PointKdTree = small_gicp::KdTree<PointCovarianceCloud>;
  using GicpRegistration =
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>;

  /// 门禁语义分层。coarse 是被迭代上限截断的筛选级，其 converged 标志不携带信息，
  /// 只有 fine 级有权认定收敛。kScreen 仅豁免 converged，其余硬门保持不变。
  enum class GateStage { kScreen, kAccept };

  struct RegistrationAttempt
  {
    bool ok{false};
    bool converged{false};
    std::size_t iterations{0};
    std::size_t num_inliers{0};
    double registration_error{std::numeric_limits<double>::infinity()};
    double overlap_ratio{0.0};
    double min_information_eigenvalue{0.0};
    double information_condition_number{std::numeric_limits<double>::infinity()};
    Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
    Eigen::Matrix<double, 6, 6> information{Eigen::Matrix<double, 6, 6>::Zero()};
    std::string stage{"none"};
    std::string reject_reason;
    // 组合评分与歧义证据。单一 residual 不再是选择依据。
    double score{std::numeric_limits<double>::infinity()};
    double alternative_score{std::numeric_limits<double>::infinity()};
    double score_margin{std::numeric_limits<double>::infinity()};
    bool ambiguous{false};
    double motion_residual{0.0};
    double prior_deviation{0.0};
  };

  /// 异步 multi_guess 的不可变输入快照。worker 线程只读这些字段，
  /// 不触碰节点状态，避免与 /initialpose、status 回调竞争。
  struct MultiGuessRequest
  {
    std::uint64_t generation{0};
    PointCovarianceCloud::Ptr source;
    std::shared_ptr<PointKdTree> source_tree;
    /// coarse 筛选专用的降采样源云。为空时退回 source。
    PointCovarianceCloud::Ptr screen_source;
    std::shared_ptr<PointKdTree> screen_source_tree;
    Eigen::Isometry3d seed{Eigen::Isometry3d::Identity()};
    /// 本次扫描是为复核待确认假设而运行，cursor 不代表探索进度。
    bool confirmation_recheck{false};
    std::size_t cursor{0};
    std::uint64_t sweep{0};
    bool allow_unconverged{false};
    /// 上一帧通过硬门的假设，供 motion 一致性软约束使用；无参考时为空。
    std::optional<ConfirmationSample> reference;
    std::optional<Eigen::Isometry3d> current_odom_to_base;
    std::size_t source_points{0};
  };

  struct MultiGuessOutcome
  {
    std::uint64_t generation{0};
    RegistrationAttempt best;
    CandidateLatticeStats lattice;
    std::size_t evaluated{0};
    std::size_t gated{0};
    /// 实际做过 fine 精配准的候选数。screen 数远大于它是预期行为。
    std::size_t refined{0};
    std::size_t skipped{0};
    std::size_t cursor{0};
    bool confirmation_recheck{false};
    std::size_t next_cursor{0};
    bool wrapped{false};
    bool budget_exhausted{false};
    double elapsed_s{0.0};
    CandidateSelection selection;
  };

  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  bool loadGlobalMap(const std::string & file_name);
  void performRegistration();
  void publishTransform();
  void publishObservation(
    const rclcpp::Time & scan_time, bool accepted, std::uint8_t status,
    const std::string & message, std::size_t inliers,
    double error, std::size_t source_points, const Eigen::Isometry3d & map_to_robot_base,
    const std::array<double, 36> & covariance);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void localizationStatusCallback(
    const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr msg);
  bool shouldRunRegistration();
  bool isRecoveringLocalization() const;
  bool preferMultiGuess() const;
  bool inInitialPoseForceWindow() const;
  bool simRelaxAllowed() const;
  void clearAccumulation();
  void preprocessAccumulatedSource();
  void recordDroppedScan(const std::string & reason);
  double accumulatedCloudAgeSeconds() const;
  std::optional<Eigen::Isometry3d> getCurrentRobotBaseToOdom() const;
  std::optional<Eigen::Isometry3d> getOdomToRobotBase(const rclcpp::Time & stamp) const;
  ConfirmationDecision evaluateCandidateConfirmation(
    const Eigen::Isometry3d & candidate, const rclcpp::Time & scan_time,
    const Eigen::Isometry3d & odom_to_base) const;
  void clearConfirmation();
  void invalidateRecovery();
  bool expireConfirmation();
  double translationDeltaFromLastTrigger(
    const Eigen::Isometry3d & current_robot_base_to_odom) const;
  double yawDeltaFromLastTrigger(const Eigen::Isometry3d & current_robot_base_to_odom) const;

  RegistrationAttempt alignOnce(
    const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations) const;
  RegistrationAttempt alignOnceOn(
    const Eigen::Isometry3d & initial_guess, float max_dist_sq, int max_iterations,
    const PointCovarianceCloud::Ptr & source, const std::shared_ptr<PointKdTree> & source_tree,
    GicpRegistration & registration) const;
  RegistrationAttempt runCoarseFineAlignment(
    const Eigen::Isometry3d & initial_guess, bool allow_unconverged);
  MultiGuessOutcome runMultiGuessAlignmentOn(
    const MultiGuessRequest & request, const std::atomic<bool> & cancel_flag) const;
  CandidateHardGates hardGates(bool allow_unconverged) const;
  bool passesQualityGates(
    RegistrationAttempt & attempt, bool allow_unconverged, GateStage stage) const;
  CandidateEvidence attemptEvidence(
    const RegistrationAttempt & attempt, const Eigen::Isometry3d & seed,
    const std::optional<ConfirmationSample> & reference,
    const std::optional<Eigen::Isometry3d> & current_odom_to_base) const;
  CandidateLatticeConfig latticeConfig() const;
  void writeCandidateDiagnostics(const std::string & rows) const;
  void startAsyncMultiGuess();
  void cancelAsyncMultiGuess();
  void drainAsyncMultiGuessResult();
  void handleRegistrationAttempt(
    RegistrationAttempt attempt, const rclcpp::Time & scan_time, std::size_t source_points);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_status_sub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::RelocalizationObservation>::SharedPtr
    observation_pub_;

  int num_threads_;
  int num_neighbors_;
  int min_source_points_;
  int min_inliers_;
  float global_leaf_size_;
  float registered_leaf_size_;
  /// coarse 候选筛选的额外降采样尺度。<=registered_leaf_size_ 时不建立独立筛选云。
  float multi_guess_screen_leaf_size_{0.0f};
  float max_dist_sq_;
  double max_registration_error_;
  bool relax_convergence_for_sim_{false};
  bool log_registration_details_;
  bool publish_tf_;
  int confirmation_count_;
  double confirmation_translation_tolerance_;
  double confirmation_yaw_tolerance_;
  double registration_interval_s_;
  double max_accumulation_age_s_;
  double min_registration_translation_delta_;
  double min_registration_yaw_delta_;
  double accepted_observation_refresh_interval_s_;
  double initial_pose_force_registration_window_s_;
  double transform_future_offset_s_;
  double max_scan_stamp_lag_s_;
  double max_scan_age_s_{1.0};
  double max_scan_future_s_{0.10};
  double scan_min_range_m_{0.10};
  double scan_max_range_m_{100.0};
  double scan_min_z_m_{-5.0};
  double scan_max_z_m_{5.0};
  double min_scan_valid_ratio_{0.50};
  std::size_t max_accumulated_points_{40000};
  int max_accumulated_frames_{30};
  std::vector<double> init_pose_;

  // Coarse-to-fine / windowed alignment (BIT icp_relocalization + HWSentry quality gates).
  std::string registration_mode_{"initial_guess"};
  int accumulate_frames_{1};
  bool fine_alignment_enabled_{true};
  bool coarse_first_window_only_{true};
  float fine_max_dist_sq_{0.2025f};  // 0.45 m
  int coarse_max_iterations_{10};
  int fine_max_iterations_{16};
  double min_overlap_ratio_{0.0};
  double min_information_eigenvalue_{0.0};
  double max_information_condition_number_{0.0};
  double multi_guess_search_half_xy_{2.0};
  double multi_guess_step_xy_{1.0};
  double multi_guess_step_yaw_{0.785398};
  std::vector<double> multi_guess_z_candidates_{0.0};
  // 候选调度预算：每次扫描只评估一段，剩余候选由 cursor 延续到下一次扫描。
  double multi_guess_time_budget_s_{1.5};
  int multi_guess_max_candidates_per_scan_{48};
  /// 每次扫描允许的 fine 精配准次数上限。筛选只排序，精配准才验收。
  int multi_guess_max_fine_per_scan_{4};
  bool multi_guess_log_candidates_{false};
  std::string multi_guess_candidate_log_path_;
  std::size_t multi_guess_cursor_{0};
  std::uint64_t multi_guess_sweep_{0};
  bool need_coarse_alignment_{true};
  bool has_accepted_alignment_{false};
  // Before the first accepted alignment, reject map->odom hypotheses that leave
  // the configured init_pose basin (straight189 accepted yaw~-2.3 and poisoned
  // the planning grid free-space check at the nominal goal).
  double cold_start_prior_max_xy_m_{1.0};
  double cold_start_prior_max_yaw_rad_{0.60};

  // 组合评分与歧义拒绝。权重/阈值由候选分布决定，min_score_margin<=0 表示门未启用。
  CandidateScoreWeights score_weights_;
  AmbiguityConfig ambiguity_;
  double confirmation_min_interval_s_{0.05};
  double confirmation_motion_translation_tolerance_{0.25};
  double confirmation_motion_yaw_tolerance_{0.15};
  double motion_reference_max_age_s_{2.0};

  // Status-driven recovery: TRACKING keeps fine-only; LOST opens async multi_guess.
  bool follow_localization_status_{true};
  bool auto_multi_guess_on_lost_{true};
  bool force_registration_when_lost_{true};
  std::uint8_t localization_state_{
    ats_navigation_interfaces::msg::LocalizationStatus::STATE_UNINITIALIZED};
  std::uint64_t localization_epoch_{0};
  bool recovery_from_lost_{false};
  double observation_silence_sec_{0.0};
  std::optional<rclcpp::Time> last_status_receive_time_;
  std::int64_t last_status_stamp_ns_{0};
  // After a stall (e.g. SIGSTOP), skip registration until a fresh status sample arrives
  // so we do not act on a ghost TRACKING state that missed LOST transitions.
  double status_stale_skip_registration_s_{1.0};
  bool height_filter_enable_{false};
  float height_filter_min_z_{-0.5f};
  float height_filter_max_z_{2.5f};

  // LOST multi_guess runs off the ROS callback thread so /initialpose stays responsive.
  RelocalizationGeneration relocalization_generation_;
  std::uint64_t stale_async_result_count_{0};
  std::atomic<bool> multi_guess_running_{false};
  std::atomic<bool> cancel_multi_guess_{false};
  std::mutex async_result_mutex_;
  bool async_result_ready_{false};
  MultiGuessOutcome async_result_;
  rclcpp::Time async_result_scan_time_;
  std::size_t async_result_source_points_{0};
  std::thread multi_guess_thread_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string robot_base_frame_;
  std::string current_scan_frame_id_;
  rclcpp::Time last_scan_time_;
  std::optional<rclcpp::Time> initial_pose_override_time_;
  bool has_received_scan_{false};
  std::int64_t last_received_scan_stamp_ns_{0};
  std::uint64_t accepted_scan_count_{0};
  std::uint64_t dropped_scan_count_{0};
  std::uint64_t stale_scan_count_{0};
  std::uint64_t invalid_scan_count_{0};
  std::uint64_t trimmed_accumulation_count_{0};
  std::uint64_t sampled_scan_points_count_{0};
  std::uint64_t evicted_scan_frames_count_{0};
  std::uint64_t observation_sequence_{0};
  int pending_confirmation_count_{0};
  std::optional<ConfirmationSample> pending_confirmation_;
  std::optional<std::chrono::steady_clock::time_point> confirmation_started_at_;
  // Set when the pending episode was opened under cold lattice search (LOST /
  // BOOTSTRAP / multi_guess). Survives brief status flicker during async apply.
  bool pending_holds_lattice_origin_{false};
  double confirmation_timeout_s_{10.0};
  // 上一帧通过硬门的假设，用于候选级 motion 一致性软约束。
  std::optional<ConfirmationSample> last_hypothesis_;
  std::optional<rclcpp::Time> last_hypothesis_time_;
  std::optional<rclcpp::Time> last_accepted_observation_scan_time_;
  Eigen::Isometry3d result_t_;
  Eigen::Isometry3d previous_result_t_;
  std::optional<Eigen::Isometry3d> last_registration_robot_base_to_odom_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_;
  ScanAccumulationWindow<pcl::PointCloud<pcl::PointXYZ>::VectorType> scan_window_{40000, 30, 1};
  PointCovarianceCloud::Ptr target_;
  PointCovarianceCloud::Ptr source_;
  /// coarse 候选筛选云。与 source_ 同一帧、同一滤波，只是更稀疏。
  PointCovarianceCloud::Ptr screen_source_;

  std::shared_ptr<PointKdTree> target_tree_;
  std::shared_ptr<PointKdTree> source_tree_;
  std::shared_ptr<PointKdTree> screen_source_tree_;
  std::shared_ptr<GicpRegistration> register_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace small_gicp_relocalization

#endif  // SMALL_GICP_RELOCALIZATION__SMALL_GICP_RELOCALIZATION_HPP_
