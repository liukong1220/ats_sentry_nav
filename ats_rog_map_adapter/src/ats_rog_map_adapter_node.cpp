// Copyright 2026

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ats_navigation_interfaces/msg/localization_status.hpp"
#include "ats_navigation_interfaces/msg/planning_map_snapshot.hpp"
#include "ats_navigation_interfaces/msg/planning_map_status.hpp"
#include "ats_rog_map_adapter/ground_projection_fusion.hpp"
#include "ats_rog_map_adapter/planning_map_snapshot.hpp"
#include "ats_rog_map_interfaces/srv/get_rog_map_projection.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "ats_rc_esdf/esdf/rc_traversability_esdf_provider.hpp"

namespace ats_rog_map_adapter
{

class AtsRogMapAdapterNode final : public rclcpp::Node
{
public:
  AtsRogMapAdapterNode()
  : Node("ats_rog_map_adapter"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    projection_service_ = declare_parameter<std::string>(
      "projection_service", "/rog_map/get_ground_projection");
    projection_rate_hz_ = std::max(0.1, declare_parameter<double>("projection_rate_hz", 2.0));
    projection_request_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("projection_request_timeout_sec", 2.0));
    projection_snapshot_timeout_sec_ = std::max(
      0.1, declare_parameter<double>("projection_snapshot_timeout_sec", 2.0));
    projection_min_height_ = declare_parameter<double>("projection_min_height", 0.10);
    projection_max_height_ = declare_parameter<double>("projection_max_height", 0.80);
    planning_resolution_ = std::max(0.01, declare_parameter<double>("planning_resolution", 0.10));
    planning_grid_owner_ = declare_parameter<std::string>("planning_grid_owner", "rog_map");
    if (planning_grid_owner_ != "rog_map" && planning_grid_owner_ != "rc_esdf") {
      throw std::invalid_argument(
        "planning_grid_owner must be 'rog_map' or 'rc_esdf'");
    }
    if (planning_grid_owner_ == "rc_esdf") {
      throw std::runtime_error(
        "rc_esdf planning-grid owner has no active publisher in this launch; refusing duplicate ownership");
    }
    static_map_topic_ = declare_parameter<std::string>("static_map_topic", "/map");
    traversability_grid_topic_ = declare_parameter<std::string>(
      "traversability_grid_topic", "/traversability_grid");
    slope_grid_topic_ = declare_parameter<std::string>(
      "traversability_slope_grid_topic", "/traversability_slope_grid");
    planning_grid_topic_ = declare_parameter<std::string>(
      "planning_grid_topic", "/rc_esdf/planning_grid");
    signed_distance_grid_topic_ = declare_parameter<std::string>(
      "signed_distance_grid_topic", "/rc_esdf/signed_distance_grid");
    footprint_clearance_grid_topic_ = declare_parameter<std::string>(
      "footprint_clearance_grid_topic", "/rc_esdf/footprint_clearance_grid");
    ready_topic_ = declare_parameter<std::string>("ready_topic", "/rog_map_adapter/ready");
    generation_topic_ = declare_parameter<std::string>(
      "generation_topic", "/rog_map_adapter/generation");
    map_status_topic_ = declare_parameter<std::string>(
        "map_status_topic", "/rog_map_adapter/status");
    planning_snapshot_topic_ = declare_parameter<std::string>(
      "planning_snapshot_topic", "/rog_map_adapter/planning_snapshot");
    localization_status_topic_ = declare_parameter<std::string>(
        "localization_status_topic", "/localization/status");
    require_localization_status_ =
        declare_parameter<bool>("require_localization_status", false);
    localization_status_timeout_sec_ = std::max(
        0.1, declare_parameter<double>("localization_status_timeout_sec", 1.0));
    fusion_params_.static_obstacle_value_threshold = declare_parameter<int>(
      "static_obstacle_value_threshold", 50);
    fusion_params_.terrain_obstacle_value_threshold = declare_parameter<int>(
      "terrain_obstacle_value_threshold", 50);
    fusion_params_.slope_grid_max_degrees = declare_parameter<double>(
      "slope_grid_max_degrees", 45.0);
    fusion_params_.slope_obstacle_degrees = declare_parameter<double>(
      "slope_obstacle_degrees", 28.0);
    fusion_params_.planning_resolution = planning_resolution_;
    fusion_params_.unknown_is_obstacle = declare_parameter<bool>("unknown_is_obstacle", true);
    input_timeout_sec_ = std::max(0.0, declare_parameter<double>("input_timeout_sec", 2.0));
    input_sync_tolerance_sec_ = std::max(
      0.0, declare_parameter<double>("input_sync_tolerance_sec", 1.0));
    signed_distance_max_m_ = std::max(
      1e-3, declare_parameter<double>("signed_distance_max_m", 2.0));
    footprint_length_ = std::max(0.0, declare_parameter<double>("footprint_length", 0.70));
    footprint_width_ = std::max(0.0, declare_parameter<double>("footprint_width", 0.55));
    footprint_safety_margin_ = std::max(
      0.0, declare_parameter<double>("footprint_safety_margin", 0.05));
    robot_frame_ = declare_parameter<std::string>("robot_frame", "gimbal_yaw_odom");
    robot_unknown_clear_radius_ = std::max(
      0.0, declare_parameter<double>("robot_unknown_clear_radius", 0.0));
    // Only for an isolated source-unknown fault.  Secondary evidence is
    // masked before the normal truth table runs; the fused output is never
    // overwritten after fusion.
    declare_parameter<bool>("test_mask_secondary_evidence", false);
    // P4 runtime swept-volume injection.  These values are only sampled when
    // explicitly enabled at runtime and never affect the normal map contract.
    declare_parameter<bool>("test_inject_dynamic_obstacle", false);
    declare_parameter<double>("test_dynamic_obstacle_x", 0.0);
    declare_parameter<double>("test_dynamic_obstacle_y", 0.0);
    declare_parameter<double>("test_dynamic_obstacle_radius", 0.10);

    static_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      static_map_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        static_map_ = std::move(msg);
      });
    traversability_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      traversability_grid_topic_, rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        traversability_grid_ = std::move(msg);
      });
    slope_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      slope_grid_topic_, rclcpp::QoS(10).reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(input_mutex_);
        slope_grid_ = std::move(msg);
      });
    localization_status_sub_ =
        create_subscription<ats_navigation_interfaces::msg::LocalizationStatus>(
            localization_status_topic_,
            rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&AtsRogMapAdapterNode::onLocalizationStatus, this,
                      std::placeholders::_1));

    const auto output_qos = rclcpp::QoS(1).reliable().transient_local();
    planning_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      planning_grid_topic_, output_qos);
    signed_distance_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      signed_distance_grid_topic_, output_qos);
    footprint_clearance_grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      footprint_clearance_grid_topic_, output_qos);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, output_qos);
    generation_pub_ = create_publisher<std_msgs::msg::UInt64>(generation_topic_, output_qos);
    map_status_pub_ =
        create_publisher<ats_navigation_interfaces::msg::PlanningMapStatus>(
            map_status_topic_, output_qos);
    planning_snapshot_pub_ = create_publisher<
      ats_navigation_interfaces::msg::PlanningMapSnapshot>(planning_snapshot_topic_, output_qos);
    projection_client_ = create_client<ats_rog_map_interfaces::srv::GetRogMapProjection>(
      projection_service_);

    publishMapStatus(false, 0, "waiting for first projection");
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / projection_rate_hz_));
    projection_timer_ = create_wall_timer(
      std::max(period, std::chrono::milliseconds(1)),
      std::bind(&AtsRogMapAdapterNode::requestProjection, this));
    RCLCPP_INFO(
      get_logger(),
      "ROGMap ground adapter ready: service='%s' planning_grid='%s' height=[%.2f, %.2f] m",
      projection_service_.c_str(), planning_grid_topic_.c_str(), projection_min_height_,
      projection_max_height_);
  }

private:
  void onLocalizationStatus(
      const ats_navigation_interfaces::msg::LocalizationStatus::SharedPtr
          message) {
    if (!require_localization_status_) {
      return;
    }
    const bool epoch_changed =
        has_localization_status_ && message->epoch != localization_epoch_;
    has_localization_status_ = true;
    localization_epoch_ = message->epoch;
    localization_state_ = message->state;
    last_localization_status_signal_ = std::chrono::steady_clock::now();
    if (epoch_changed || message->state !=
                             ats_navigation_interfaces::msg::
                                 LocalizationStatus::STATE_TRACKING) {
      if (request_pending_) {
        projection_client_->remove_pending_request(active_request_id_);
        request_pending_ = false;
      }
      ++active_request_epoch_;
      RogMapEsdfSnapshot invalidated_snapshot;
      last_numeric_snapshot_ = std::move(invalidated_snapshot);
      publishUnavailable(
          epoch_changed
              ? "localization epoch changed; invalidating planning snapshot"
              : "localization is not tracking");
    }
  }

  bool localizationReady() const {
    if (!require_localization_status_) {
      return true;
    }
    return has_localization_status_ && last_localization_status_signal_ &&
           localization_state_ == ats_navigation_interfaces::msg::
                                      LocalizationStatus::STATE_TRACKING &&
           std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         *last_localization_status_signal_)
                   .count() <= localization_status_timeout_sec_;
  }

  bool inputFresh(const nav_msgs::msg::OccupancyGrid & grid) const
  {
    if (input_timeout_sec_ <= 0.0) {
      return true;
    }
    const rclcpp::Time stamp(grid.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      return false;
    }
    const double age = (now() - stamp).seconds();
    return age >= -0.1 && age <= input_timeout_sec_;
  }

  bool projectionFresh(const nav_msgs::msg::OccupancyGrid & grid) const
  {
    const rclcpp::Time stamp(grid.header.stamp);
    if (stamp.nanoseconds() <= 0) {
      return false;
    }
    const double age = (now() - stamp).seconds();
    return age >= -0.1 && age <= projection_snapshot_timeout_sec_;
  }

  bool inputSynchronized(
    const nav_msgs::msg::OccupancyGrid & grid,
    const builtin_interfaces::msg::Time & projection_stamp) const
  {
    if (input_sync_tolerance_sec_ <= 0.0) {
      return true;
    }
    const rclcpp::Time input_stamp(grid.header.stamp);
    const rclcpp::Time rog_stamp(projection_stamp);
    return input_stamp.nanoseconds() > 0 && rog_stamp.nanoseconds() > 0 &&
           std::abs((input_stamp - rog_stamp).seconds()) <= input_sync_tolerance_sec_;
  }

  void requestProjection()
  {
    if (!localizationReady()) {
      publishUnavailable(
          "localization status is missing, stale, or not tracking");
      return;
    }
    if (request_pending_) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - active_request_sent_time_).count();
      if (elapsed <= projection_request_timeout_sec_) {
        return;
      }
      projection_client_->remove_pending_request(active_request_id_);
      request_pending_ = false;
      publishUnavailable("ROGMap projection request timed out");
      return;
    }
    if (!projection_client_->service_is_ready()) {
      publishUnavailable("ROGMap projection service is unavailable");
      return;
    }
    auto request = std::make_shared<ats_rog_map_interfaces::srv::GetRogMapProjection::Request>();
    request->min_height = static_cast<float>(projection_min_height_);
    request->max_height = static_cast<float>(projection_max_height_);
    request->resolution = static_cast<float>(planning_resolution_);
    const std::uint64_t epoch = ++next_request_epoch_;
    const std::uint64_t localization_epoch = localization_epoch_;
    const rclcpp::Time request_start_stamp = now();
    active_request_epoch_ = epoch;
    active_request_sent_time_ = std::chrono::steady_clock::now();
    request_pending_ = true;
    try {
      const auto pending = projection_client_->async_send_request(
          request,
          [this, epoch, localization_epoch, request_start_stamp](
              rclcpp::Client<ats_rog_map_interfaces::srv::GetRogMapProjection>::
                  SharedFuture future) {
            if (!request_pending_ || epoch != active_request_epoch_) {
              return;
            }
            request_pending_ = false;
            try {
              if (!localizationReady() ||
                  localization_epoch != localization_epoch_) {
                publishUnavailable(
                    "discarded projection from an obsolete localization epoch");
                return;
              }
              const auto & response = *future.get();
              const rclcpp::Time response_stamp = now();
              const double round_trip_ms = 1000.0 * std::chrono::duration<double>(
                std::chrono::steady_clock::now() - active_request_sent_time_).count();
              RCLCPP_INFO(
                get_logger(),
                "P2 adapter projection end request=%llu start_ns=%lld end_ns=%lld "
                "round_trip_ms=%.1f source_generation=%llu source_stamp_ns=%lld ready=%d stale=%d",
                static_cast<unsigned long long>(epoch),
                static_cast<long long>(request_start_stamp.nanoseconds()),
                static_cast<long long>(response_stamp.nanoseconds()), round_trip_ms,
                static_cast<unsigned long long>(response.generation),
                static_cast<long long>(rclcpp::Time(response.occupancy_grid.header.stamp).nanoseconds()),
                response.ready ? 1 : 0, response.stale ? 1 : 0);
              processProjection(response, localization_epoch, epoch);
            } catch (const std::exception &exception) {
              RCLCPP_ERROR(get_logger(),
                           "ROGMap projection response failed: %s",
                           exception.what());
              publishUnavailable("ROGMap projection response failed");
            }
          });
      active_request_id_ = pending.request_id;
      RCLCPP_INFO(
        get_logger(),
        "P2 adapter projection begin request=%llu request_id=%ld start_ns=%lld "
        "localization_epoch=%llu deadline_sec=%.3f",
        static_cast<unsigned long long>(epoch), static_cast<long>(active_request_id_),
        static_cast<long long>(request_start_stamp.nanoseconds()),
        static_cast<unsigned long long>(localization_epoch), projection_request_timeout_sec_);
    } catch (const std::exception & exception) {
      request_pending_ = false;
      RCLCPP_ERROR(get_logger(), "ROGMap projection request failed: %s", exception.what());
      publishUnavailable("ROGMap projection request failed");
    }
  }

  void processProjection(
      const ats_rog_map_interfaces::srv::GetRogMapProjection::Response
          &response,
      std::uint64_t localization_epoch, std::uint64_t request_epoch) {
    const RogMapEsdfSnapshot numeric_snapshot = RogMapEsdfSnapshot::fromResponse(response);
    if (response.occupancy_grid.info.width > 0 && response.occupancy_grid.info.height > 0 &&
      !response.occupancy_grid.data.empty())
    {
      last_blocking_grid_ = response.occupancy_grid;
    }
    // Keep every ready=false heartbeat tied to the latest numeric response;
    // unavailable paths must not report an older ROGMap source generation.
    last_rog_generation_ = response.generation;
    nav_msgs::msg::OccupancyGrid::SharedPtr static_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr traversability;
    nav_msgs::msg::OccupancyGrid::SharedPtr slope;
    {
      std::lock_guard<std::mutex> lock(input_mutex_);
      static_map = static_map_;
      traversability = traversability_grid_;
      slope = slope_grid_;
    }
    const auto message_stamp_ns = [](const nav_msgs::msg::OccupancyGrid::SharedPtr & grid) {
        return grid ? static_cast<long long>(rclcpp::Time(grid->header.stamp).nanoseconds()) : 0LL;
      };
    const auto message_age = [this](const nav_msgs::msg::OccupancyGrid::SharedPtr & grid) {
        if (!grid || rclcpp::Time(grid->header.stamp).nanoseconds() <= 0) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        return (now() - rclcpp::Time(grid->header.stamp)).seconds();
      };
    const auto timestamp_delta = [](const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
                                    const builtin_interfaces::msg::Time & projection_stamp) {
        if (!grid || rclcpp::Time(grid->header.stamp).nanoseconds() <= 0 ||
          rclcpp::Time(projection_stamp).nanoseconds() <= 0)
        {
          return std::numeric_limits<double>::quiet_NaN();
        }
        return std::abs((rclcpp::Time(grid->header.stamp) -
          rclcpp::Time(projection_stamp)).seconds());
      };
    RCLCPP_INFO(
      get_logger(),
      "P2 adapter projection inputs request=%llu localization_epoch=%llu source_generation=%llu "
      "projection_stamp_ns=%lld terrain_stamp_ns=%lld terrain_age=%.3f terrain_delta=%.3f "
      "slope_stamp_ns=%lld slope_age=%.3f slope_delta=%.3f",
      static_cast<unsigned long long>(request_epoch),
      static_cast<unsigned long long>(localization_epoch),
      static_cast<unsigned long long>(response.generation),
      static_cast<long long>(rclcpp::Time(response.occupancy_grid.header.stamp).nanoseconds()),
      message_stamp_ns(traversability), message_age(traversability),
      timestamp_delta(traversability, response.occupancy_grid.header.stamp),
      message_stamp_ns(slope), message_age(slope),
      timestamp_delta(slope, response.occupancy_grid.header.stamp));
    if (!numeric_snapshot.available() || !projectionFresh(response.occupancy_grid)) {
      publishUnavailable("ROGMap projection is unavailable or stale");
      return;
    }
    const bool mask_secondary_evidence =
      get_parameter("test_mask_secondary_evidence").as_bool();
    if (mask_secondary_evidence && std::none_of(
        response.occupancy_grid.data.begin(), response.occupancy_grid.data.end(),
        [](const std::int8_t value) {return value < 0;}))
    {
      publishUnavailable(
        "test source-unknown fixture refused: ROGMap numeric projection has no unknown cells");
      return;
    }
    const bool traversability_fresh = traversability && inputFresh(*traversability);
    const bool slope_fresh = slope && inputFresh(*slope);
    const bool traversability_synchronized = traversability && inputSynchronized(
      *traversability, response.occupancy_grid.header.stamp);
    const bool slope_synchronized = slope && inputSynchronized(
      *slope, response.occupancy_grid.header.stamp);
    if (!static_map || !traversability_fresh || !slope_fresh ||
      !traversability_synchronized || !slope_synchronized)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Terrain rejected: static=%d traversal fresh=%d sync=%d age=%.3f delta=%.3f; "
        "slope fresh=%d sync=%d age=%.3f delta=%.3f",
        static_map ? 1 : 0, traversability_fresh ? 1 : 0,
        traversability_synchronized ? 1 : 0, message_age(traversability),
        timestamp_delta(traversability, response.occupancy_grid.header.stamp),
        slope_fresh ? 1 : 0, slope_synchronized ? 1 : 0, message_age(slope),
        timestamp_delta(slope, response.occupancy_grid.header.stamp));
      publishUnavailable("Terrain inputs are missing, stale, or unsynchronized");
      return;
    }
    if (mask_secondary_evidence) {
      const auto unknown_copy = [](const nav_msgs::msg::OccupancyGrid & source) {
          auto masked = std::make_shared<nav_msgs::msg::OccupancyGrid>(source);
          std::fill(masked->data.begin(), masked->data.end(), static_cast<std::int8_t>(-1));
          return masked;
        };
      static_map = unknown_copy(*static_map);
      traversability = unknown_copy(*traversability);
      slope = unknown_copy(*slope);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "P2 test fixture masks static/terrain/slope evidence before fusion; "
        "ROGMap numeric unknown remains the only source evidence");
    }

    geometry_msgs::msg::TransformStamped static_from_projection;
    geometry_msgs::msg::TransformStamped static_from_robot;
    try {
      static_from_projection = tf_buffer_.lookupTransform(
        static_map->header.frame_id, response.occupancy_grid.header.frame_id,
        rclcpp::Time(response.occupancy_grid.header.stamp), rclcpp::Duration::from_seconds(0.1));
      static_from_robot = tf_buffer_.lookupTransform(
        static_map->header.frame_id, robot_frame_,
        rclcpp::Time(response.occupancy_grid.header.stamp), rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ROGMap adapter waits for static-map TF: %s",
        exception.what());
      publishUnavailable("ROGMap projection transform is unavailable");
      return;
    }

    GroundProjectionFusionResult fusion;
    if (!GroundProjectionFusion::fuse(
        response.occupancy_grid, *traversability, *slope, *static_map,
        static_from_projection, fusion_params_, fusion))
    {
      publishUnavailable("ROGMap terrain fusion failed");
      return;
    }
    const std::size_t ego_unknown_cleared = GroundProjectionFusion::clearUnknownCircle(
      fusion, static_from_robot.transform.translation.x, static_from_robot.transform.translation.y,
      robot_unknown_clear_radius_);
    last_blocking_grid_ = fusion.planning_grid;
    if (fusion.known_free_cells == 0) {
      publishUnavailable("ROGMap terrain fusion produced no known-free cells", true);
      return;
    }
    const std::size_t injected_cells = injectDynamicObstacleForTest(fusion.planning_grid);

    ats_rc_esdf::RcTraversabilityEsdfProvider esdf;
    esdf.updateGrid(
      fusion.planning_grid, fusion_params_.terrain_obstacle_value_threshold,
      fusion_params_.unknown_is_obstacle);
    std::vector<double> signed_distance;
    if (!esdf.copySignedDistanceField(signed_distance)) {
      last_blocking_grid_ = fusion.planning_grid;
      publishUnavailable("Fused RC-ESDF construction failed");
      return;
    }

    if (!localizationReady() || localization_epoch != localization_epoch_) {
      publishUnavailable(
          "localization changed before planning snapshot commit");
      return;
    }
    last_numeric_snapshot_ = numeric_snapshot;
    last_blocking_grid_ = fusion.planning_grid;
    const builtin_interfaces::msg::Time publication_stamp = now();
    const auto publication_sequence = map_status_sequence_ + 1U;
    const auto planning_snapshot = makePlanningMapSnapshot(
      fusion.planning_grid, signed_distance, publication_stamp, localization_epoch,
      response.generation, publication_sequence, fusion_params_.unknown_is_obstacle,
      fusion_params_.terrain_obstacle_value_threshold);
    if (!planning_snapshot.ready) {
      publishUnavailable("fused planning snapshot serialization failed");
      return;
    }
    planning_grid_pub_->publish(fusion.planning_grid);
    signed_distance_grid_pub_->publish(
      encodeDistanceGrid(fusion.planning_grid, signed_distance, 0.0));
    const double footprint_radius = std::hypot(
      0.5 * footprint_length_ + footprint_safety_margin_,
      0.5 * footprint_width_ + footprint_safety_margin_);
    footprint_clearance_grid_pub_->publish(
      encodeDistanceGrid(fusion.planning_grid, signed_distance, footprint_radius));
    std_msgs::msg::UInt64 generation;
    generation.data = response.generation;
    generation_pub_->publish(generation);
    last_rog_generation_ = response.generation;
    planning_snapshot_pub_->publish(planning_snapshot);
    publishMapStatus(true, response.generation, "planning snapshot ready");
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "P2 snapshot generation=%llu free=%zu occupied=%zu unknown=%zu ego_clear=%zu injected=%zu numeric_esdf=%zu",
      static_cast<unsigned long long>(response.generation), fusion.known_free_cells,
      fusion.occupied_cells, fusion.unknown_cells, ego_unknown_cleared, injected_cells,
      numeric_snapshot.signed_distance.size());
  }

  std::size_t injectDynamicObstacleForTest(nav_msgs::msg::OccupancyGrid & grid)
  {
    if (!get_parameter("test_inject_dynamic_obstacle").as_bool() ||
      grid.info.resolution <= 0.0 || grid.data.empty())
    {
      return 0;
    }
    const double world_x = get_parameter("test_dynamic_obstacle_x").as_double();
    const double world_y = get_parameter("test_dynamic_obstacle_y").as_double();
    const double radius = std::max(
      0.0, get_parameter("test_dynamic_obstacle_radius").as_double());
    const double yaw = std::atan2(
      2.0 * (grid.info.origin.orientation.w * grid.info.origin.orientation.z +
        grid.info.origin.orientation.x * grid.info.origin.orientation.y),
      1.0 - 2.0 * (grid.info.origin.orientation.y * grid.info.origin.orientation.y +
        grid.info.origin.orientation.z * grid.info.origin.orientation.z));
    const double dx = world_x - grid.info.origin.position.x;
    const double dy = world_y - grid.info.origin.position.y;
    const double grid_x =
      (std::cos(yaw) * dx + std::sin(yaw) * dy) / grid.info.resolution;
    const double grid_y =
      (-std::sin(yaw) * dx + std::cos(yaw) * dy) / grid.info.resolution;
    const int center_x = static_cast<int>(std::floor(grid_x));
    const int center_y = static_cast<int>(std::floor(grid_y));
    const double conservative_radius = radius +
      0.5 * std::sqrt(2.0) * grid.info.resolution;
    const int radius_cells = std::max(
      0, static_cast<int>(std::ceil(conservative_radius / grid.info.resolution)));
    std::size_t injected = 0;
    for (int y = center_y - radius_cells; y <= center_y + radius_cells; ++y) {
      for (int x = center_x - radius_cells; x <= center_x + radius_cells; ++x) {
        if (x < 0 || y < 0 || x >= static_cast<int>(grid.info.width) ||
          y >= static_cast<int>(grid.info.height))
        {
          continue;
        }
        const double cell_dx =
          (static_cast<double>(x) + 0.5 - grid_x) * grid.info.resolution;
        const double cell_dy =
          (static_cast<double>(y) + 0.5 - grid_y) * grid.info.resolution;
        if (cell_dx * cell_dx + cell_dy * cell_dy >
          conservative_radius * conservative_radius)
        {
          continue;
        }
        const std::size_t index = static_cast<std::size_t>(y) * grid.info.width + x;
        if (grid.data[index] < fusion_params_.terrain_obstacle_value_threshold) {
          ++injected;
        }
        grid.data[index] = static_cast<int8_t>(
          std::max(fusion_params_.terrain_obstacle_value_threshold, 100));
      }
    }
    return injected;
  }

  nav_msgs::msg::OccupancyGrid encodeDistanceGrid(
    const nav_msgs::msg::OccupancyGrid & planning_grid,
    const std::vector<double> & distance_field, double clearance_offset) const
  {
    nav_msgs::msg::OccupancyGrid encoded = planning_grid;
    encoded.data.assign(planning_grid.data.size(), -1);
    const std::size_t size = std::min(planning_grid.data.size(), distance_field.size());
    for (std::size_t index = 0; index < size; ++index) {
      if (planning_grid.data[index] < 0 || !std::isfinite(distance_field[index])) {
        continue;
      }
      const double normalized = std::clamp(
        (distance_field[index] - clearance_offset) / signed_distance_max_m_, -1.0, 1.0);
      encoded.data[index] = static_cast<int8_t>(std::lround(50.0 + 50.0 * normalized));
    }
    return encoded;
  }

  void publishBlockedGrid(const nav_msgs::msg::OccupancyGrid & source)
  {
    if (source.info.width == 0 || source.info.height == 0 || source.data.empty()) {
      return;
    }
    nav_msgs::msg::OccupancyGrid blocked = source;
    blocked.data.assign(source.data.size(), -1);
    planning_grid_pub_->publish(blocked);
  }

  void publishUnavailable(const char * reason, bool include_blocking_grid = false)
  {
    const auto publication_sequence = map_status_sequence_ + 1U;
    const builtin_interfaces::msg::Time publication_stamp = now();
    if (include_blocking_grid) {
      planning_snapshot_pub_->publish(makeBlockedUnavailablePlanningMapSnapshot(
        last_blocking_grid_, publication_stamp, localization_epoch_, last_rog_generation_,
        publication_sequence, fusion_params_.unknown_is_obstacle,
        fusion_params_.terrain_obstacle_value_threshold));
    } else {
      planning_snapshot_pub_->publish(makeUnavailablePlanningMapSnapshot(
        last_blocking_grid_, publication_stamp, localization_epoch_, last_rog_generation_,
        publication_sequence, fusion_params_.unknown_is_obstacle,
        fusion_params_.terrain_obstacle_value_threshold));
    }
    publishMapStatus(false, last_rog_generation_, reason);
    publishBlockedGrid(last_blocking_grid_);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason);
  }

  void publishMapStatus(bool ready, std::uint64_t rog_generation,
                        const std::string &reason) {
    std_msgs::msg::Bool message;
    message.data = ready;
    ready_pub_->publish(message);

    ats_navigation_interfaces::msg::PlanningMapStatus status;
    status.header.stamp = now();
    status.header.frame_id = last_blocking_grid_.header.frame_id;
    status.ready = ready;
    status.localization_epoch = localization_epoch_;
    status.rog_generation = rog_generation;
    status.publication_sequence = ++map_status_sequence_;
    status.message = reason;
    map_status_pub_->publish(status);
    RCLCPP_INFO(
      get_logger(),
      "P2 adapter heartbeat ready=%d publication_sequence=%llu source_generation=%llu "
      "localization_epoch=%llu stamp_ns=%lld reason=%s",
      ready ? 1 : 0, static_cast<unsigned long long>(status.publication_sequence),
      static_cast<unsigned long long>(status.rog_generation),
      static_cast<unsigned long long>(status.localization_epoch),
      static_cast<long long>(rclcpp::Time(status.header.stamp).nanoseconds()), reason.c_str());
  }

  std::string projection_service_;
  std::string static_map_topic_;
  std::string traversability_grid_topic_;
  std::string slope_grid_topic_;
  std::string planning_grid_topic_;
  std::string planning_grid_owner_;
  std::string signed_distance_grid_topic_;
  std::string footprint_clearance_grid_topic_;
  std::string ready_topic_;
  std::string generation_topic_;
  std::string map_status_topic_;
  std::string planning_snapshot_topic_;
  std::string localization_status_topic_;
  std::string robot_frame_;
  double projection_rate_hz_{2.0};
  double projection_request_timeout_sec_{2.0};
  double projection_snapshot_timeout_sec_{2.0};
  double projection_min_height_{0.10};
  double projection_max_height_{0.80};
  double planning_resolution_{0.10};
  double input_timeout_sec_{2.0};
  double input_sync_tolerance_sec_{1.0};
  double signed_distance_max_m_{2.0};
  double footprint_length_{0.70};
  double footprint_width_{0.55};
  double footprint_safety_margin_{0.05};
  double robot_unknown_clear_radius_{0.0};
  double localization_status_timeout_sec_{1.0};
  bool require_localization_status_{false};
  GroundProjectionFusionParams fusion_params_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::mutex input_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr traversability_grid_;
  nav_msgs::msg::OccupancyGrid::SharedPtr slope_grid_;
  nav_msgs::msg::OccupancyGrid last_blocking_grid_;
  RogMapEsdfSnapshot last_numeric_snapshot_;
  bool request_pending_{false};
  std::int64_t active_request_id_{0};
  std::uint64_t next_request_epoch_{0};
  std::uint64_t active_request_epoch_{0};
  std::uint64_t localization_epoch_{0};
  std::uint64_t last_rog_generation_{0};
  std::uint64_t map_status_sequence_{0};
  std::uint8_t localization_state_{
      ats_navigation_interfaces::msg::LocalizationStatus::STATE_UNINITIALIZED};
  bool has_localization_status_{false};
  std::optional<std::chrono::steady_clock::time_point>
      last_localization_status_signal_;
  std::chrono::steady_clock::time_point active_request_sent_time_{};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr static_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr traversability_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr slope_sub_;
  rclcpp::Subscription<ats_navigation_interfaces::msg::LocalizationStatus>::
      SharedPtr localization_status_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr planning_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr signed_distance_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr footprint_clearance_grid_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr generation_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::PlanningMapStatus>::
      SharedPtr map_status_pub_;
  rclcpp::Publisher<ats_navigation_interfaces::msg::PlanningMapSnapshot>::
      SharedPtr planning_snapshot_pub_;
  rclcpp::Client<ats_rog_map_interfaces::srv::GetRogMapProjection>::SharedPtr projection_client_;
  rclcpp::TimerBase::SharedPtr projection_timer_;
};

}  // namespace ats_rog_map_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ats_rog_map_adapter::AtsRogMapAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
