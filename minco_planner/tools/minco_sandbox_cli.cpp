// Copyright 2026
//
// minco_sandbox_cli: 把真实 MincoPlannerNode 接到 参考/minco_sandbox 调试平台的命令行适配器。
//
// 做法: 在本进程里构造真实节点(参数取自项目 YAML), 把沙盒场景(圆/矩形/地图边界)栅格化成
// OccupancyGrid 后同步喂给 onGrid, 再同步调 planGoal, 最后读取节点提交的 ReferenceTrajectory
// 导出 JSON。所以沙盒里看到的是完整规划链(JPS -> 预处理 -> 联合优化 -> 足迹门禁 -> yaw ->
// 候选链 -> 提交门)的结果, 而不是另写一份近似实现。
//
// 私有成员访问: 节点的方法和状态全是 private, 只在本编译单元用 `#define private public`
// 打开, 不改生产头文件。node 头文件需要的标准库/ROS/项目头文件都先于宏包含, 宏只作用在
// minco_planner_node.hpp 自己。
//
// 输入(stdin, 文本, 每行一条, # 开头为注释):
//   bounds xmin xmax ymin ymax
//   start  x y [yaw vx vy ax ay]
//   goal   x y [yaw]
//   vmax   V                    (可选, 覆盖 max_velocity; 巡航速度仍由 reference_speed 决定)
//   circle x y r
//   rect   cx cy width height   (轴对齐)
// 选项:
//   --params FILE      ROS 参数 YAML(必须, 取其中 minco_planner 段)
//   --resolution R     栅格分辨率, 默认 0.10 m(与 adapter 一致)
//   --set key=value    追加/覆盖节点参数, 可重复
// 输出(stdout): 一个 JSON 对象。退出码 0=得到可提交参考, 1=规划失败, 2=输入错误。

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ats_navigation_interfaces/msg/planner_goal.hpp"
#include "ats_navigation_interfaces/msg/planner_status.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "minco_planner/debug/planner_debug_visualizer.hpp"
#include "minco_planner/nodes/planning_map_snapshot.hpp"
#include "minco_planner/planning/grid_astar.hpp"
#include "minco_planner/planning/grid_jps.hpp"
#include "minco_planner/safety/escape_prefix.hpp"
#include "minco_planner/safety/goal_pose_admission.hpp"
#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/safety/local_collision_repair.hpp"
#include "minco_planner/trajectory/minco_trajectory_optimizer.hpp"
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

#define private public
#include "minco_planner/nodes/minco_planner_node.hpp"
#undef private

namespace
{

using Clock = std::chrono::steady_clock;
using minco_planner::MincoPlannerNode;
using PlannerStatusMsg = ats_navigation_interfaces::msg::PlannerStatus;

double msSince(const Clock::time_point & t0)
{
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

struct Circle
{
  double x;
  double y;
  double r;
};

struct Rect
{
  double cx;
  double cy;
  double w;
  double h;
};

struct SceneInput
{
  bool has_bounds = false;
  bool has_start = false;
  bool has_goal = false;
  bool has_vmax = false;
  double xmin = 0.0;
  double xmax = 0.0;
  double ymin = 0.0;
  double ymax = 0.0;
  double sx = 0.0;
  double sy = 0.0;
  double syaw = 0.0;
  double svx = 0.0;
  double svy = 0.0;
  double sax = 0.0;
  double say = 0.0;
  double gx = 0.0;
  double gy = 0.0;
  double gyaw = 0.0;
  double vmax = 0.0;
  std::vector<Circle> circles;
  std::vector<Rect> rects;
};

struct Options
{
  std::string params_file;
  double resolution = 0.10;
  std::vector<std::string> overrides;
};

// ---- JSON 输出 -----------------------------------------------------------------

std::string jsonString(const std::string & text)
{
  std::string out = "\"";
  for (const char c : text) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", static_cast<unsigned>(c));
          out += buffer;
        } else {
          out += c;
        }
    }
  }
  return out + "\"";
}

std::string jsonNumber(double value)
{
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream stream;
  stream << std::setprecision(10) << value;
  return stream.str();
}

// ROS 参数覆盖按 YAML 标量解析, double 参数必须带小数点, 否则会被当成整数而类型不符。
std::string realLiteral(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

std::string pointsJson(const std::vector<std::array<double, 2>> & points)
{
  std::string out = "[";
  for (std::size_t i = 0; i < points.size(); ++i) {
    out += (i ? "," : "");
    out += "[" + jsonNumber(points[i][0]) + "," + jsonNumber(points[i][1]) + "]";
  }
  return out + "]";
}

const char * failureName(std::uint8_t reason)
{
  switch (reason) {
    case PlannerStatusMsg::FAILURE_NONE: return "NONE";
    case PlannerStatusMsg::FAILURE_INVALID_GOAL: return "INVALID_GOAL";
    case PlannerStatusMsg::FAILURE_MAP_UNREADY: return "MAP_UNREADY";
    case PlannerStatusMsg::FAILURE_START_TF: return "START_TF";
    case PlannerStatusMsg::FAILURE_GOAL_TF: return "GOAL_TF";
    case PlannerStatusMsg::FAILURE_START_OR_GOAL_OCCUPIED: return "START_OR_GOAL_OCCUPIED";
    case PlannerStatusMsg::FAILURE_NO_PATH: return "NO_PATH";
    case PlannerStatusMsg::FAILURE_OPTIMIZER: return "OPTIMIZER";
    case PlannerStatusMsg::FAILURE_FOOTPRINT: return "FOOTPRINT";
    case PlannerStatusMsg::FAILURE_REPAIR: return "REPAIR";
    case PlannerStatusMsg::FAILURE_REFERENCE_TF: return "REFERENCE_TF";
    case PlannerStatusMsg::FAILURE_SNAPSHOT_CHANGED: return "SNAPSHOT_CHANGED";
    case PlannerStatusMsg::FAILURE_RUNTIME_UNSAFE: return "RUNTIME_UNSAFE";
    case PlannerStatusMsg::FAILURE_START_ENCLOSED: return "START_ENCLOSED";
    default: return "UNKNOWN";
  }
}

// 沙盒界面约定的失败阶段: validation(输入本身不可规划) / seed_path(图搜索无路) /
// backend(地图或 TF 前置条件不满足) / optimizer(其余规划链环节)。
const char * failureStage(std::uint8_t reason)
{
  switch (reason) {
    case PlannerStatusMsg::FAILURE_INVALID_GOAL:
    case PlannerStatusMsg::FAILURE_START_OR_GOAL_OCCUPIED:
    case PlannerStatusMsg::FAILURE_START_ENCLOSED:
      return "validation";
    case PlannerStatusMsg::FAILURE_NO_PATH:
      return "seed_path";
    case PlannerStatusMsg::FAILURE_MAP_UNREADY:
    case PlannerStatusMsg::FAILURE_START_TF:
    case PlannerStatusMsg::FAILURE_GOAL_TF:
    case PlannerStatusMsg::FAILURE_REFERENCE_TF:
    case PlannerStatusMsg::FAILURE_SNAPSHOT_CHANGED:
      return "backend";
    default:
      return "optimizer";
  }
}

int emitFailure(const std::string & stage, const std::string & message, int code)
{
  std::cout << "{\"success\":false,\"stage\":" << jsonString(stage) << ",\"message\":" <<
    jsonString(message) << "}" << std::endl;
  return code;
}

// ---- 输入解析 ------------------------------------------------------------------

bool parseScene(std::istream & in, SceneInput * scene, std::string * error)
{
  std::string line;
  int line_number = 0;
  while (std::getline(in, line)) {
    ++line_number;
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line.erase(hash);
    }
    std::istringstream stream(line);
    std::string tag;
    if (!(stream >> tag)) {
      continue;
    }
    std::vector<double> values;
    double value = 0.0;
    while (stream >> value) {
      values.push_back(value);
    }
    if (!stream.eof()) {
      *error = "line " + std::to_string(line_number) + ": not a number";
      return false;
    }
    for (const double v : values) {
      if (!std::isfinite(v)) {
        *error = "line " + std::to_string(line_number) + ": non-finite value";
        return false;
      }
    }
    const auto need = [&](std::size_t low, std::size_t high) {
        if (values.size() < low || values.size() > high) {
          *error = "line " + std::to_string(line_number) + ": '" + tag + "' takes " +
            std::to_string(low) + "-" + std::to_string(high) + " numbers";
          return false;
        }
        return true;
      };
    if (tag == "bounds") {
      if (!need(4, 4)) {return false;}
      scene->has_bounds = true;
      scene->xmin = values[0];
      scene->xmax = values[1];
      scene->ymin = values[2];
      scene->ymax = values[3];
    } else if (tag == "start") {
      if (!need(2, 7)) {return false;}
      scene->has_start = true;
      scene->sx = values[0];
      scene->sy = values[1];
      scene->syaw = values.size() > 2 ? values[2] : 0.0;
      scene->svx = values.size() > 3 ? values[3] : 0.0;
      scene->svy = values.size() > 4 ? values[4] : 0.0;
      scene->sax = values.size() > 5 ? values[5] : 0.0;
      scene->say = values.size() > 6 ? values[6] : 0.0;
    } else if (tag == "goal") {
      if (!need(2, 3)) {return false;}
      scene->has_goal = true;
      scene->gx = values[0];
      scene->gy = values[1];
      scene->gyaw = values.size() > 2 ? values[2] : 0.0;
    } else if (tag == "vmax") {
      if (!need(1, 1)) {return false;}
      if (!(values[0] > 0.0)) {
        *error = "vmax must be positive";
        return false;
      }
      scene->has_vmax = true;
      scene->vmax = values[0];
    } else if (tag == "circle") {
      if (!need(3, 3)) {return false;}
      scene->circles.push_back({values[0], values[1], values[2]});
    } else if (tag == "rect") {
      if (!need(4, 4)) {return false;}
      scene->rects.push_back({values[0], values[1], values[2], values[3]});
    } else {
      *error = "line " + std::to_string(line_number) + ": unknown tag '" + tag + "'";
      return false;
    }
  }
  if (!scene->has_bounds || !scene->has_start || !scene->has_goal) {
    *error = "scene needs bounds, start and goal";
    return false;
  }
  if (!(scene->xmax > scene->xmin) || !(scene->ymax > scene->ymin)) {
    *error = "bounds must satisfy xmin < xmax and ymin < ymax";
    return false;
  }
  return true;
}

// ---- 栅格化 --------------------------------------------------------------------

// 单元只要与障碍有面积重叠就记 100(保守), 地图边界之外一律是障碍。栅格外围多补 kPad 圈
// 占据单元, 让边界墙在 ESDF 里有明确的表示; 世界坐标不变, 只是 origin 向外平移 kPad 格。
nav_msgs::msg::OccupancyGrid rasterize(
  const SceneInput & scene, double resolution, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  constexpr int kPad = 2;
  constexpr double kEps = 1.0e-6;
  const int nx = std::max(
    1, static_cast<int>(std::ceil((scene.xmax - scene.xmin) / resolution - kEps)));
  const int ny = std::max(
    1, static_cast<int>(std::ceil((scene.ymax - scene.ymin) / resolution - kEps)));
  const int width = nx + 2 * kPad;
  const int height = ny + 2 * kPad;
  const double ox = scene.xmin - kPad * resolution;
  const double oy = scene.ymin - kPad * resolution;

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = frame;
  grid.header.stamp = stamp;
  grid.info.map_load_time = stamp;
  grid.info.resolution = static_cast<float>(resolution);
  grid.info.width = static_cast<std::uint32_t>(width);
  grid.info.height = static_cast<std::uint32_t>(height);
  grid.info.origin.position.x = ox;
  grid.info.origin.position.y = oy;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      const double x0 = ox + i * resolution;
      const double y0 = oy + j * resolution;
      const double x1 = x0 + resolution;
      const double y1 = y0 + resolution;
      bool occupied = x0 < scene.xmin - kEps || x1 > scene.xmax + kEps ||
        y0 < scene.ymin - kEps || y1 > scene.ymax + kEps;
      for (std::size_t k = 0; !occupied && k < scene.circles.size(); ++k) {
        const Circle & c = scene.circles[k];
        const double qx = std::min(std::max(c.x, x0), x1);
        const double qy = std::min(std::max(c.y, y0), y1);
        occupied = (qx - c.x) * (qx - c.x) + (qy - c.y) * (qy - c.y) < c.r * c.r - kEps;
      }
      for (std::size_t k = 0; !occupied && k < scene.rects.size(); ++k) {
        const Rect & r = scene.rects[k];
        occupied = x0 < r.cx + 0.5 * r.w - kEps && x1 > r.cx - 0.5 * r.w + kEps &&
          y0 < r.cy + 0.5 * r.h - kEps && y1 > r.cy - 0.5 * r.h + kEps;
      }
      if (occupied) {
        grid.data[static_cast<std::size_t>(j) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(i)] = 100;
      }
    }
  }
  return grid;
}

geometry_msgs::msg::PoseStamped makePose(
  const std::string & frame, double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  pose.pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

struct PathCapture
{
  bool received = false;
  std::vector<std::array<double, 2>> points;
};

bool parseOptions(int argc, char ** argv, Options * options, std::string * error)
{
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&](std::string * out) {
        if (i + 1 >= argc) {
          *error = arg + " needs a value";
          return false;
        }
        *out = argv[++i];
        return true;
      };
    std::string text;
    if (arg == "--params") {
      if (!value(&options->params_file)) {return false;}
    } else if (arg == "--resolution") {
      if (!value(&text)) {return false;}
      options->resolution = std::atof(text.c_str());
      if (!(options->resolution > 0.0 && options->resolution <= 1.0)) {
        *error = "--resolution must be in (0, 1] m";
        return false;
      }
    } else if (arg == "--set") {
      if (!value(&text)) {return false;}
      if (text.find('=') == std::string::npos) {
        *error = "--set expects key=value";
        return false;
      }
      options->overrides.push_back(text);
    } else {
      *error = "unknown option " + arg;
      return false;
    }
  }
  if (options->params_file.empty()) {
    *error = "--params FILE is required";
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  const auto t_start = Clock::now();
  Options options;
  std::string error;
  if (!parseOptions(argc, argv, &options, &error)) {
    return emitFailure("validation", error, 2);
  }
  SceneInput scene;
  if (!parseScene(std::cin, &scene, &error)) {
    return emitFailure("validation", error, 2);
  }

  rclcpp::init(0, nullptr);
  int exit_code = 1;
  {
    // 参数: 项目 YAML 为准。vmax 只覆盖 max_velocity(硬上限); 巡航速度 reference_speed 仍取 YAML,
    // 规划器内部按 min(reference_speed, max_velocity) 使用, 不需要在这里改它。
    std::vector<std::string> args = {"--ros-args", "--params-file", options.params_file};
    const auto add_param = [&args](const std::string & name, const std::string & literal) {
        args.push_back("-p");
        args.push_back(name + ":=" + literal);
      };
    // 打开引导线话题, 让沙盒能看到 JPS 原始路径和中间引导; 话题在本进程内由探针节点接收。
    add_param("preprocessed_guide_topic", "/minco/preprocessed_guide");
    add_param("esdf_refined_guide_topic", "/minco/esdf_refined_guide");
    if (scene.has_vmax) {
      add_param("max_velocity", realLiteral(scene.vmax));
    }
    for (const auto & item : options.overrides) {
      const auto eq = item.find('=');
      add_param(item.substr(0, eq), item.substr(eq + 1));
    }
    rclcpp::NodeOptions node_options;
    node_options.arguments(args);

    std::shared_ptr<MincoPlannerNode> node;
    try {
      node = std::make_shared<MincoPlannerNode>(node_options);
    } catch (const std::exception & exception) {
      rclcpp::shutdown();
      return emitFailure("backend", std::string("cannot construct MincoPlannerNode: ") +
               exception.what(), 1);
    }
    const double build_ms = msSince(t_start);

    // 探针节点: 同进程内订阅规划器发布的路径话题, 只用来取 JPS 原始路径和中间引导。
    auto probe = std::make_shared<rclcpp::Node>("minco_sandbox_probe");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(probe);
    PathCapture raw_capture;
    PathCapture preprocessed_capture;
    PathCapture refined_capture;
    const auto subscribe = [&probe](const std::string & topic, PathCapture * capture) {
        return probe->create_subscription<nav_msgs::msg::Path>(
          topic, rclcpp::QoS(10).reliable(),
          [capture](nav_msgs::msg::Path::SharedPtr message) {
            capture->received = true;
            capture->points.clear();
            for (const auto & pose : message->poses) {
              capture->points.push_back({pose.pose.position.x, pose.pose.position.y});
            }
          });
      };
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr preprocessed_sub;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr refined_sub;
    if (node->raw_path_pub_) {
      raw_sub = subscribe(node->raw_path_topic_, &raw_capture);
    }
    if (node->preprocessed_guide_pub_) {
      preprocessed_sub = subscribe(node->preprocessed_guide_topic_, &preprocessed_capture);
    }
    if (node->esdf_refined_guide_pub_) {
      refined_sub = subscribe(node->esdf_refined_guide_topic_, &refined_capture);
    }

    // 喂地图: 栅格的 frame 取节点的 global_frame_, 起点/目标同 frame, 因此全程不需要 TF。
    const std::string frame = node->global_frame_;
    const auto t_grid = Clock::now();
    auto grid = std::make_shared<nav_msgs::msg::OccupancyGrid>(
      rasterize(scene, options.resolution, frame, node->now()));
    const int grid_width = static_cast<int>(grid->info.width);
    const int grid_height = static_cast<int>(grid->info.height);
    node->onGrid(grid);
    if (!node->map_ready_topic_.empty()) {
      // 生产里由 adapter 的 ready 心跳续租; 这里补一拍, 提交门要求租约新鲜。
      auto ready = std::make_shared<std_msgs::msg::Bool>();
      ready->data = true;
      node->onMapReady(ready);
    }
    const double grid_ms = msSince(t_grid);

    // 等探针与规划器的发布者互相匹配(同一 context 内通常立即完成), 避免丢第一条消息。
    const auto match_deadline = Clock::now() + std::chrono::milliseconds(1000);
    while (Clock::now() < match_deadline) {
      executor.spin_some();
      const bool raw_ok = !node->raw_path_pub_ || node->raw_path_pub_->get_subscription_count() > 0;
      const bool pre_ok = !node->preprocessed_guide_pub_ ||
        node->preprocessed_guide_pub_->get_subscription_count() > 0;
      const bool ref_ok = !node->esdf_refined_guide_pub_ ||
        node->esdf_refined_guide_pub_->get_subscription_count() > 0;
      if (raw_ok && pre_ok && ref_ok) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const auto goal = makePose(frame, scene.gx, scene.gy, scene.gyaw);
    const auto start = makePose(frame, scene.sx, scene.sy, scene.syaw);
    minco_planner::InitialKinematicState initial_state;
    const bool seeded = scene.svx != 0.0 || scene.svy != 0.0 || scene.sax != 0.0 ||
      scene.say != 0.0;
    if (seeded) {
      initial_state.valid = true;
      initial_state.velocity = Eigen::Vector2d(scene.svx, scene.svy);
      initial_state.acceleration = Eigen::Vector2d(scene.sax, scene.say);
    }

    const auto t_plan = Clock::now();
    node->planGoal(
      goal, /*goal_id=*/1, /*localization_epoch=*/1, /*plan_request_sequence=*/1,
      /*map_publication_sequence=*/1, /*report_status=*/true, &start,
      seeded ? &initial_state : nullptr);
    const double plan_ms = msSince(t_plan);

    // 收探针消息: 期望的话题都到齐或 250 ms 超时即止(失败时部分话题本就不会发布)。
    const auto collect_deadline = Clock::now() + std::chrono::milliseconds(250);
    while (Clock::now() < collect_deadline) {
      executor.spin_some();
      if ((!raw_sub || raw_capture.received) &&
        (!preprocessed_sub || preprocessed_capture.received) &&
        (!refined_sub || refined_capture.received))
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    minco_planner::ReferenceTrajectory reference;
    bool committed = false;
    std::optional<PlannerStatusMsg> status;
    std::uint64_t generation = 0;
    {
      std::lock_guard<std::mutex> lock(node->map_mutex_);
      committed = node->active_safety_reference_.has_value() && node->safety_state_.plan_safe;
      if (committed) {
        reference = node->active_safety_reference_->trajectory;
      }
      status = node->planner_status_state_.latest;
      generation = node->latest_map_snapshot_ ? node->latest_map_snapshot_->generation : 0;
    }

    std::ostringstream out;
    out << "{";
    out << "\"success\":" << (committed && reference.valid() ? "true" : "false");
    if (committed && reference.valid()) {
      double peak_speed = 0.0;
      double peak_acc = 0.0;
      for (const auto & p : reference.points) {
        peak_speed = std::max(peak_speed, std::hypot(p.vx, p.vy));
        peak_acc = std::max(peak_acc, std::hypot(p.ax, p.ay));
      }
      out << ",\"stage\":\"ok\"";
      std::ostringstream message;
      message << "committed by the planner node: " << reference.points.size() <<
        " reference points, " << std::fixed << std::setprecision(2) << reference.totalLength() <<
        " m";
      out << ",\"message\":" << jsonString(message.str());
      out << ",\"duration\":" << jsonNumber(reference.totalTime());
      out << ",\"length\":" << jsonNumber(reference.totalLength());
      out << ",\"peak_speed\":" << jsonNumber(peak_speed);
      out << ",\"peak_acc\":" << jsonNumber(peak_acc);
      out << ",\"samples\":[";
      for (std::size_t i = 0; i < reference.points.size(); ++i) {
        const auto & p = reference.points[i];
        out << (i ? "," : "") << "[" << jsonNumber(p.t) << "," << jsonNumber(p.x) << "," <<
          jsonNumber(p.y) << "," << jsonNumber(p.vx) << "," << jsonNumber(p.vy) << "," <<
          jsonNumber(p.ax) << "," << jsonNumber(p.ay) << "]";
      }
      out << "]";
      out << ",\"yaw\":[";
      for (std::size_t i = 0; i < reference.points.size(); ++i) {
        out << (i ? "," : "") << jsonNumber(reference.points[i].yaw);
      }
      out << "],\"yaw_rate\":[";
      for (std::size_t i = 0; i < reference.points.size(); ++i) {
        out << (i ? "," : "") << jsonNumber(reference.points[i].yaw_rate);
      }
      out << "]";
      exit_code = 0;
    } else {
      const std::uint8_t reason = status ? status->failure_reason : PlannerStatusMsg::FAILURE_NONE;
      out << ",\"stage\":" << jsonString(failureStage(reason));
      out << ",\"message\":" << jsonString(
        std::string("planner node rejected the request: ") + failureName(reason));
      out << ",\"samples\":[]";
    }
    if (status) {
      out << ",\"failure_code\":" << static_cast<int>(status->failure_reason);
      out << ",\"failure_reason\":" << jsonString(failureName(status->failure_reason));
      out << ",\"status_state\":" << static_cast<int>(status->state);
    }
    out << ",\"seed_path\":" << pointsJson(raw_capture.points);
    out << ",\"preprocessed_guide\":" << pointsJson(preprocessed_capture.points);
    out << ",\"esdf_refined_guide\":" << pointsJson(refined_capture.points);
    out << ",\"footprint\":{\"length\":" << jsonNumber(node->footprint_length_) <<
      ",\"width\":" << jsonNumber(node->footprint_width_) <<
      ",\"margin\":" << jsonNumber(node->footprint_safety_margin_) << "}";
    out << ",\"grid\":{\"resolution\":" << jsonNumber(options.resolution) <<
      ",\"width\":" << grid_width << ",\"height\":" << grid_height <<
      ",\"frame\":" << jsonString(frame) << ",\"generation\":" << generation << "}";
    out << ",\"timing_ms\":{\"node_construct\":" << jsonNumber(build_ms) <<
      ",\"grid_and_esdf\":" << jsonNumber(grid_ms) << ",\"plan_goal\":" << jsonNumber(plan_ms) <<
      ",\"total\":" << jsonNumber(msSince(t_start)) << "}";
    out << "}";
    std::cout << out.str() << std::endl;
  }
  rclcpp::shutdown();
  return exit_code;
}
