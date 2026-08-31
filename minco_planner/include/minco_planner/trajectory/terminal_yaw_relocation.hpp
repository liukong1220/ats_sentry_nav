// Copyright 2026

#ifndef MINCO_PLANNER__TERMINAL_YAW_RELOCATION_HPP_
#define MINCO_PLANNER__TERMINAL_YAW_RELOCATION_HPP_

#include <cstddef>
#include <vector>

#include "minco_planner/safety/footprint_safety_checker.hpp"
#include "minco_planner/trajectory/reference_trajectory.hpp"

namespace minco_planner
{

/// 终端 yaw 转向重定位。
///
/// 背景:yaw 规划在窄通道模式下会保留通道切线 yaw,再在终点位置追加一段原地
/// 转向把车头拧到 goal_yaw。这段追加没有任何 footprint 感知,方向只按最短角
/// 距离选,所以当目标点到墙的净空介于内切半宽与全 yaw 外接圆半径之间时,原地
/// 转向必然扫过不可行的 yaw 带,矩形足迹门禁正确拒绝,规划器随后每 2 s 复现
/// 同一条被拒轨迹直到超时(domain 171/173 目标 8 `east_mid` 实测形态)。
///
/// 处理方式不是放宽门禁,而是换一个转向位置:在路径上更早的某个采样点原地把
/// yaw 拧到 goal_yaw,之后全程保持 goal_yaw 平移进入目标。rotation_index 取
/// 尾段起点时结果与现状完全一致,所以这一族候选是现状的超集,由调用方逐个过
/// footprint 门禁,只接受安全的那个。
struct TerminalYawRelocationParams
{
  /// 与 YawSplinePlannerParams::yaw_rate_limit 同义,原地转向的角速度上界。
  double yaw_rate_limit = 2.5;
  /// 与 YawSplinePlannerParams::terminal_yaw_sample_period 同义。
  double sample_period = 0.10;
  /// 终端近域窗口弧长(m)。冲突全部落在末点前这段弧长内时才允许重定位。
  /// 只看"与末点重合的原地转向段"太窄:实测(domain 189 目标 9)冲突是最后
  /// 6 个采样点,其中前几个仍在平移,坐标与末点差 0.014 m,于是旧判据直接
  /// 返回 false,重定位一次都没试过。而这些点的 yaw 正是通道切线 yaw
  /// (-0.098 rad),矩形在该 yaw 下的前向外扩 e(yaw)=0.345 m 比 e(0)=0.320 m
  /// 多出 0.025 m,恰好把左前角推过墙面 1 mm。提前转向到 goal_yaw 再平移
  /// 进目标正是这一族候选要做的事,所以问题只在触发条件太窄。
  double terminal_window_length = 1.0;
};

/// 返回与轨迹末点坐标重合的尾段起始下标。
/// 没有重合尾段时返回 points.size() - 1,即末点自身;空轨迹返回 0。
std::size_t terminalCoincidentTailStart(const ReferenceTrajectory & trajectory);

/// 返回"距末点弧长不超过 window_length"的最早采样下标。
/// window_length <= 0 时返回末点自身;整条轨迹都在窗口内时返回 0。
/// 弧长按相邻采样点的欧氏距离累计,与 ReferencePoint::s 的语义无关,
/// 因此终端原地转向段(坐标重合、s 不变)不消耗窗口预算。
std::size_t terminalApproachWindowStart(
  const ReferenceTrajectory & trajectory,
  double window_length);

/// 判断所有冲突采样是否都落在"终端近域窗口"内。窗口起点为
/// terminalApproachWindowStart() 与终端重合尾段起点两者的较小值,因此这一判据
/// 是 collisionsConfinedToTerminalRotation() 的超集,并且仍然把中途不可行交回
/// 既有拒绝路径:窗口之外只要有一个冲突就返回 false。
bool collisionsConfinedToTerminalApproach(
  const ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & safety,
  double window_length);

/// 判断所有冲突采样是否都落在终端原地转向段内。
/// 只有这种形态才适用重定位;冲突在中途说明是路径本身不可行,应交给既有的
/// 拒绝路径,不要在这里改 yaw 掩盖问题。空冲突集返回 false(无需重定位)。
bool collisionsConfinedToTerminalRotation(
  const ReferenceTrajectory & trajectory,
  const FootprintSafetyResult & safety);

/// 生成"在 rotation_index 处原地转到 goal_yaw,之后保持 goal_yaw"的候选轨迹。
///
/// rotation_index 必须落在 [0, terminalCoincidentTailStart()] 内。原轨迹在该
/// 下标之前的 yaw 计划保持不变;该下标处插入零平移的原地转向采样;之后到尾段
/// 起点的所有点 yaw 被改写为 goal_yaw,原有的终端转向尾段被丢弃。
/// 时间轴整体后移一个转向时长,保证 ReferenceTrajectory::valid() 仍然成立。
/// 返回 false 表示无法构造(输入非法或结果不满足 valid())。
bool relocateTerminalYawRotation(
  const ReferenceTrajectory & input,
  double goal_yaw,
  std::size_t rotation_index,
  const TerminalYawRelocationParams & params,
  ReferenceTrajectory * output);

/// 按候选上限给出一组待试的 rotation_index,从最早(0)到最晚(尾段起点)。
/// 最早优先:越早转向,后续平移越接近纯 goal_yaw 姿态,扫掠体积也越可预测。
std::vector<std::size_t> terminalYawRelocationCandidates(
  const ReferenceTrajectory & trajectory,
  std::size_t max_candidates);

}  // namespace minco_planner

#endif  // MINCO_PLANNER__TERMINAL_YAW_RELOCATION_HPP_
