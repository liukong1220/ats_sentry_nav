/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#include <rog_map/prob_map.h>

#include <algorithm>
#include <limits>
using namespace rog_map;
using namespace super_utils;

// TAG 概率地图初始化
void ProbMap::initProbMap() {
    static bool init_once{false};
    if (init_once) {
        throw std::runtime_error(" -- [ROGMap] ProbMap can only init once.");
    }
    init_once = true;
    // 初始化 ProbMap 自己这层滑动地图
    initSlidingMap(cfg_.half_map_size_i, cfg_.resolution,
                   cfg_.map_sliding_en, cfg_.map_sliding_thresh,
                   cfg_.fix_map_origin);
    // 性能统计
    time_consuming_.resize(7);
    // 创建膨胀地图 根据概率地图的cell状态维护 膨胀占据栅格区域 用于规划器查询
    inf_map_ = std::make_shared<InfMap>(cfg_);

    // 是否启用 frontier extraction
    if (cfg_.frontier_extraction_en) {
        fcnt_map_ = std::make_shared<FreeCntMap>(cfg_.half_map_size_i + Vec3i::Constant(2),
                                                 cfg_.resolution,
                                                 cfg_.map_sliding_en,
                                                 cfg_.map_sliding_thresh,
                                                 cfg_.fix_map_origin);
    }

    // 启用ESDF 创建并初始化ESDFMap
    if (cfg_.esdf_en) {
        esdf_map_ = std::make_shared<ESDFMap>();
        esdf_map_->initESDFMap(cfg_.half_map_size_i,
                               cfg_.resolution,
                               cfg_.esdf_resolution,
                               cfg_.esdf_local_update_box,
                               cfg_.map_sliding_en,
                               cfg_.map_sliding_thresh,
                               cfg_.fix_map_origin,
                               cfg_.unk_thresh);
    }

    // 可视化范围
    posToGlobalIndex(cfg_.visualization_range, sc_.visualization_range_i);
    posToGlobalIndex(cfg_.virtual_ceil_height, sc_.virtual_ceil_height_id_g);
    posToGlobalIndex(cfg_.virtual_ground_height, sc_.virtual_ground_height_id_g);

    // 高度对齐到栅格分辨率上
    cfg_.virtual_ceil_height = sc_.virtual_ceil_height_id_g * cfg_.resolution;
    cfg_.virtual_ground_height = sc_.virtual_ground_height_id_g * cfg_.resolution;

    cout<<"[ProbMap] virtual_ceil_height: "<<cfg_.virtual_ceil_height<<endl;
    cout<< "[ProbMap] virtual_ground_height: "<<cfg_.virtual_ground_height<<endl;

    // 关闭滑动地图
    if (!cfg_.map_sliding_en) {
        std::cout << YELLOW << " -- [ProbMap] Map sliding disabled, set origin to [" << cfg_.fix_map_origin.transpose()
            << "] -- " << RESET << std::endl;
        slideAllMap(cfg_.fix_map_origin);
    }

    // 分配核心内存
    int map_size = sc_.map_size_i.prod();


    occupancy_buffer_.resize(map_size, 0);
    cell_last_observed_update_index_.resize(map_size, 0);
    cell_last_update_index_.resize(map_size, 0);
    cell_last_hit_update_index_.resize(map_size, 0);
    cell_last_miss_update_index_.resize(map_size, 0);
    cell_last_hit_count_.resize(map_size, 0);
    cell_last_miss_count_.resize(map_size, 0);
    occupied_cell_flags_.resize(map_size, 0U);
    occupied_cell_listed_flags_.resize(map_size, 0U);
    occupied_cell_hash_ids_.clear();
    raycast_data_.raycaster.setResolution(cfg_.resolution);
    raycast_data_.operation_cnt.resize(map_size, 0);
    raycast_data_.hit_cnt.resize(map_size, 0);

    // 整个地图 重置成 unknown
    resetLocalMap();

    std::cout << GREEN << " -- [ProbMap] Init successfully -- ." << RESET << std::endl;
    printMapInformation();
}

Vec3f ProbMap::getLocalMapOrigin() const {
    return local_map_origin_d_;
}

Vec3f ProbMap::getLocalMapSize() const {
    return cfg_.map_size_d;
}

void ProbMap::getRaycastLocalUpdateBox(Vec3f & box_min, Vec3f & box_max) const {
    std::lock_guard<std::mutex> lock(raycast_data_.raycast_range_mtx);
    box_min = raycast_data_.local_update_box_min;
    box_max = raycast_data_.local_update_box_max;
}

// Query====================================================
bool ProbMap::isOccupied(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return false;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return true;
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isUnknown(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return true;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }

    return isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isKnownFree(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return false;
    }
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromPos(pos)]);
}

bool ProbMap::isFrontier(const Vec3f& pos) const {
    // 1) Check local map
    if (!insideLocalMap(pos)) {
        return false;
    }

    // 2) Check virtual ceil and ground
    if (pos.z() > cfg_.virtual_ceil_height ||
        pos.z() < cfg_.virtual_ground_height) {
        return false;
    }

    // 3) Check frontier
    if (isUnknown(occupancy_buffer_[getHashIndexFromPos(pos)])) {
        if (fcnt_map_->getFreeCnt(pos)) {
            return true;
        }
    }
    return false;
}

// ======================================================
bool ProbMap::isOccupied(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
        return true;
    }
    return isOccupied(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isUnknown(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return true;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
        return false;
    }
    return isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isKnownFree(const Vec3i& id_g) const {
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
        return true;
    }
    return isKnownFree(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)]);
}

bool ProbMap::isFrontier(const Vec3i& id_g) const {
    // 1) Check local map
    if (!insideLocalMap(id_g)) {
        return false;
    }
    if (id_g.z() > sc_.virtual_ceil_height_id_g ||
        id_g.z() < sc_.virtual_ground_height_id_g + sc_.safe_margin_i) {
        return false;
    }

    if (isUnknown(occupancy_buffer_[getHashIndexFromGlobalIndex(id_g)])) {
        if (fcnt_map_->getFreeCnt(id_g) > 0) {
            return true;
        }
    }
    return false;
}

bool ProbMap::isKnownFreeInflate(const Vec3f& pos) const {
    return inf_map_->isKnownFreeInflate(pos);
}


bool ProbMap::isOccupiedInflate(const Vec3f& pos) const {
    return inf_map_->isOccupiedInflate(pos);
}

bool ProbMap::isUnknownInflate(const Vec3f& pos) const {
    return inf_map_->isUnknownInflate(pos);
}

// Query====================================================


void ProbMap::writeTimeConsumingToLog(std::ofstream& log_file) {
    for (long unsigned int i = 0; i < time_consuming_.size(); i++) {
        log_file << time_consuming_[i];
        if (i != time_consuming_.size() - 1)
            log_file << ", ";
    }
    log_file << std::endl;
}

void ProbMap::writeMapInfoToLog(std::ofstream& log_file) {
    log_file << "[ProbMap]" << std::endl;
    log_file << "\tmap_size_d: " << cfg_.map_size_d.transpose() << std::endl;
    log_file << "\tresolution: " << cfg_.resolution << std::endl;
    log_file << "\tmap_size_i: " << sc_.map_size_i.transpose() << std::endl;
    log_file << "\tlocal_update_box_size: " << cfg_.local_update_box_d.transpose() << std::endl;
    log_file << "\tp_min: " << cfg_.p_min << std::endl;
    log_file << "\tp_max: " << cfg_.p_max << std::endl;
    log_file << "\tp_hit: " << cfg_.p_hit << std::endl;
    log_file << "\tp_miss: " << cfg_.p_miss << std::endl;
    log_file << "\tp_occ: " << cfg_.p_occ << std::endl;
    log_file << "\tp_free: " << cfg_.p_free << std::endl;
    log_file << "\tunk_thresh: " << cfg_.unk_thresh << std::endl;
    log_file << "\tmap_sliding_thresh: " << cfg_.map_sliding_thresh << std::endl;
    log_file << "\tmap_sliding_en: " << cfg_.map_sliding_en << std::endl;
    log_file << "\tfix_map_origin: " << cfg_.fix_map_origin.transpose() << std::endl;
    log_file << "\tvisualization_range: " << cfg_.visualization_range.transpose() << std::endl;
    log_file << "\tvirtual_ceil_height: " << cfg_.virtual_ceil_height << std::endl;
    log_file << "\tvirtual_ground_height: " << cfg_.virtual_ground_height << std::endl;
    log_file << "\tbatch_update_size: " << cfg_.batch_update_size << std::endl;
    log_file << "\tstale_decay_en: " << cfg_.stale_decay_en << std::endl;
    log_file << "\tstale_soft_ttl_updates: " << cfg_.stale_soft_ttl_updates << std::endl;
    log_file << "\tstale_hard_ttl_updates: " << cfg_.stale_hard_ttl_updates << std::endl;
    log_file << "\tstale_decay_log_odds_step: " << cfg_.stale_decay_log_odds_step << std::endl;
    log_file << "\tstale_sweep_interval_updates: " << cfg_.stale_sweep_interval_updates << std::endl;
    log_file << "\tfrontier_extraction_en: " << cfg_.frontier_extraction_en << std::endl;
    inf_map_->writeMapInfoToLog(log_file);
}

void ProbMap::updateOccPointCloud(const PointCloud& input_cloud) {
    /// Step 1; Raycast and add to update cache.
    const int cloud_in_size = input_cloud.size();
    Vec3f localmap_min = local_map_bound_min_d_;
    Vec3f localmap_max = local_map_bound_max_d_;
    for (int i = 0; i < cloud_in_size; i++) {
        static Vec3f p, ray_pt;
        static Vec3i pt_id_g, pt_id_l;
        if (cfg_.intensity_thresh > 0 &&
            input_cloud[i].intensity < cfg_.intensity_thresh) {
            continue;
        }

        p.x() = input_cloud[i].x;
        p.y() = input_cloud[i].y;
        p.z() = input_cloud[i].z;

        posToGlobalIndex(p, pt_id_g);

        if (p.z() > cfg_.virtual_ceil_height || p.z() < cfg_.virtual_ground_height) {
            continue;
        }
        if (insideLocalMap(pt_id_g)) {
            const int occ_hit_num = ceil(cfg_.l_occ / cfg_.l_hit);
            for (int j = 0; j < occ_hit_num; j++) {
                insertUpdateCandidate(pt_id_g, true);
            }
            localmap_max = localmap_max.cwiseMax(p);
            localmap_min = localmap_min.cwiseMin(p);
        }
    }
    if(cfg_.map_sliding_en) {
        local_map_bound_max_d_ = localmap_max;
        local_map_bound_min_d_ = localmap_min;
    }
    probabilisticMapFromCache();
    map_empty_ = false;
}

void ProbMap::slideAllMap(const rog_map::Vec3f& pos) {
    mapSliding(pos);
    inf_map_->mapSliding(pos);
    if (cfg_.frontier_extraction_en) {
        fcnt_map_->mapSliding(pos);
    }
    if (cfg_.esdf_en) {
        esdf_map_->mapSliding(pos);
    }
}

// TAG 地图更新入口
void ProbMap::updateProbMap(
    const PointCloud& cloud, const Pose& sensor_pose, const Vec3f& map_center,
    bool update_esdf) {
    TimeConsuming tc("updateMap", false);
    const Vec3f& sensor_pos = sensor_pose.first;
    time_consuming_[4] = cloud.size();
    // insideLocalMap 机器人当前位置已经抛出当前局部地图的窗口了  并且没有要处理的点云
    if (cfg_.map_sliding_en && !insideLocalMap(map_center) && raycast_data_.batch_update_counter == 0) {

        const double recenter_jump = (map_center - local_map_origin_d_).norm();
        if (cfg_.map_sliding_max_recenter_jump > 0.0 &&
            recenter_jump > cfg_.map_sliding_max_recenter_jump) {
            std::cout << RED
                << " -- [ROGMapCore] cur_pose jump " << recenter_jump
                << " m exceeds max_recenter_jump "
                << cfg_.map_sliding_max_recenter_jump
                << "; refuse map reset (fail-closed)." << RESET << std::endl;
            return;
        }
        std::cout << YELLOW << " -- [ROGMapCore] cur_pose out of map range, reset the map." << RESET << std::endl;
        std::cout << YELLOW << " -- [ROGMapCore] Sliding to map center at: " << map_center.transpose() << RESET << std::endl;
        slideAllMap(map_center);
        // Keep diagnostics and the next raycast decision aligned with the new
        // sliding window even though this frame has no map update yet.
        updateLocalBox(map_center);
        return;
    }

    if (sensor_pos.z() > cfg_.virtual_ceil_height) {
        std::cout << YELLOW << " -- [ROGMapCore] Sensor z " << sensor_pos.z()
            << " is above virtual ceil " << cfg_.virtual_ceil_height << "." << RESET
            << std::endl;
        return;
    }
    else if (sensor_pos.z() < cfg_.virtual_ground_height) {
        std::cout << YELLOW << " -- [ROGMapCore] Sensor z " << sensor_pos.z()
            << " is below virtual ground " << cfg_.virtual_ground_height << "." << RESET
            << std::endl;
        return;
    }

    // 当前没有需要更新的数据时 才允许滑动
    // TAG 是否滑动地图判断位置
    if (raycast_data_.batch_update_counter == 0 &&
        cfg_.map_sliding_en  &&
        // map_empty_ 第一次需要滑动 当前位置与地图原点超过阈值
        (map_empty_ || (map_center - local_map_origin_d_).norm() > cfg_.map_sliding_thresh)
        ) {
        // 执行滑动
        slideAllMap(map_center);
    }

    // 根据车的位置 更新局部更新框
    updateLocalBox(map_center);
    TimeConsuming t_raycast("raycast", false);
    // Raycast 处理 缓存hit和miss
    raycastProcess(cloud, sensor_pos);
    time_consuming_[1] = t_raycast.stop();
    // 累积多少次 raycast结果 ???
    // HACK 订阅到一帧 点云 累加一次 但是在这个函数中处理
    raycast_data_.batch_update_counter++;
    // 批量更新 检查是否攒够了做够多的帧数
    if (raycast_data_.batch_update_counter >= cfg_.batch_update_size) {
        raycast_data_.batch_update_counter = 0;
        time_consuming_[5] = raycast_data_.update_cache_id_g.size();
        TimeConsuming t_update("update", false);
        // TAG 概率更新
        probabilisticMapFromCache();
        time_consuming_[2] = t_update.stop();
        map_empty_ = false;
    }
    inf_map_->getInflationNumAndTime(time_consuming_[6], time_consuming_[3]);
    time_consuming_[0] = tc.stop();

    /* Update ESDF map */
    if (update_esdf && cfg_.esdf_en &&
        map_update_index_ % static_cast<std::uint64_t>(cfg_.esdf_update_interval_updates) == 0) {
        esdf_map_->updateESDF3D(map_center);
    }

    /* For the first frame, clear all unknown around the robot */
    static bool first = true;
    if (first) {
        first = false;
        for (double dx = -cfg_.raycast_range_min; dx <= cfg_.raycast_range_min; dx += cfg_.resolution) {
            for (double dy = -cfg_.raycast_range_min; dy <= cfg_.raycast_range_min; dy += cfg_.resolution) {
                for (double dz = -cfg_.raycast_range_min; dz <= cfg_.raycast_range_min; dz += cfg_.resolution) {
                    Vec3f p(dx, dy, dz);
                    if (p.norm() <= cfg_.raycast_range_min) {
                        Vec3f pp = sensor_pos + p;
                        int hash_id = getHashIndexFromPos(pp);
                        missPointUpdate(pp, hash_id, 999);
                    }
                }
            }
        }
    }
}

GridType ProbMap::getGridType(Vec3i& id_g) const {
    if (id_g.z() <= sc_.virtual_ground_height_id_g ||
        id_g.z() >= sc_.virtual_ceil_height_id_g - sc_.safe_margin_i) {
        return super_utils::OCCUPIED;
    }
    if (!insideLocalMap(id_g)) {
        return super_utils::OUT_OF_MAP;
    }
    Vec3i id_l;
    globalIndexToLocalIndex(id_g, id_l);
    int hash_id = getLocalIndexHash(id_l);
    double ret = occupancy_buffer_[hash_id];
    if (isKnownFree(ret)) {
        return GridType::KNOWN_FREE;
    }
    else if (isOccupied(ret)) {
        return GridType::OCCUPIED;
    }
    else {
        return GridType::UNKNOWN;
    }
}

GridType ProbMap::getGridType(const Vec3f& pos) const {
    if (pos.z() <= cfg_.virtual_ground_height ||
        pos.z() >= cfg_.virtual_ceil_height) {
        return OCCUPIED;
    }
    if (!insideLocalMap(pos)) {
        return OUT_OF_MAP;
    }
    Vec3i id_g, id_l;
    posToGlobalIndex(pos, id_g);
    return getGridType(id_g);
}

GridType ProbMap::getInfGridType(const Vec3f& pos) const {
    // NOTE, we consider, if the pos is not inside prob map, it is also out of inf map.
    if(!insideLocalMap(pos)) {
        return OUT_OF_MAP;
    }
    return inf_map_->getGridType(pos);
}

double ProbMap::getMapValue(const Vec3f& pos) const {
    if (!insideLocalMap(pos)) {
        return 0;
    }
    return occupancy_buffer_[getHashIndexFromPos(pos)];
}

void ProbMap::collectLocalVoxelDebug(std::vector<VoxelDebugCell>& cells,   // 输出 位置 状态 hit/miss 等信息
                                     VoxelDebugStats& stats,                // 输出统计量
                                     int stride,                            // 采样步长
                                     const bool include_unknown) const {    // 是否要将 unknown也放进 cells
    collectVoxelDebugInBox(
        local_map_bound_min_d_, local_map_bound_max_d_,
        cells, stats, stride, include_unknown);
}

void ProbMap::collectVoxelDebugInBox(const Vec3f &box_min, const Vec3f &box_max,
                                     std::vector<VoxelDebugCell>& cells,
                                     VoxelDebugStats& stats,
                                     int stride,
                                     const bool include_unknown) const {
    cells.clear();
    stats = VoxelDebugStats{};
    stats.update_index = map_update_index_;
    stats.total_cells = static_cast<int>(occupancy_buffer_.size());

    if (stride <= 0) {
        stride = 1;
    }
    if (occupancy_buffer_.empty()) {
        return;
    }

    const std::uint64_t current_update = map_update_index_;
    const std::uint64_t soft_ttl =
        cfg_.stale_soft_ttl_updates > 0
            ? static_cast<std::uint64_t>(cfg_.stale_soft_ttl_updates)
            : 0;
    const std::uint64_t hard_ttl =
        cfg_.stale_hard_ttl_updates > 0
            ? static_cast<std::uint64_t>(cfg_.stale_hard_ttl_updates)
            : 0;

    if ((box_max - box_min).minCoeff() < 0.0F) {
        return;
    }

    Vec3i requested_min_id_g;
    Vec3i requested_max_id_g;
    posToGlobalIndex(box_min, requested_min_id_g);
    posToGlobalIndex(box_max, requested_max_id_g);
    const Vec3i scan_min_id_g = (requested_min_id_g - Vec3i::Ones()).cwiseMax(
        local_map_bound_min_i_);
    const Vec3i scan_max_id_g = (requested_max_id_g + Vec3i::Ones()).cwiseMin(
        local_map_bound_max_i_);
    if ((scan_max_id_g - scan_min_id_g).minCoeff() < 0) {
        return;
    }

    const auto sampled_axis_count = [stride](const int min_index, const int max_index) {
        return static_cast<std::size_t>(max_index - min_index) /
            static_cast<std::size_t>(stride) + 1U;
    };
    const auto saturated_multiply = [](const std::size_t left, const std::size_t right) {
        return left > std::numeric_limits<std::size_t>::max() / right
            ? std::numeric_limits<std::size_t>::max()
            : left * right;
    };
    const std::size_t sampled_cells = saturated_multiply(
        saturated_multiply(
            sampled_axis_count(scan_min_id_g.x(), scan_max_id_g.x()),
            sampled_axis_count(scan_min_id_g.y(), scan_max_id_g.y())),
        sampled_axis_count(scan_min_id_g.z(), scan_max_id_g.z()));
    // The export is bounded by the requested visualization box. Reserving that exact upper
    // bound keeps snapshot collection from repeatedly reallocating without copying the map.
    cells.reserve(std::min(sampled_cells, occupancy_buffer_.size()));

    for (int gx = scan_min_id_g.x(); gx <= scan_max_id_g.x(); gx += stride) {
        for (int gy = scan_min_id_g.y(); gy <= scan_max_id_g.y(); gy += stride) {
            for (int gz = scan_min_id_g.z(); gz <= scan_max_id_g.z(); gz += stride) {
                const Vec3i id_g(gx, gy, gz);
                Vec3f center;
                globalIndexToPos(id_g, center);
                if (
                    !insideLocalMap(id_g) ||
                    (center.array() < box_min.array()).any() ||
                    (center.array() > box_max.array()).any())
                {
                    continue;
                }
                Vec3i id_l;
                globalIndexToLocalIndex(id_g, id_l);
                const int hash_id = getLocalIndexHash(id_l);
                if (hash_id < 0 || hash_id >= static_cast<int>(occupancy_buffer_.size())) {
                    continue;
                }

                const float value = occupancy_buffer_[hash_id];
                GridType type = GridType::UNKNOWN;
                if (isOccupied(value)) {
                    type = GridType::OCCUPIED;
                    ++stats.occupied_cells;
                }
                else if (isKnownFree(value)) {
                    type = GridType::KNOWN_FREE;
                    ++stats.known_free_cells;
                }
                else {
                    ++stats.unknown_cells;
                    if (!include_unknown) {
                        continue;
                    }
                }

                VoxelDebugCell cell;
                cell.id_g = id_g;
                cell.center = center;
                cell.type = type;
                cell.log_odds = value;

                if (hash_id < static_cast<int>(cell_last_update_index_.size())) {
                    cell.last_update_index = cell_last_update_index_[hash_id];
                }
                if (hash_id < static_cast<int>(cell_last_hit_update_index_.size())) {
                    cell.last_hit_update_index = cell_last_hit_update_index_[hash_id];
                }
                if (hash_id < static_cast<int>(cell_last_miss_update_index_.size())) {
                    cell.last_miss_update_index = cell_last_miss_update_index_[hash_id];
                }
                if (hash_id < static_cast<int>(cell_last_observed_update_index_.size())) {
                    cell.last_observed_update_index = cell_last_observed_update_index_[hash_id];
                }
                if (hash_id < static_cast<int>(cell_last_hit_count_.size())) {
                    cell.last_hit_count = cell_last_hit_count_[hash_id];
                }
                if (hash_id < static_cast<int>(cell_last_miss_count_.size())) {
                    cell.last_miss_count = cell_last_miss_count_[hash_id];
                }
                if (hash_id < static_cast<int>(occupied_cell_flags_.size())) {
                    cell.tracked_occupied = occupied_cell_flags_[hash_id] != 0U;
                }

                cell.updated_this_frame =
                    cell.last_update_index != 0 && cell.last_update_index == current_update;
                cell.hit_this_frame =
                    cell.last_hit_update_index != 0 && cell.last_hit_update_index == current_update;
                cell.miss_this_frame =
                    cell.last_miss_update_index != 0 && cell.last_miss_update_index == current_update;
                cell.passed_by_ray = cell.last_miss_update_index != 0;
                cell.never_touched =
                    cell.last_update_index == 0 &&
                    cell.last_hit_update_index == 0 &&
                    cell.last_miss_update_index == 0;
                cell.age_updates =
                    current_update > cell.last_observed_update_index
                        ? current_update - cell.last_observed_update_index
                        : 0;
                cell.stale_soft_band =
                    type == GridType::OCCUPIED &&
                    soft_ttl > 0 &&
                    cell.age_updates >= soft_ttl &&
                    (hard_ttl == 0 || cell.age_updates < hard_ttl);
                cell.stale_hard_band =
                    type == GridType::OCCUPIED &&
                    hard_ttl > 0 &&
                    cell.age_updates >= hard_ttl;

                if (cell.updated_this_frame) {
                    ++stats.updated_this_frame_cells;
                }
                if (cell.hit_this_frame) {
                    ++stats.hit_this_frame_cells;
                }
                if (cell.miss_this_frame) {
                    ++stats.miss_this_frame_cells;
                }
                if (cell.hit_this_frame && cell.miss_this_frame) {
                    ++stats.hit_and_miss_this_frame_cells;
                }
                if (cell.passed_by_ray) {
                    ++stats.ray_passed_cells;
                }
                if (cell.never_touched) {
                    ++stats.never_touched_cells;
                }
                if (cell.stale_soft_band) {
                    ++stats.stale_soft_band_cells;
                }
                if (cell.stale_hard_band) {
                    ++stats.stale_hard_band_cells;
                }

                cells.push_back(cell);
            }
        }
    }

    stats.exported_cells = static_cast<int>(cells.size());
}

void
ProbMap::boxSearch(const Vec3f& _box_min, const Vec3f& _box_max, const GridType& gt, vec_E<Vec3f>& out_points) const {
    out_points.clear();
    if (map_empty_) {
        std::cout << YELLOW << " -- [ROG] Map is empty, cannot perform box search." << RESET << std::endl;
        return;
    }
    if ((_box_max - _box_min).minCoeff() <= 0) {
        std::cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << std::endl;
        return;
    }
    Vec3f box_min_d = _box_min, box_max_d = _box_max;
    boundBoxByLocalMap(box_min_d, box_max_d);
    if ((box_max_d - box_min_d).minCoeff() <= 0) {
        std::cout << YELLOW << " -- [ROG] Box search failed, box size is zero." << RESET << std::endl;
        return;
    }
    Vec3i box_min_id_g, box_max_id_g;
    posToGlobalIndex(box_min_d, box_min_id_g);
    posToGlobalIndex(box_max_d, box_max_id_g);
    Vec3i box_size = box_max_id_g - box_min_id_g;
    if (gt == UNKNOWN) {
        out_points.reserve(box_size.prod());
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isUnknown(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else if (gt == OCCUPIED) {
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isOccupied(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else if (gt == FRONTIER) {
        out_points.reserve(box_size.prod() / 3);
        for (int i = box_min_id_g.x() + 1; i < box_max_id_g.x(); i++) {
            for (int j = box_min_id_g.y() + 1; j < box_max_id_g.y(); j++) {
                for (int k = box_min_id_g.z() + 1; k < box_max_id_g.z(); k++) {
                    Vec3i id_g(i, j, k);
                    if (isFrontier(id_g)) {
                        Vec3f pos;
                        globalIndexToPos(id_g, pos);
                        out_points.push_back(pos);
                    }
                }
            }
        }
    }
    else {
        throw std::runtime_error(" -- [ROG-Map] Box search does not support KNOWN_FREE.");
    }
}

void ProbMap::boxSearchInflate(const Vec3f& box_min, const Vec3f& box_max, const GridType& gt,
                               vec_E<Vec3f>& out_points) const {
    inf_map_->boxSearch(box_min, box_max, gt, out_points);
}

void ProbMap::boundBoxByLocalMap(Vec3f& box_min, Vec3f& box_max) const {
    if ((box_max - box_min).minCoeff() <= 0) {
        box_min = box_max;
        std::cout << YELLOW << "-- [ROG] Bound box is invalid." << RESET << std::endl;
        return;
    }

    box_min = box_min.cwiseMax(local_map_bound_min_d_);
    box_max = box_max.cwiseMin(local_map_bound_max_d_);
    box_max.z() = std::min(box_max.z(), cfg_.virtual_ceil_height);
    box_min.z() = std::max(box_min.z(), cfg_.virtual_ground_height);
}

void ProbMap::resetCell(const int& hash_id) {
    float& ret = occupancy_buffer_[hash_id];
    if (isOccupied(ret)) {
        /// if current state is occupied
        Vec3f pos;
        hashIdToPos(hash_id, pos);
        inf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(pos, OCCUPIED, UNKNOWN);
        }
    }
    else if (isKnownFree(ret)) {
        /// if current state is free
        Vec3f pos;
        hashIdToPos(hash_id, pos);
        inf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(pos, KNOWN_FREE, UNKNOWN);
        }
        if (cfg_.frontier_extraction_en) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            if (cfg_.frontier_extraction_en) {
                fcnt_map_->updateFrontierCounter(id_g, false);
            }
        }
    }
    else {
        // nothing need to do
    }
    ret = 0;
    if (hash_id >= 0 && hash_id < static_cast<int>(cell_last_observed_update_index_.size())) {
        cell_last_observed_update_index_[hash_id] = 0;
        cell_last_update_index_[hash_id] = 0;
        cell_last_hit_update_index_[hash_id] = 0;
        cell_last_miss_update_index_[hash_id] = 0;
        cell_last_hit_count_[hash_id] = 0;
        cell_last_miss_count_[hash_id] = 0;
        occupied_cell_flags_[hash_id] = 0U;
    }
}

// TAG 概率更新
