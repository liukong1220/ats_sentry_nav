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
    const PointCloud& cloud, const Pose& sensor_pose, const Vec3f& map_center) {
    TimeConsuming tc("updateMap", false);
    const Vec3f& sensor_pos = sensor_pose.first;
    time_consuming_[4] = cloud.size();
    // insideLocalMap 机器人当前位置已经抛出当前局部地图的窗口了  并且没有要处理的点云
    if (cfg_.map_sliding_en && !insideLocalMap(map_center) && raycast_data_.batch_update_counter == 0) {
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
    if (cfg_.esdf_en &&
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

    for (int lx = -sc_.half_map_size_i.x(); lx <= sc_.half_map_size_i.x(); lx += stride) {
        for (int ly = -sc_.half_map_size_i.y(); ly <= sc_.half_map_size_i.y(); ly += stride) {
            for (int lz = -sc_.half_map_size_i.z(); lz <= sc_.half_map_size_i.z(); lz += stride) {
                const Vec3i id_l(lx, ly, lz);
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
                localIndexToGlobalIndex(id_l, cell.id_g);
                localIndexToPos(id_l, cell.center);
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
void ProbMap::probabilisticMapFromCache() {
    //    int addr = getHashIndexFromGlobalIndex(Vec3i(41,
    //                                                 -216,
    //                                                 -6));
    //    float ret = occupancy_buffer_[addr];
    //    std::cout << "ret: " << ret << std::endl;
    // 初始化
    // 记录这个更新多少的格子
    current_raycast_debug_stats_.update_cache_cells = static_cast<int>(raycast_data_.update_cache_id_g.size());
    last_raycast_debug_cells_.clear();
    last_stale_debug_cells_.clear();
    // 遍历缓存队列
    while (!raycast_data_.update_cache_id_g.empty()) {
        Vec3f pos;
        // 去除全局栅格索引
        Vec3i id_g = raycast_data_.update_cache_id_g.front();
        raycast_data_.update_cache_id_g.pop();
        Vec3i id_l;
        // 转成局部搜一年和一维数组下标
        globalIndexToLocalIndex(id_g, id_l);
        int hash_id = getLocalIndexHash(id_l);
        globalIndexToPos(id_g, pos);
        // 获取 hit_count 和 miss_count 的计数
        // raycast_data_.hit_cnt 在 raycastProcess 中更新

        // 端点命中次数
        const int hit_count = raycast_data_.hit_cnt[hash_id];
        // 射线穿过次数
        const int miss_count = raycast_data_.operation_cnt[hash_id] - hit_count;
        if (hash_id >= 0 && hash_id < static_cast<int>(cell_last_update_index_.size())) {
            // 记录信息
            cell_last_update_index_[hash_id] = map_update_index_;
            cell_last_hit_count_[hash_id] =
                static_cast<std::uint16_t>(
                    std::min(hit_count, static_cast<int>(std::numeric_limits<std::uint16_t>::max())));
            cell_last_miss_count_[hash_id] =
                static_cast<std::uint16_t>(
                    std::min(miss_count, static_cast<int>(std::numeric_limits<std::uint16_t>::max())));


            if (hit_count > 0) {
                cell_last_hit_update_index_[hash_id] = map_update_index_;
            }
            if (miss_count > 0) {
                cell_last_miss_update_index_[hash_id] = map_update_index_;
            }
        }
        // 记录更新前状态
        GridType from_type;
        if (isOccupied(occupancy_buffer_[hash_id])) {
            from_type = GridType::OCCUPIED;
        }
        else if (isKnownFree(occupancy_buffer_[hash_id])) {
            from_type = GridType::KNOWN_FREE;
        }
        else {
            from_type = GridType::UNKNOWN;
        }
        const float old_log_odds = occupancy_buffer_[hash_id];
        current_raycast_debug_stats_.hit_votes += hit_count;
        current_raycast_debug_stats_.miss_votes += miss_count;

        if (hit_count > 0 && miss_count > 0) {
            ++current_raycast_debug_stats_.hit_and_miss_cells;
        }
        else if (hit_count > 0) {
            ++current_raycast_debug_stats_.hit_only_cells;
        }
        else {
            ++current_raycast_debug_stats_.miss_only_cells;
        }

        // 概率更新
        if (from_type == GridType::OCCUPIED) {
            ++current_raycast_debug_stats_.occupied_touched_cells;
            if (hit_count > 0) {
                ++current_raycast_debug_stats_.occupied_hit_cells;
            }
            else {
                ++current_raycast_debug_stats_.occupied_miss_only_cells;
            }
        }
        if (hit_count > 0 && miss_count > 0) {
            mixedPointUpdate(pos, hash_id, hit_count, miss_count);
        }
        else if (hit_count > 0) {
            hitPointUpdate(pos, hash_id, hit_count);
        }
        else {
            missPointUpdate(pos, hash_id, miss_count);
        }
        if (hit_count > 0) {
            markObservedCell(hash_id);
        }
        GridType to_type;
        if (isOccupied(occupancy_buffer_[hash_id])) {
            to_type = GridType::OCCUPIED;
        }
        else if (isKnownFree(occupancy_buffer_[hash_id])) {
            to_type = GridType::KNOWN_FREE;
        }
        else {
            to_type = GridType::UNKNOWN;
        }
        RaycastDebugCell debug_cell;
        debug_cell.id_g = id_g;
        debug_cell.center = pos;
        debug_cell.old_log_odds = old_log_odds;
        debug_cell.new_log_odds = occupancy_buffer_[hash_id];
        debug_cell.hit_count = hit_count;
        debug_cell.miss_count = miss_count;
        debug_cell.from_type = from_type;
        debug_cell.to_type = to_type;
        last_raycast_debug_cells_.push_back(debug_cell);
        if (from_type == GridType::OCCUPIED && to_type == GridType::UNKNOWN) {
            ++current_raycast_debug_stats_.occupied_to_unknown_cells;
        }
        else if (from_type == GridType::OCCUPIED && to_type == GridType::KNOWN_FREE) {
            ++current_raycast_debug_stats_.occupied_to_free_cells;
        }
        else if (from_type == GridType::UNKNOWN && to_type == GridType::OCCUPIED) {
            ++current_raycast_debug_stats_.unknown_to_occupied_cells;
        }
        else if (from_type == GridType::UNKNOWN && to_type == GridType::KNOWN_FREE) {
            ++current_raycast_debug_stats_.unknown_to_free_cells;
        }
        else if (from_type == GridType::KNOWN_FREE && to_type == GridType::OCCUPIED) {
            ++current_raycast_debug_stats_.free_to_occupied_cells;
        }
        else if (from_type == GridType::KNOWN_FREE && to_type == GridType::UNKNOWN) {
            ++current_raycast_debug_stats_.free_to_unknown_cells;
        }
        // 检测状态跳变并触发子地图更新
        updateTrackedOccupiedCell(hash_id, to_type);
        // 清零缓存计数
        raycast_data_.hit_cnt[hash_id] = 0;
        raycast_data_.operation_cnt[hash_id] = 0;
    }
    // 遍历所有被跟踪的占据cell 对长期未被 hit 的 cell 执行衰减
    applyStaleDecay();
    last_raycast_debug_stats_ = current_raycast_debug_stats_;
}

float ProbMap::applyRaycastLogOddsUpdate(const float value,
                                         const float l_hit,
                                         const float l_miss,
                                         const float l_min,
                                         const float l_max,
                                         const int hit_count,
                                         const int miss_count) {
    float updated = value + l_hit * hit_count + l_miss * miss_count;
    if (updated > l_max) {
        updated = l_max;
    }
    if (updated < l_min) {
        updated = l_min;
    }
    return updated;
}

float ProbMap::applyStaleDecayUpdate(const float value,
                                     const std::uint64_t last_observed_update,
                                     const std::uint64_t current_update,
                                     const int soft_ttl_updates,
                                     const int hard_ttl_updates,
                                     const float decay_log_odds_step,
                                     const float unknown_log_odds,
                                     bool& hard_clear) {
    hard_clear = false;
    const std::uint64_t age =
        current_update > last_observed_update ? current_update - last_observed_update : 0;
    if (hard_ttl_updates > 0 &&
        age >= static_cast<std::uint64_t>(hard_ttl_updates)) {
        hard_clear = true;
        return unknown_log_odds;
    }
    if (age < static_cast<std::uint64_t>(soft_ttl_updates) ||
        decay_log_odds_step <= 0.0F) {
        return value;
    }
    const float updated = value - decay_log_odds_step;
    return updated < unknown_log_odds ? unknown_log_odds : updated;
}

void ProbMap::markObservedCell(const int& hash_id) {
    if (hash_id < 0 || hash_id >= static_cast<int>(cell_last_observed_update_index_.size())) {
        return;
    }
    cell_last_observed_update_index_[hash_id] = map_update_index_;
}

void ProbMap::updateTrackedOccupiedCell(const int& hash_id, const GridType to_type) {
    if (!cfg_.stale_decay_en ||
        hash_id < 0 ||
        hash_id >= static_cast<int>(occupied_cell_flags_.size()) ||
        hash_id >= static_cast<int>(occupied_cell_listed_flags_.size())) {
        return;
    }
    if (to_type == GridType::OCCUPIED) {
        occupied_cell_flags_[hash_id] = 1U;
        if (occupied_cell_listed_flags_[hash_id] == 0U) {
            occupied_cell_hash_ids_.push_back(hash_id);
            occupied_cell_listed_flags_[hash_id] = 1U;
        }
    }
    else {
        occupied_cell_flags_[hash_id] = 0U;
        cell_last_observed_update_index_[hash_id] = 0;
    }
}

void ProbMap::applyStaleDecay() {
    if (!cfg_.stale_decay_en || map_update_index_ == 0) {
        return;
    }
    if (cfg_.stale_sweep_interval_updates > 1 &&
        map_update_index_ % static_cast<std::uint64_t>(cfg_.stale_sweep_interval_updates) != 0) {
        return;
    }

    const std::uint64_t current_update = map_update_index_;
    const float unknown_value = unknownLogOdds();
    std::vector<int> still_occupied;
    still_occupied.reserve(occupied_cell_hash_ids_.size());

    for (const int hash_id : occupied_cell_hash_ids_) {
        if (hash_id < 0 ||
            hash_id >= static_cast<int>(occupancy_buffer_.size()) ||
            occupied_cell_flags_[hash_id] == 0U) {
            if (hash_id >= 0 && hash_id < static_cast<int>(occupied_cell_listed_flags_.size())) {
                occupied_cell_listed_flags_[hash_id] = 0U;
            }
            continue;
        }

        const float old_value = occupancy_buffer_[hash_id];
        if (!isOccupied(old_value)) {
            occupied_cell_flags_[hash_id] = 0U;
            occupied_cell_listed_flags_[hash_id] = 0U;
            cell_last_observed_update_index_[hash_id] = 0;
            continue;
        }

        if (cfg_.stale_decay_local_update_box_only) {
            Vec3i id_g;
            hashIdToGlobalIndex(hash_id, id_g);
            Vec3f center_pos;
            globalIndexToPos(id_g, center_pos);
            if (((center_pos - raycast_data_.local_update_box_min).minCoeff() < 0) ||
                ((center_pos - raycast_data_.local_update_box_max).maxCoeff() > 0)) {
                ++current_raycast_debug_stats_.stale_outside_update_box_cells;
                still_occupied.push_back(hash_id);
                occupied_cell_flags_[hash_id] = 1U;
                occupied_cell_listed_flags_[hash_id] = 1U;
                continue;
            }
        }

        const std::uint64_t last_observed = cell_last_observed_update_index_[hash_id];
        const std::uint64_t age =
            current_update > last_observed ? current_update - last_observed : 0;
        ++current_raycast_debug_stats_.stale_tracked_occupied_cells;
        current_raycast_debug_stats_.stale_max_age =
            std::max(current_raycast_debug_stats_.stale_max_age, age);
        if (age < static_cast<std::uint64_t>(cfg_.stale_soft_ttl_updates)) {
            ++current_raycast_debug_stats_.stale_fresh_occupied_cells;
        }
        else {
            ++current_raycast_debug_stats_.stale_occupied_cells;
            if (cfg_.stale_hard_ttl_updates > 0 &&
                age >= static_cast<std::uint64_t>(cfg_.stale_hard_ttl_updates)) {
                ++current_raycast_debug_stats_.stale_age_over_hard_cells;
            }
            else {
                ++current_raycast_debug_stats_.stale_age_soft_to_hard_cells;
            }
        }

        bool hard_clear = false;
        occupancy_buffer_[hash_id] = applyStaleDecayUpdate(old_value,
                                                           last_observed,
                                                           current_update,
                                                           cfg_.stale_soft_ttl_updates,
                                                           cfg_.stale_hard_ttl_updates,
                                                           cfg_.stale_decay_log_odds_step,
                                                           unknown_value,
                                                           hard_clear);
        if (hard_clear) {
            ++current_raycast_debug_stats_.stale_hard_cleared_cells;
        }
        else if (occupancy_buffer_[hash_id] != old_value) {
            ++current_raycast_debug_stats_.stale_decayed_cells;
        }
        if (hard_clear || occupancy_buffer_[hash_id] != old_value) {
            StaleDebugCell debug_cell;
            Vec3i id_g;
            hashIdToGlobalIndex(hash_id, id_g);
            debug_cell.id_g = id_g;
            globalIndexToPos(id_g, debug_cell.center);
            debug_cell.old_log_odds = old_value;
            debug_cell.new_log_odds = occupancy_buffer_[hash_id];
            debug_cell.age_updates = age;
            debug_cell.hard_cleared = hard_clear;
            debug_cell.from_type = GridType::OCCUPIED;
            if (isOccupied(occupancy_buffer_[hash_id])) {
                debug_cell.to_type = GridType::OCCUPIED;
            }
            else if (isKnownFree(occupancy_buffer_[hash_id])) {
                debug_cell.to_type = GridType::KNOWN_FREE;
            }
            else {
                debug_cell.to_type = GridType::UNKNOWN;
            }
            last_stale_debug_cells_.push_back(debug_cell);
        }
        if (isOccupied(occupancy_buffer_[hash_id])) {
            still_occupied.push_back(hash_id);
            occupied_cell_flags_[hash_id] = 1U;
            occupied_cell_listed_flags_[hash_id] = 1U;
            continue;
        }

        occupied_cell_flags_[hash_id] = 0U;
        occupied_cell_listed_flags_[hash_id] = 0U;
        cell_last_observed_update_index_[hash_id] = 0;
        ++current_raycast_debug_stats_.stale_to_unknown_cells;
        Vec3f center_pos;
        Vec3i id_g;
        hashIdToGlobalIndex(hash_id, id_g);
        globalIndexToPos(id_g, center_pos);
        inf_map_->updateGridCounter(center_pos, GridType::OCCUPIED, GridType::UNKNOWN);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, GridType::OCCUPIED, GridType::UNKNOWN);
        }
    }

    occupied_cell_hash_ids_.swap(still_occupied);
}

void ProbMap::hitPointUpdate(const Vec3f& pos, const int& hash_id, const int& hit_num) {
    float& ret = occupancy_buffer_[hash_id];
    GridType from_type = UNDEFINED;

    if (isOccupied(ret)) {
        from_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        from_type = GridType::KNOWN_FREE;
    }
    else {
        from_type = GridType::UNKNOWN;
    }


    ret += cfg_.l_hit * hit_num;
    if (ret > cfg_.l_max) {
        ret = cfg_.l_max;
    }

    GridType to_type;
    if (isOccupied(ret)) {
        to_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        to_type = GridType::KNOWN_FREE;
    }
    else {
        to_type = GridType::UNKNOWN;
    }

    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }
        if (cfg_.frontier_extraction_en && from_type == KNOWN_FREE) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, false);
        }
    }
}

void ProbMap::missPointUpdate(const Vec3f& pos, const int& hash_id, const int& hit_num) {
    float& ret = occupancy_buffer_[hash_id];
    GridType from_type;
    if (isOccupied(ret)) {
        from_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        from_type = GridType::KNOWN_FREE;
    }
    else {
        from_type = GridType::UNKNOWN;
    }
    ret += cfg_.l_miss * hit_num;
    if (ret < cfg_.l_min) {
        ret = cfg_.l_min;
    }

    GridType to_type;
    if (isOccupied(ret)) {
        to_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        to_type = GridType::KNOWN_FREE;
    }
    else {
        to_type = GridType::UNKNOWN;
    }
    // Catch the jump edge
    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        // Update inf map
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }


        if (cfg_.frontier_extraction_en && to_type == KNOWN_FREE) {
            Vec3i id_g;
            posToGlobalIndex(pos, id_g);
            fcnt_map_->updateFrontierCounter(id_g, true);
        }
    }
}

void ProbMap::mixedPointUpdate(const Vec3f& pos,
                               const int& hash_id,
                               const int& hit_num,
                               const int& miss_num) {
    float& ret = occupancy_buffer_[hash_id];
    GridType from_type;
    if (isOccupied(ret)) {
        from_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        from_type = GridType::KNOWN_FREE;
    }
    else {
        from_type = GridType::UNKNOWN;
    }

    ret = applyRaycastLogOddsUpdate(ret,
                                    cfg_.l_hit,
                                    cfg_.l_miss,
                                    cfg_.l_min,
                                    cfg_.l_max,
                                    hit_num,
                                    miss_num);

    GridType to_type;
    if (isOccupied(ret)) {
        to_type = GridType::OCCUPIED;
    }
    else if (isKnownFree(ret)) {
        to_type = GridType::KNOWN_FREE;
    }
    else {
        to_type = GridType::UNKNOWN;
    }

    if (from_type != to_type) {
        Vec3f center_pos;
        Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        globalIndexToPos(id_g, center_pos);
        inf_map_->updateGridCounter(center_pos, from_type, to_type);
        if (cfg_.esdf_en) {
            esdf_map_->updateGridCounter(center_pos, from_type, to_type);
        }
        if (cfg_.frontier_extraction_en && from_type == KNOWN_FREE) {
            fcnt_map_->updateFrontierCounter(id_g, false);
        }
        if (cfg_.frontier_extraction_en && to_type == KNOWN_FREE) {
            fcnt_map_->updateFrontierCounter(id_g, true);
        }
    }
}

// TAG 点云处理
void ProbMap::raycastProcess(const PointCloud& input_cloud, const Vec3f& cur_odom) {
    ++map_update_index_;
    // 初始化状态 debug 模式
    current_raycast_debug_stats_ = RaycastDebugStats{};
    current_raycast_debug_stats_.update_index = map_update_index_;
    current_raycast_debug_stats_.input_points = input_cloud.size();
    // bounding box of updated region
    raycast_data_.cache_box_min = cur_odom;
    raycast_data_.cache_box_max = cur_odom;
    Vec3f raycast_box_min, raycast_box_max;

    // 读取当前的局部更新框
    {
        std::lock_guard<std::mutex> lck{raycast_data_.raycast_range_mtx};
        raycast_box_max = raycast_data_.local_update_box_max;
        raycast_box_min = raycast_data_.local_update_box_min;
    }

    /// Step 1; Raycast and add to update cache.
    const int& cloud_in_size = input_cloud.size();
    // Endpoints that will be used as raycasting targets.
    auto raycast_endpoints = vec_Vec3f{};
    raycast_endpoints.reserve(cloud_in_size);

    // 1) process all non-inf points, update occupied probability
    int temperol_cnt{0};
    // 遍历输入点云 处理端点
    for (const auto& pcl_p : input_cloud) {
        // 1.1) intensity filter
        // 强度太低的去除
        if (cfg_.intensity_thresh > 0 &&
            pcl_p.intensity < cfg_.intensity_thresh) {
            ++current_raycast_debug_stats_.intensity_skipped_points;
            continue;
        }

        // 1.2) temporal filter
        // HACK 降采样  好粗暴的降采样 并且有必要吗 ???
        if (temperol_cnt++ % cfg_.point_filt_num) {
            ++current_raycast_debug_stats_.temporal_skipped_points;
            continue;
        }

        Vec3f p(pcl_p.x, pcl_p.y, pcl_p.z);
        Vec3i pt_id_g;

        // no raycasting, purely add occ pints
        if (!cfg_.raycasting_en) {
            // 不做射线清空 只把点云端点当做障碍物 hit 来更新 不会进入
            if (insideLocalMap(p)) {
                // 检查点云点是否在 local map中
                double sqrdis = (p - cur_odom).squaredNorm();
                if(sqrdis<cfg_.sqr_raycast_range_min){
                    ++current_raycast_debug_stats_.min_range_skipped_points;
                    continue;
                }
                posToGlobalIndex(p, pt_id_g);
                // 对有效端点调用 insertUpdateCandidate 标记为hit
                insertUpdateCandidate(pt_id_g, true);
                ++current_raycast_debug_stats_.hit_endpoint_points;
                // record cache box size;
                raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
                raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);
            }
            continue;
        }

        // 标记是否为有效的障碍点
        bool update_hit{true};
        bool clipped_to_free_endpoint{false};
        // 1.3) filter for virtual ceil and ground
        // 打到了天花板
        if (p.z() > cfg_.virtual_ceil_height) {
            // 不在作为真实障碍物端点进行更新
            update_hit = false;
            ++current_raycast_debug_stats_.virtual_ceil_clipped_points;
            // find the intersect point with the ceil
            const double dz = p.z() - cur_odom.z();
            // 原始点 p
            //     *
            //     |
            //     ---|--- virtual_ceil_height
            //     x   裁剪后的 p
            //     |
            //     robot cur_odom
            const double pc = cfg_.virtual_ceil_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }
        else if (p.z() < cfg_.virtual_ground_height) {
            // 点 低于 天花板 裁剪到射线和虚拟地面平面的交点
            update_hit = false;
            ++current_raycast_debug_stats_.virtual_ground_clipped_points;
            // find the intersect point with the ground
            const double dz = p.z() - cur_odom.z();
            const double pc = cfg_.virtual_ground_height - cur_odom.z();
            p = cur_odom + (p - cur_odom).normalized() * pc / dz;
        }

        // 1.4) bounding box filter
        // raycasting max
        // 计算 p 到机器人 cur_odom 的距离平方
        const double sqr_dis = (p - cur_odom).squaredNorm();
        // 点太远 裁剪到最大量程
        if (sqr_dis > cfg_.sqr_raycast_range_max) {
            double k = cfg_.raycast_range_max / sqrt(sqr_dis);
            p = k * (p - cur_odom) + cur_odom;
            // 不是真实的障碍物的碰撞点
            update_hit = false;
            clipped_to_free_endpoint = true;
            ++current_raycast_debug_stats_.range_clipped_points;
        }

        // 点过近 跳过
        if(sqr_dis < cfg_.sqr_raycast_range_min) {
            ++current_raycast_debug_stats_.min_range_skipped_points;
            continue;
        }

        // local map bound
        //
        if (((p - raycast_box_min).minCoeff() < 0) ||
            ((p - raycast_box_max).maxCoeff() > 0)) {
            // p 不在更新框内 ?? 不是已经裁剪了吗
            // 再进行一次裁剪 裁剪到 box
            p = lineBoxIntersectPoint(p,
                                      cur_odom,
                                      raycast_box_min,
                                      raycast_box_max);
            update_hit = false;
            clipped_to_free_endpoint = true;
            ++current_raycast_debug_stats_.local_box_clipped_points;
        }


        // record cache box size;
        // 更新本次处理过的点的包围盒
        raycast_data_.cache_box_min = raycast_data_.cache_box_min.cwiseMin(p);
        raycast_data_.cache_box_max = raycast_data_.cache_box_max.cwiseMax(p);

        // 1.4) for all validate hit points, update probability
        // 添加到 raycast_endpoints 中用于更新
        raycast_endpoints.push_back(p);
        ++current_raycast_debug_stats_.raycast_endpoint_points;

        // 没有被裁剪才算做真实的 hit
        if (update_hit) {
            // p 点云中每个点都标记为 hit
            // HACK hit 可视化为 marker ??
            posToGlobalIndex(p, pt_id_g);
            // 缓存 hit 与 miss 的计数
            insertUpdateCandidate(pt_id_g, true);
            ++current_raycast_debug_stats_.hit_endpoint_points;
        }
        else if (cfg_.clear_clipped_endpoint && clipped_to_free_endpoint) {
            posToGlobalIndex(p, pt_id_g);
            if (insideLocalMap(pt_id_g)) {
                insertUpdateCandidate(pt_id_g, false);
                ++current_raycast_debug_stats_.clipped_endpoint_miss_cells;
            }
        }
    }

    if (cfg_.raycasting_en) {
        // 4) process all inf points, updae free probability
        for (const auto& p : raycast_endpoints) {
            // 遍历处理好的有效射线端点
            Vec3f raycast_start = (p - cur_odom).normalized() * cfg_.raycast_range_min + cur_odom;
            // 设置射线起点和终点
            raycast_data_.raycaster.setInput(raycast_start, p);
            Vec3f ray_pt;
            // 沿射线每次前进一个像素返回当前体素中心坐标
            while (raycast_data_.raycaster.step(ray_pt)) {
                Vec3i cur_ray_id_g;
                posToGlobalIndex(ray_pt, cur_ray_id_g);
                if (!insideLocalMap(cur_ray_id_g)) {
                    // 超出地图
                    break;
                }
                // 沿途每个cell标记为miss
                insertUpdateCandidate(cur_ray_id_g, false);
            }
        }
    }
}

void ProbMap::insertUpdateCandidate(const Vec3i& id_g, bool is_hit) {
    // 将三维全局栅格坐标转成一维数组下标
    const auto& hash_id = getHashIndexFromGlobalIndex(id_g);
    raycast_data_.operation_cnt[hash_id]++;
    if (raycast_data_.operation_cnt[hash_id] == 1) {
        raycast_data_.update_cache_id_g.push(id_g);
    }
    if (is_hit) {
        raycast_data_.hit_cnt[hash_id]++;
    }
}

void ProbMap::updateLocalBox(const Vec3f& cur_odom) {
    /* This function is only used for decide the local update range
     * and do not related to the map origin and map bound
     * the map origin and map bound is only update in [SlidingMap::mapSliding(const Vec3f &odom)]
     * */
    // The local map should be inside in index wise
    // update: the virtual floor and ceil should not influence the raycasting.
    // 2) local map size
    // The update box should follow odom.
    // The local map should follow current map center.
    std::lock_guard<std::mutex> lck(raycast_data_.raycast_range_mtx);

    Vec3i cur_odom_i;
    // 转换为栅格索引
    posToGlobalIndex(cur_odom, cur_odom_i);
    Vec3i local_updatebox_min_i, local_updatebox_max_i;

    // 计算出一个立方体更新范围
    if (cfg_.raycasting_en) {
        local_updatebox_max_i = cur_odom_i + cfg_.half_local_update_box_i;
        local_updatebox_min_i = cur_odom_i - cfg_.half_local_update_box_i;
    }

    // 转换会世界坐标系
    globalIndexToPos(local_updatebox_min_i, raycast_data_.local_update_box_min);
    globalIndexToPos(local_updatebox_max_i, raycast_data_.local_update_box_max);

    // the local update box must inside the local map
    // 存储回 local_update_box_max
    raycast_data_.local_update_box_max = raycast_data_.local_update_box_max.cwiseMin(
        local_map_bound_max_d_);
    raycast_data_.local_update_box_min = raycast_data_.local_update_box_min.cwiseMax(
        local_map_bound_min_d_);
}

void ProbMap::resetLocalMap() {
    std::cout << YELLOW << " -- [Prob-Map] Clear all local map." << RESET << std::endl;
    double unk_value = (cfg_.l_free + cfg_.l_occ)/2.0;
    // Clear local map
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), unk_value);
    std::fill(cell_last_observed_update_index_.begin(), cell_last_observed_update_index_.end(), 0);
    std::fill(cell_last_update_index_.begin(), cell_last_update_index_.end(), 0);
    std::fill(cell_last_hit_update_index_.begin(), cell_last_hit_update_index_.end(), 0);
    std::fill(cell_last_miss_update_index_.begin(), cell_last_miss_update_index_.end(), 0);
    std::fill(cell_last_hit_count_.begin(), cell_last_hit_count_.end(), 0);
    std::fill(cell_last_miss_count_.begin(), cell_last_miss_count_.end(), 0);
    std::fill(occupied_cell_flags_.begin(), occupied_cell_flags_.end(), 0U);
    std::fill(occupied_cell_listed_flags_.begin(), occupied_cell_listed_flags_.end(), 0U);
    occupied_cell_hash_ids_.clear();
    while (!raycast_data_.update_cache_id_g.empty()) {
        raycast_data_.update_cache_id_g.pop();
    }
    raycast_data_.batch_update_counter = 0;
    std::fill(raycast_data_.operation_cnt.begin(), raycast_data_.operation_cnt.end(), 0);
    std::fill(raycast_data_.hit_cnt.begin(), raycast_data_.hit_cnt.end(), 0);
}
