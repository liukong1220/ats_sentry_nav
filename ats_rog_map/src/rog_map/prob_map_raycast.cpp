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
