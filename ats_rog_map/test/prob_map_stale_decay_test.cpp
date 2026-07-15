#include <rog_map/prob_map.h>

#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

class TestProbMap : public rog_map::ProbMap {
public:
    void setupTrackedUnknownCell() {
        cfg_.l_free = -1.0F;
        cfg_.l_occ = 1.0F;
        occupancy_buffer_.assign(1, 0.0F);
        cell_last_observed_update_index_.assign(1, 42);
        cell_last_update_index_.assign(1, 42);
        cell_last_hit_update_index_.assign(1, 42);
        cell_last_miss_update_index_.assign(1, 0);
        cell_last_hit_count_.assign(1, 1);
        cell_last_miss_count_.assign(1, 0);
        occupied_cell_flags_.assign(1, 1U);
        occupied_cell_listed_flags_.assign(1, 1U);
        occupied_cell_hash_ids_.assign(1, 0);
    }

    void setupReusedTrackedCellWithoutCompaction() {
        cfg_.stale_decay_en = true;
        setupTrackedUnknownCell();
        resetCell(0);
        occupancy_buffer_[0] = 2.0F;
        cell_last_observed_update_index_[0] = 43;
        updateTrackedOccupiedCell(0, rog_map::GridType::OCCUPIED);
    }

    void setupSingleCellMapForDebug() {
        cfg_.stale_decay_en = true;
        cfg_.stale_soft_ttl_updates = 1;
        cfg_.stale_hard_ttl_updates = 0;
        cfg_.stale_decay_log_odds_step = 0.5F;
        cfg_.stale_sweep_interval_updates = 1;
        cfg_.l_free = -1.0F;
        cfg_.l_occ = 1.0F;
        cfg_.l_hit = 0.5F;
        cfg_.l_miss = -0.25F;
        cfg_.l_min = -10.0F;
        cfg_.l_max = 10.0F;
        sc_.resolution = 1.0;
        sc_.resolution_inv = 1.0;
        sc_.half_map_size_i = rog_map::Vec3i::Zero();
        sc_.map_size_i = rog_map::Vec3i::Ones();
        sc_.map_vox_num = 1;
        local_map_origin_i_ = rog_map::Vec3i::Zero();
        local_map_origin_d_ = rog_map::Vec3f::Zero();
        raycast_data_.local_update_box_min = rog_map::Vec3f(-1.0, -1.0, -1.0);
        raycast_data_.local_update_box_max = rog_map::Vec3f(1.0, 1.0, 1.0);
        occupancy_buffer_.assign(1, 2.0F);
        cell_last_observed_update_index_.assign(1, 9);
        cell_last_update_index_.assign(1, 0);
        cell_last_hit_update_index_.assign(1, 0);
        cell_last_miss_update_index_.assign(1, 0);
        cell_last_hit_count_.assign(1, 0);
        cell_last_miss_count_.assign(1, 0);
        occupied_cell_flags_.assign(1, 1U);
        occupied_cell_listed_flags_.assign(1, 1U);
        occupied_cell_hash_ids_.assign(1, 0);
        raycast_data_.operation_cnt.assign(1, 1);
        raycast_data_.hit_cnt.assign(1, 0);
        raycast_data_.update_cache_id_g.push(rog_map::Vec3i::Zero());
        map_update_index_ = 10;
    }

    void setupTwoTrackedCellsForUpdateBoxDecay() {
        cfg_.stale_decay_en = true;
        cfg_.stale_decay_local_update_box_only = true;
        cfg_.stale_soft_ttl_updates = 1;
        cfg_.stale_hard_ttl_updates = 0;
        cfg_.stale_decay_log_odds_step = 0.5F;
        cfg_.stale_sweep_interval_updates = 1;
        cfg_.l_free = -1.0F;
        cfg_.l_occ = 1.0F;
        sc_.resolution = 1.0;
        sc_.resolution_inv = 1.0;
        sc_.half_map_size_i = rog_map::Vec3i(1, 0, 0);
        sc_.map_size_i = rog_map::Vec3i(3, 1, 1);
        sc_.map_vox_num = 3;
        local_map_origin_i_ = rog_map::Vec3i::Zero();
        local_map_origin_d_ = rog_map::Vec3f::Zero();
        occupancy_buffer_.assign(3, 2.0F);
        cell_last_observed_update_index_.assign(3, 1);
        cell_last_update_index_.assign(3, 0);
        cell_last_hit_update_index_.assign(3, 0);
        cell_last_miss_update_index_.assign(3, 0);
        cell_last_hit_count_.assign(3, 0);
        cell_last_miss_count_.assign(3, 0);
        occupied_cell_flags_.assign(3, 1U);
        occupied_cell_listed_flags_.assign(3, 1U);
        occupied_cell_hash_ids_ = {1, 2};
        raycast_data_.local_update_box_min = rog_map::Vec3f(-0.1, -1.0, -1.0);
        raycast_data_.local_update_box_max = rog_map::Vec3f(0.9, 1.0, 1.0);
        map_update_index_ = 10;
    }

    void setupLineMapForClippedEndpoint() {
        cfg_.raycasting_en = true;
        cfg_.clear_clipped_endpoint = true;
        cfg_.point_filt_num = 1;
        cfg_.intensity_thresh = -1.0;
        cfg_.raycast_range_min = 0.0;
        cfg_.raycast_range_max = 1.5;
        cfg_.sqr_raycast_range_min = 0.0;
        cfg_.sqr_raycast_range_max = cfg_.raycast_range_max * cfg_.raycast_range_max;
        cfg_.virtual_ground_height = -10.0;
        cfg_.virtual_ceil_height = 10.0;
        sc_.resolution = 1.0;
        sc_.resolution_inv = 1.0;
        sc_.half_map_size_i = rog_map::Vec3i(3, 0, 0);
        sc_.map_size_i = rog_map::Vec3i(7, 1, 1);
        sc_.map_vox_num = 7;
        local_map_origin_i_ = rog_map::Vec3i::Zero();
        local_map_origin_d_ = rog_map::Vec3f::Zero();
        local_map_bound_min_d_ = rog_map::Vec3f(-2.5, -0.5, -0.5);
        local_map_bound_max_d_ = rog_map::Vec3f(3.5, 0.5, 0.5);
        raycast_data_.local_update_box_min = rog_map::Vec3f(-2.5, -0.5, -0.5);
        raycast_data_.local_update_box_max = rog_map::Vec3f(3.5, 0.5, 0.5);
        occupancy_buffer_.assign(7, 0.0F);
        cell_last_observed_update_index_.assign(7, 0);
        cell_last_update_index_.assign(7, 0);
        cell_last_hit_update_index_.assign(7, 0);
        cell_last_miss_update_index_.assign(7, 0);
        cell_last_hit_count_.assign(7, 0);
        cell_last_miss_count_.assign(7, 0);
        occupied_cell_flags_.assign(7, 0U);
        occupied_cell_listed_flags_.assign(7, 0U);
        occupied_cell_hash_ids_.clear();
        raycast_data_.raycaster.setResolution(1.0);
        raycast_data_.operation_cnt.assign(7, 0);
        raycast_data_.hit_cnt.assign(7, 0);
        while (!raycast_data_.update_cache_id_g.empty()) {
            raycast_data_.update_cache_id_g.pop();
        }
        map_update_index_ = 1;
    }

    void resetCellForTest(const int hash_id) {
        resetCell(hash_id);
    }

    bool hasTrackedFlag(const int hash_id) const {
        return occupied_cell_flags_[hash_id] != 0U;
    }

    std::uint64_t lastObservedUpdate(const int hash_id) const {
        return cell_last_observed_update_index_[hash_id];
    }

    std::uint64_t lastUpdate(const int hash_id) const {
        return cell_last_update_index_[hash_id];
    }

    std::uint64_t lastHitUpdate(const int hash_id) const {
        return cell_last_hit_update_index_[hash_id];
    }

    std::uint64_t lastMissUpdate(const int hash_id) const {
        return cell_last_miss_update_index_[hash_id];
    }

    std::size_t trackedHashCount() const {
        return occupied_cell_hash_ids_.size();
    }

    void setupSingleOccupiedCellCache(const int hit_count,
                                      const int operation_count,
                                      const std::uint64_t last_observed,
                                      const std::uint64_t current_update) {
        cfg_.stale_decay_en = false;
        cfg_.l_free = -1.0F;
        cfg_.l_occ = 1.0F;
        cfg_.l_hit = 0.5F;
        cfg_.l_miss = -0.25F;
        cfg_.l_min = -10.0F;
        cfg_.l_max = 10.0F;
        sc_.resolution = 1.0;
        sc_.resolution_inv = 1.0;
        sc_.half_map_size_i = rog_map::Vec3i::Zero();
        sc_.map_size_i = rog_map::Vec3i::Ones();
        sc_.map_vox_num = 1;
        local_map_origin_i_ = rog_map::Vec3i::Zero();
        local_map_origin_d_ = rog_map::Vec3f::Zero();
        occupancy_buffer_.assign(1, 2.0F);
        cell_last_observed_update_index_.assign(1, last_observed);
        cell_last_update_index_.assign(1, 0);
        cell_last_hit_update_index_.assign(1, 0);
        cell_last_miss_update_index_.assign(1, 0);
        cell_last_hit_count_.assign(1, 0);
        cell_last_miss_count_.assign(1, 0);
        occupied_cell_flags_.assign(1, 0U);
        occupied_cell_listed_flags_.assign(1, 0U);
        occupied_cell_hash_ids_.clear();
        raycast_data_.operation_cnt.assign(1, operation_count);
        raycast_data_.hit_cnt.assign(1, hit_count);
        raycast_data_.update_cache_id_g.push(rog_map::Vec3i::Zero());
        map_update_index_ = current_update;
    }

    void runCachedUpdateForTest() {
        probabilisticMapFromCache();
    }

    void runStaleDecayForTest() {
        applyStaleDecay();
    }

    void runRaycastForTest(const rog_map::PointCloud &cloud, const rog_map::Vec3f &cur_odom) {
        raycastProcess(cloud, cur_odom);
    }

    float occupancyValue(const int hash_id) const {
        return occupancy_buffer_[hash_id];
    }

    int operationCount(const int hash_id) const {
        return raycast_data_.operation_cnt[hash_id];
    }

    int hitCount(const int hash_id) const {
        return raycast_data_.hit_cnt[hash_id];
    }

    const rog_map::ProbMap::RaycastDebugStats &rayDebugStats() const {
        return current_raycast_debug_stats_;
    }

    const std::vector<rog_map::ProbMap::RaycastDebugCell> &rayDebugCells() const {
        return getLastRaycastDebugCells();
    }

    const std::vector<rog_map::ProbMap::StaleDebugCell> &staleDebugCells() const {
        return getLastStaleDebugCells();
    }
};

bool expectNear(const float actual, const float expected, const char *label) {
    if (std::fabs(actual - expected) < 1.0e-5F) {
        return true;
    }
    std::cerr << label << ": expected " << expected << ", got " << actual << std::endl;
    return false;
}

bool expectEqual(const bool actual, const bool expected, const char *label) {
    if (actual == expected) {
        return true;
    }
    std::cerr << label << ": expected " << expected << ", got " << actual << std::endl;
    return false;
}

bool expectEqualUint64(const std::uint64_t actual,
                       const std::uint64_t expected,
                       const char *label) {
    if (actual == expected) {
        return true;
    }
    std::cerr << label << ": expected " << expected << ", got " << actual << std::endl;
    return false;
}

bool expectEqualSize(const std::size_t actual,
                     const std::size_t expected,
                     const char *label) {
    if (actual == expected) {
        return true;
    }
    std::cerr << label << ": expected " << expected << ", got " << actual << std::endl;
    return false;
}

}  // namespace

int main() {
    bool ok = true;
    bool hard_clear = false;

    ok = expectNear(
        rog_map::ProbMap::applyStaleDecayUpdate(
            2.0F,
            9,
            10,
            3,
            8,
            0.4F,
            0.0F,
            hard_clear),
        2.0F,
        "fresh occupied cell should not decay") && ok;
    ok = expectEqual(hard_clear, false, "fresh occupied cell should not hard clear") && ok;

    ok = expectNear(
        rog_map::ProbMap::applyStaleDecayUpdate(
            2.0F,
            5,
            10,
            3,
            8,
            0.4F,
            0.0F,
            hard_clear),
        1.6F,
        "soft stale occupied cell should decay one step") && ok;
    ok = expectEqual(hard_clear, false, "soft stale occupied cell should not hard clear") && ok;

    ok = expectNear(
        rog_map::ProbMap::applyStaleDecayUpdate(
            2.0F,
            1,
            10,
            3,
            8,
            0.4F,
            0.0F,
            hard_clear),
        0.0F,
        "hard stale occupied cell should return to unknown") && ok;
    ok = expectEqual(hard_clear, true, "hard stale occupied cell should report hard clear") && ok;

    ok = expectNear(
        rog_map::ProbMap::applyStaleDecayUpdate(
            0.2F,
            5,
            10,
            3,
            8,
            0.4F,
            0.0F,
            hard_clear),
        0.0F,
        "soft decay should clamp to unknown value") && ok;
    ok = expectEqual(hard_clear, false, "soft clamp should not report hard clear") && ok;

    TestProbMap map;
    map.setupTrackedUnknownCell();
    map.resetCellForTest(0);
    ok = expectEqual(map.hasTrackedFlag(0), false, "reset cell should clear stale tracking flag") && ok;
    ok = expectEqualUint64(map.lastObservedUpdate(0), 0, "reset cell should clear last observed update") && ok;
    ok = expectEqualUint64(map.lastUpdate(0), 0, "reset cell should clear last update") && ok;
    ok = expectEqualUint64(map.lastHitUpdate(0), 0, "reset cell should clear last hit update") && ok;
    ok = expectEqualUint64(map.lastMissUpdate(0), 0, "reset cell should clear last miss update") && ok;

    TestProbMap reused_map;
    reused_map.setupReusedTrackedCellWithoutCompaction();
    ok = expectEqualSize(
        reused_map.trackedHashCount(),
        1,
        "reused hash should not be inserted into stale tracking twice") && ok;

    TestProbMap miss_only_map;
    miss_only_map.setupSingleOccupiedCellCache(0, 1, 4, 10);
    miss_only_map.runCachedUpdateForTest();
    ok = expectEqualUint64(
        miss_only_map.lastObservedUpdate(0),
        4,
        "miss-only occupied update should not refresh stale TTL") && ok;
    ok = expectEqualUint64(
        miss_only_map.lastUpdate(0),
        10,
        "miss-only update should record the touched update") && ok;
    ok = expectEqualUint64(
        miss_only_map.lastHitUpdate(0),
        0,
        "miss-only update should not record a hit") && ok;
    ok = expectEqualUint64(
        miss_only_map.lastMissUpdate(0),
        10,
        "miss-only update should record ray pass") && ok;
    {
        std::vector<rog_map::ProbMap::VoxelDebugCell> cells;
        rog_map::ProbMap::VoxelDebugStats stats;
        miss_only_map.collectLocalVoxelDebug(cells, stats);
        ok = expectEqualSize(cells.size(), 1, "voxel debug should export the single test cell") && ok;
        if (!cells.empty()) {
            ok = expectEqual(cells.front().updated_this_frame, true, "voxel debug should mark touched this frame") && ok;
            ok = expectEqual(cells.front().miss_this_frame, true, "voxel debug should mark miss this frame") && ok;
            ok = expectEqual(cells.front().hit_this_frame, false, "voxel debug should not mark hit this frame") && ok;
            ok = expectEqual(cells.front().passed_by_ray, true, "voxel debug should mark ray-passed cells") && ok;
            ok = expectEqual(cells.front().never_touched, false, "voxel debug should not call a touched cell never_touched") && ok;
        }
        ok = expectEqualSize(
            static_cast<std::size_t>(stats.miss_this_frame_cells),
            1,
            "voxel debug stats should count miss-this-frame") && ok;
    }

    TestProbMap hit_map;
    hit_map.setupSingleOccupiedCellCache(1, 1, 4, 11);
    hit_map.runCachedUpdateForTest();
    ok = expectEqualUint64(
        hit_map.lastObservedUpdate(0),
        11,
        "hit occupied update should refresh stale TTL") && ok;
    ok = expectEqualUint64(
        hit_map.lastHitUpdate(0),
        11,
        "hit update should record hit time") && ok;

    TestProbMap debug_map;
    debug_map.setupSingleCellMapForDebug();
    debug_map.runCachedUpdateForTest();
    ok = expectEqualSize(
        debug_map.rayDebugCells().size(),
        1,
        "raycast debug should include cached update cells") && ok;
    if (!debug_map.rayDebugCells().empty()) {
        const auto &cell = debug_map.rayDebugCells().front();
        ok = expectEqualUint64(
            static_cast<std::uint64_t>(cell.miss_count),
            1,
            "raycast debug cell should record miss count") && ok;
        ok = expectEqual(cell.from_type == rog_map::GridType::OCCUPIED, true, "raycast debug cell should record from type") && ok;
    }
    ok = expectEqualSize(
        debug_map.staleDebugCells().size(),
        1,
        "stale debug should include decayed cells") && ok;
    if (!debug_map.staleDebugCells().empty()) {
        const auto &cell = debug_map.staleDebugCells().front();
        ok = expectEqual(cell.hard_cleared, false, "soft stale debug cell should not be hard cleared") && ok;
        ok = expectNear(cell.old_log_odds, 1.75F, "stale debug should see post-raycast log odds") && ok;
        ok = expectNear(cell.new_log_odds, 1.25F, "stale debug should record decayed log odds") && ok;
    }

    TestProbMap update_box_map;
    update_box_map.setupTwoTrackedCellsForUpdateBoxDecay();
    update_box_map.runStaleDecayForTest();
    ok = expectNear(
        update_box_map.occupancyValue(1),
        1.5F,
        "stale decay should apply inside local update box") && ok;
    ok = expectNear(
        update_box_map.occupancyValue(2),
        2.0F,
        "stale decay should skip occupied cells outside local update box") && ok;
    ok = expectEqualUint64(
        static_cast<std::uint64_t>(update_box_map.rayDebugStats().stale_outside_update_box_cells),
        1,
        "stale decay should count occupied cells skipped outside update box") && ok;

    TestProbMap clipped_endpoint_map;
    clipped_endpoint_map.setupLineMapForClippedEndpoint();
    rog_map::PointCloud clipped_cloud;
    rog_map::PclPoint point;
    point.x = 3.0F;
    point.y = 0.0F;
    point.z = 0.0F;
    point.intensity = 1.0F;
    clipped_cloud.points.push_back(point);
    clipped_cloud.width = 1;
    clipped_cloud.height = 1;
    clipped_cloud.is_dense = true;
    clipped_endpoint_map.runRaycastForTest(clipped_cloud, rog_map::Vec3f(0.0, 0.0, 0.0));
    ok = expectEqualUint64(
        static_cast<std::uint64_t>(clipped_endpoint_map.rayDebugStats().range_clipped_points),
        1,
        "raycast should classify beyond-range point as clipped") && ok;
    ok = expectEqualUint64(
        static_cast<std::uint64_t>(clipped_endpoint_map.rayDebugStats().clipped_endpoint_miss_cells),
        1,
        "clipped free endpoint should add one explicit miss") && ok;
    ok = expectEqualUint64(
        static_cast<std::uint64_t>(clipped_endpoint_map.operationCount(4)),
        1,
        "clipped endpoint voxel should be in update cache as miss") && ok;
    ok = expectEqualUint64(
        static_cast<std::uint64_t>(clipped_endpoint_map.hitCount(4)),
        0,
        "clipped endpoint voxel should not be marked as hit") && ok;

    return ok ? 0 : 1;
}
