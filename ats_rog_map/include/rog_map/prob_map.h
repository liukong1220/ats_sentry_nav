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


#pragma once

#include <cstdint>
#include <limits>
#include <mutex>
#include <queue>
#include <rog_map/inf_map.h>
#include <rog_map/free_cnt_map.h>
#include <rog_map/esdf_map.h>
#include <rog_map/rog_map_core/raycaster.h>


namespace rog_map {
    using super_utils::Pose;


    class ProbMap : public SlidingMap {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<ProbMap> Ptr;

        ProbMap() = default;

        ~ProbMap() override = default;

        struct RaycastDebugStats {
            std::uint64_t update_index{0};
            int input_points{0};
            int intensity_skipped_points{0};
            int temporal_skipped_points{0};
            int min_range_skipped_points{0};
            int virtual_ceil_clipped_points{0};
            int virtual_ground_clipped_points{0};
            int range_clipped_points{0};
            int local_box_clipped_points{0};
            int raycast_endpoint_points{0};
            int hit_endpoint_points{0};
            int update_cache_cells{0};
            int hit_only_cells{0};
            int miss_only_cells{0};
            int hit_and_miss_cells{0};
            int hit_votes{0};
            int miss_votes{0};
            int occupied_touched_cells{0};
            int occupied_hit_cells{0};
            int occupied_miss_only_cells{0};
            int occupied_to_unknown_cells{0};
            int occupied_to_free_cells{0};
            int unknown_to_occupied_cells{0};
            int unknown_to_free_cells{0};
            int free_to_occupied_cells{0};
            int free_to_unknown_cells{0};
            int stale_tracked_occupied_cells{0};
            int stale_fresh_occupied_cells{0};
            int stale_occupied_cells{0};
            int stale_decayed_cells{0};
            int stale_to_unknown_cells{0};
            int stale_hard_cleared_cells{0};
            int stale_age_soft_to_hard_cells{0};
            int stale_age_over_hard_cells{0};
            int stale_outside_update_box_cells{0};
            int clipped_endpoint_miss_cells{0};
            std::uint64_t stale_max_age{0};
        };

        struct RaycastDebugCell {
            Vec3i id_g = Vec3i::Zero();
            Vec3f center = Vec3f::Zero();
            float old_log_odds{0.0F};
            float new_log_odds{0.0F};
            int hit_count{0};
            int miss_count{0};
            GridType from_type{super_utils::UNDEFINED};
            GridType to_type{super_utils::UNDEFINED};
        };

        struct StaleDebugCell {
            Vec3i id_g = Vec3i::Zero();
            Vec3f center = Vec3f::Zero();
            float old_log_odds{0.0F};
            float new_log_odds{0.0F};
            std::uint64_t age_updates{0};
            bool hard_cleared{false};
            GridType from_type{super_utils::UNDEFINED};
            GridType to_type{super_utils::UNDEFINED};
        };

        struct VoxelDebugCell {
            Vec3i id_g = Vec3i::Zero();
            Vec3f center = Vec3f::Zero();
            GridType type{super_utils::UNDEFINED};
            float log_odds{0.0F};
            std::uint64_t last_update_index{0};
            std::uint64_t last_hit_update_index{0};
            std::uint64_t last_miss_update_index{0};
            std::uint64_t last_observed_update_index{0};
            std::uint64_t age_updates{0};
            int last_hit_count{0};
            int last_miss_count{0};
            bool updated_this_frame{false};
            bool hit_this_frame{false};
            bool miss_this_frame{false};
            bool passed_by_ray{false};
            bool never_touched{true};
            bool tracked_occupied{false};
            bool stale_soft_band{false};
            bool stale_hard_band{false};
        };

        struct VoxelDebugStats {
            std::uint64_t update_index{0};
            int total_cells{0};
            int exported_cells{0};
            int occupied_cells{0};
            int known_free_cells{0};
            int unknown_cells{0};
            int updated_this_frame_cells{0};
            int hit_this_frame_cells{0};
            int miss_this_frame_cells{0};
            int hit_and_miss_this_frame_cells{0};
            int ray_passed_cells{0};
            int never_touched_cells{0};
            int stale_soft_band_cells{0};
            int stale_hard_band_cells{0};
        };

        void initProbMap();

        bool isOccupied(const Vec3f &pos) const;

        bool isUnknown(const Vec3f &pos) const;

        bool isKnownFree(const Vec3f &pos) const;

        bool isOccupiedInflate(const Vec3f &pos) const;

        bool isUnknownInflate(const Vec3f &pos) const;

        bool isKnownFreeInflate(const Vec3f & pos) const;

        bool isFrontier(const Vec3f &pos) const;

        bool isFrontier(const Vec3i &id_g) const;

        // Query result
        GridType getGridType(Vec3i &id_g) const;

        GridType getGridType(const Vec3f &pos) const;

        GridType getInfGridType(const Vec3f &pos) const;

        double getMapValue(const Vec3f &pos) const;

        void boxSearch(const Vec3f &_box_min, const Vec3f &_box_max,
                       const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boxSearchInflate(const Vec3f &box_min, const Vec3f &box_max,
                              const GridType &gt, vec_E<Vec3f> &out_points) const;

        void boundBoxByLocalMap(Vec3f &box_min, Vec3f &box_max) const;

        Vec3f getLocalMapOrigin() const;

        Vec3f getLocalMapSize() const;

        // The raycast update box is the quantized, map-clipped region used by
        // the current update cycle. It is exposed for diagnostics only.
        void getRaycastLocalUpdateBox(Vec3f & box_min, Vec3f & box_max) const;

        double getResolution() const{
            return sc_.resolution;
        }

        double getInfResolution()const {
            return inf_map_->getResolution();
        }

        bool hasESDF() const {
            return static_cast<bool>(esdf_map_);
        }

        double getESDFDistance(const Vec3f &pos) const {
            return esdf_map_ ? esdf_map_->getDistance(pos) :
                   std::numeric_limits<double>::quiet_NaN();
        }

        void updateOccPointCloud(const PointCloud &input_cloud);

        void writeTimeConsumingToLog(std::ofstream &log_file);

        void writeMapInfoToLog(std::ofstream &log_file);

        void updateProbMap(
                const PointCloud &cloud, const Pose &sensor_pose,
                const Vec3f &map_center);

        const RaycastDebugStats &getLastRaycastDebugStats() const {
            return last_raycast_debug_stats_;
        }

        const std::vector<RaycastDebugCell> &getLastRaycastDebugCells() const {
            return last_raycast_debug_cells_;
        }

        const std::vector<StaleDebugCell> &getLastStaleDebugCells() const {
            return last_stale_debug_cells_;
        }

        void collectLocalVoxelDebug(std::vector<VoxelDebugCell> &cells,
                                    VoxelDebugStats &stats,
                                    int stride = 1,
                                    bool include_unknown = true) const;

        static float applyRaycastLogOddsUpdate(float value,
                                               float l_hit,
                                               float l_miss,
                                               float l_min,
                                               float l_max,
                                               int hit_count,
                                               int miss_count);

        static float applyStaleDecayUpdate(float value,
                                           std::uint64_t last_observed_update,
                                           std::uint64_t current_update,
                                           int soft_ttl_updates,
                                           int hard_ttl_updates,
                                           float decay_log_odds_step,
                                           float unknown_log_odds,
                                           bool &hard_clear);

    protected:
        rog_map::Config cfg_;
        InfMap::Ptr inf_map_;
        FreeCntMap::Ptr fcnt_map_;
        ESDFMap::Ptr esdf_map_;
        /// Spherical neighborhood lookup table
        std::vector<float> occupancy_buffer_;
        std::vector<std::uint64_t> cell_last_observed_update_index_;
        std::vector<std::uint64_t> cell_last_update_index_;
        std::vector<std::uint64_t> cell_last_hit_update_index_;
        std::vector<std::uint64_t> cell_last_miss_update_index_;
        std::vector<std::uint16_t> cell_last_hit_count_;
        std::vector<std::uint16_t> cell_last_miss_count_;
        std::vector<std::uint8_t> occupied_cell_flags_;
        std::vector<std::uint8_t> occupied_cell_listed_flags_;
        std::vector<int> occupied_cell_hash_ids_;

        bool map_empty_{true};
        struct RaycastData {
            raycaster::RayCaster raycaster;
            std::queue<Vec3i> update_cache_id_g;
            std::vector<uint16_t> operation_cnt;
            std::vector<uint16_t> hit_cnt;
            Vec3f cache_box_max, cache_box_min, local_update_box_max, local_update_box_min;
            int batch_update_counter{0};
            mutable std::mutex raycast_range_mtx;
        } raycast_data_;

        vector<double> time_consuming_;
        vector<string> time_consuming_name_{"Total", "Raycast", "Update_cache", "Inflation", "PointCloudNumber",
                                            "CacheNumber", "InflationNumber"};
        std::uint64_t map_update_index_{0};
        RaycastDebugStats current_raycast_debug_stats_;
        RaycastDebugStats last_raycast_debug_stats_;
        std::vector<RaycastDebugCell> last_raycast_debug_cells_;
        std::vector<StaleDebugCell> last_stale_debug_cells_;

        // standardization query
        // Known free < l_free
        // occupied >= l_occ
        bool isKnownFree(const double &prob) const {
            return prob < cfg_.l_free;
        }

        bool isOccupied(const double &prob) const {
            return prob >= cfg_.l_occ;
        }

        bool isUnknown(const double &prob) const {
            return prob >= cfg_.l_free && prob < cfg_.l_occ;
        }

        void slideAllMap(const Vec3f &pos);

        // warning using this function will cause memory leak if the id_g is not in the map
        bool isOccupied(const Vec3i &id_g) const;

        bool isUnknown(const Vec3i &id_g) const;

        bool isKnownFree(const Vec3i &id_g) const;

        //====================================================================
        void resetCell(const int &hash_id) override;

        void probabilisticMapFromCache();

        float unknownLogOdds() const {
            return (cfg_.l_free + cfg_.l_occ) * 0.5F;
        }

        void hitPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void missPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num);

        void mixedPointUpdate(const Vec3f &pos, const int &hash_id, const int &hit_num, const int &miss_num);

        void markObservedCell(const int &hash_id);

        void updateTrackedOccupiedCell(const int &hash_id, GridType to_type);

        void applyStaleDecay();

        void raycastProcess(const PointCloud &input_cloud, const Vec3f &cur_odom);

        void insertUpdateCandidate(const Vec3i &id_g, bool is_hit);

        void updateLocalBox(const Vec3f &cur_odom);

        void resetLocalMap() override;
    };
}
