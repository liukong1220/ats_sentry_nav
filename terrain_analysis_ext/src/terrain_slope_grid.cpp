#include "terrain_slope_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

double clampUnit(double value) {
  return std::max(0.0, std::min(1.0, value));
}

int8_t normalizedToOccupancy(double value) {
  return static_cast<int8_t>(std::round(clampUnit(value) * 100.0));
}

// Convert a continuous slope angle into a few coarse bands so RViz can quickly
// show "gentle / moderate / steep" terrain without requiring custom messages.
double slopeDegToBand(double slope_deg) {
  if (slope_deg >= slopeSteepDegThre) {
    return 1.0;
  }
  if (slope_deg >= slopeModerateDegThre) {
    return 0.67;
  }
  if (slope_deg >= slopeGentleDegThre) {
    return 0.34;
  }
  return 0.0;
}

// Estimate the local slope from neighboring ground elevations on the 2D planar
// grid. This keeps V1 in a 2D navigation topology while still exposing 2.5D
// terrain semantics to later speed planning and obstacle policies.
void computeSlopeGrid() {
  const double resolution = std::max(static_cast<double>(planarVoxelSize), 1e-3);
  // Keep a physical >=0.20 m horizontal baseline for slope estimation so unified
  // 5 cm planar pixels do not amplify 2-3 cm LiDAR z-jitter into false >28 deg walls.
  const int stride = std::max(1, static_cast<int>(std::lround(0.20 / resolution)));
  const double baseline = resolution * static_cast<double>(stride);

  for (int indX = 0; indX < planarVoxelWidth; ++indX) {
    for (int indY = 0; indY < planarVoxelWidth; ++indY) {
      const int ind = planarVoxelWidth * indX + indY;
      const bool has_points =
          planarVoxelPointCount[ind] >= traversabilityMinPointCount;
      const bool is_connected = !checkTerrainConn || planarVoxelConn[ind] == 2;
      if (!has_points || !is_connected) {
        planarVoxelSlopeDeg[ind] = 0.0f;
        planarVoxelSlopeBand[ind] = 0.0f;
        continue;
      }

      const auto sample_elev = [&](int sx, int sy, float &elev) -> bool {
        if (sx < 0 || sx >= planarVoxelWidth || sy < 0 || sy >= planarVoxelWidth) {
          return false;
        }
        const int sample_ind = planarVoxelWidth * sx + sy;
        const bool sample_has_points =
            planarVoxelPointCount[sample_ind] >= traversabilityMinPointCount;
        const bool sample_is_connected =
            !checkTerrainConn || planarVoxelConn[sample_ind] == 2;
        if (!sample_has_points || !sample_is_connected) {
          return false;
        }
        elev = planarVoxelElev[sample_ind];
        return true;
      };

      const float center_elev = planarVoxelElev[ind];
      const auto sample_continuous_elev = [&](int dx, int dy, float &elev) -> bool {
        // Walk from the center cell to the stride endpoint and reject any ray that
        // crosses a vertical wall/step discontinuity (> 0.08 m in a single cell or a
        // flat local cell suddenly jumping > 0.14 m at the stride endpoint). This keeps
        // genuine continuous ramps (<= 58 deg) intact while preventing vertical tunnel
        // sidewalls from smearing 0.20 m of false >28 deg slope onto adjacent flat floors.
        float prev = center_elev;
        const float max_single_step =
            static_cast<float>(std::max(0.08, 1.2 * resolution));
        for (int k = 1; k <= stride; ++k) {
          float cur = 0.0f;
          if (!sample_elev(indX + dx * k, indY + dy * k, cur)) {
            if (k == stride) {
              return false;
            }
            continue;
          }
          if (std::fabs(cur - prev) > max_single_step) {
            return false;
          }
          prev = cur;
        }
        elev = prev;
        return true;
      };

      float left = 0.0f;
      float right = 0.0f;
      float down = 0.0f;
      float up = 0.0f;
      const bool has_left = sample_continuous_elev(-1, 0, left);
      const bool has_right = sample_continuous_elev(1, 0, right);
      const bool has_down = sample_continuous_elev(0, -1, down);
      const bool has_up = sample_continuous_elev(0, 1, up);

      double dzdx = 0.0;
      double dzdy = 0.0;
      bool valid_dx = false;
      bool valid_dy = false;

      if (has_left && has_right) {
        dzdx = (static_cast<double>(right) - static_cast<double>(left)) /
               (2.0 * baseline);
        valid_dx = true;
      } else if (has_right) {
        dzdx = (static_cast<double>(right) -
                static_cast<double>(planarVoxelElev[ind])) /
               baseline;
        valid_dx = true;
      } else if (has_left) {
        dzdx = (static_cast<double>(planarVoxelElev[ind]) -
                static_cast<double>(left)) /
               baseline;
        valid_dx = true;
      }

      if (has_down && has_up) {
        dzdy = (static_cast<double>(up) - static_cast<double>(down)) /
               (2.0 * baseline);
        valid_dy = true;
      } else if (has_up) {
        dzdy = (static_cast<double>(up) -
                static_cast<double>(planarVoxelElev[ind])) /
               baseline;
        valid_dy = true;
      } else if (has_down) {
        dzdy = (static_cast<double>(planarVoxelElev[ind]) -
                static_cast<double>(down)) /
               baseline;
        valid_dy = true;
      }

      if (!valid_dx && !valid_dy) {
        planarVoxelSlopeDeg[ind] = 0.0f;
        planarVoxelSlopeBand[ind] = 0.0f;
        continue;
      }

      const double gradient_norm = std::sqrt(dzdx * dzdx + dzdy * dzdy);
      const double slope_deg = std::atan(gradient_norm) * 180.0 / M_PI;
      planarVoxelSlopeDeg[ind] = static_cast<float>(slope_deg);
      planarVoxelSlopeBand[ind] = static_cast<float>(slopeDegToBand(slope_deg));
    }
  }
}

nav_msgs::msg::OccupancyGrid makePlanarGridMessage(const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = "odom";
  grid.info.map_load_time = stamp;
  grid.info.resolution = planarVoxelSize;
  grid.info.width = planarVoxelWidth;
  grid.info.height = planarVoxelWidth;
  grid.info.origin.position.x =
      planarAnchorX - (planarVoxelHalfWidth + 0.5) * planarVoxelSize;
  grid.info.origin.position.y =
      planarAnchorY - (planarVoxelHalfWidth + 0.5) * planarVoxelSize;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(planarVoxelNum, -1);
  return grid;
}

std::size_t planarIndexToOccupancyGridIndex(int planar_ind) {
  const int ind_x = planar_ind / planarVoxelWidth;
  const int ind_y = planar_ind % planarVoxelWidth;
  return static_cast<std::size_t>(ind_y) *
             static_cast<std::size_t>(planarVoxelWidth) +
         static_cast<std::size_t>(ind_x);
}

void publishScalarGrid(
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher,
    const rclcpp::Time &stamp, const std::vector<float> &values,
    double scale_limit) {
  if (!publisher || scale_limit <= 0.0) {
    return;
  }

  nav_msgs::msg::OccupancyGrid grid = makePlanarGridMessage(stamp);
  for (int i = 0; i < planarVoxelNum; ++i) {
    const bool has_points = planarVoxelPointCount[i] >= traversabilityMinPointCount;
    if (!has_points) {
      continue;
    }
    grid.data[planarIndexToOccupancyGridIndex(i)] =
        normalizedToOccupancy(static_cast<double>(values[i]) / scale_limit);
  }

  publisher->publish(grid);
}

void publishTraversabilityGrid(
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr &publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &height_diff_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &occupancy_ratio_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &ground_confidence_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &slope_publisher,
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        &slope_band_publisher,
    const rclcpp::Time &stamp) {
  nav_msgs::msg::OccupancyGrid grid = makePlanarGridMessage(stamp);

  const double height_range = std::max(
      traversabilityObstacleHeightThre - traversabilitySafeHeightThre, 1e-3);
  const double weight_sum =
      std::max(traversabilityHeightWeight + traversabilityOccupancyWeight +
                   traversabilityGroundWeight,
               1e-6);

  for (int i = 0; i < planarVoxelNum; ++i) {
    const std::size_t grid_ind = planarIndexToOccupancyGridIndex(i);
    const bool has_points = planarVoxelPointCount[i] >= traversabilityMinPointCount;
    const bool is_connected = !checkTerrainConn || planarVoxelConn[i] == 2;
    if (!has_points) {
      grid.data[grid_ind] =
          traversabilityUnknownAsOccupied ? 100 : -1;
      continue;
    }

    const double point_confidence = clampUnit(
        static_cast<double>(planarVoxelPointCount[i]) /
        static_cast<double>(std::max(traversabilityMinPointCount * 2, 1)));
    const double connectivity_confidence =
        !checkTerrainConn ? 1.0 : (is_connected ? 1.0 : 0.0);
    const double ground_confidence =
        0.6 * point_confidence + 0.4 * connectivity_confidence;
    planarVoxelGroundConfidence[i] = static_cast<float>(ground_confidence);

    const double height_metric = static_cast<double>(
        terrain_analysis_ext::voxelHeightMetric(
            planarVoxelMinZ[i], planarVoxelMaxZ[i], planarVoxelMaxRelHeight[i]));
    const double height_score = clampUnit(
        (height_metric - traversabilitySafeHeightThre) / height_range);
    const double occupancy_activation = clampUnit(
        (height_metric - 0.5 * traversabilitySafeHeightThre) /
        std::max(traversabilityObstacleHeightThre - 0.5 * traversabilitySafeHeightThre,
                 1e-3));
    const double occupancy_score =
        occupancy_activation *
        clampUnit(planarVoxelOccupancyRatio[i] /
                  std::max(traversabilityOccupancyRatioThre, 1e-3));
    const double ground_penalty = clampUnit(
        (traversabilityGroundConfidenceThre - ground_confidence) /
        std::max(traversabilityGroundConfidenceThre, 1e-3));
    const double slope_deg = static_cast<double>(planarVoxelSlopeDeg[i]);
    const bool slope_blocked =
        useSlopeAsObstacle && slope_deg >= slopeObstacleDegThre;

    // V1 default: slope is published as terrain semantics for later velocity
    // adaptation. If `useSlopeAsObstacle` is enabled, steep cells are upgraded
    // to hard obstacles here so the 2D planner will route around them directly.
    double combined_score =
        (traversabilityHeightWeight * height_score +
         traversabilityOccupancyWeight * occupancy_score +
         traversabilityGroundWeight * ground_penalty) /
        weight_sum;
    if (!is_connected || height_metric >= traversabilityObstacleHeightThre ||
        slope_blocked) {
      combined_score = 1.0;
    }

    grid.data[grid_ind] = normalizedToOccupancy(combined_score);
  }

  publisher->publish(grid);
  publishScalarGrid(slope_publisher, stamp, planarVoxelSlopeDeg,
                    std::max(slopeGridMaxDeg, 1e-3));
  if (slope_band_publisher && slope_band_publisher->get_subscription_count() > 0U) {
    publishScalarGrid(slope_band_publisher, stamp, planarVoxelSlopeBand, 1.0);
  }
  if (publishTraversabilityDebugGrids) {
    if (height_diff_publisher && height_diff_publisher->get_subscription_count() > 0U) {
      publishScalarGrid(height_diff_publisher, stamp, planarVoxelHeightDiff,
                        traversabilityDebugHeightDiffCap);
    }
    if (occupancy_ratio_publisher && occupancy_ratio_publisher->get_subscription_count() > 0U) {
      publishScalarGrid(occupancy_ratio_publisher, stamp, planarVoxelOccupancyRatio,
                        traversabilityDebugOccupancyRatioCap);
    }
    if (ground_confidence_publisher && ground_confidence_publisher->get_subscription_count() > 0U) {
      publishScalarGrid(ground_confidence_publisher, stamp,
                        planarVoxelGroundConfidence, 1.0);
    }
  }
}

// Read the planar lattice parameters, clamp them to a usable centred lattice and
// size every planar buffer from the result. Kept out of main() so the entry point
// stays inside the readability/fn_size budget.
