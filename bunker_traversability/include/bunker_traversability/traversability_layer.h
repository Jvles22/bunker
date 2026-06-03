#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>   // CostmapLayer = Layer + Costmap2D
#include <costmap_2d/layered_costmap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <vector>
#include <utility>

namespace bunker_traversability {

/**
 * TraversabilityLayer — costmap_2d plugin for slope-aware navigation.
 *
 * Subscribes to /bunker/elevation_map (grid_map_msgs/GridMap), reads the
 * "elevation" layer, computes the local slope angle at each cell using a
 * 4-neighbour finite-difference gradient, and writes a cost proportional to
 * slope into the master costmap.
 *
 * Design intent:
 *   - Runs AFTER ObstacleLayer in the plugin list.
 *   - Uses updateWithOverwrite(): cells left at NO_INFORMATION are transparent
 *     (obstacle_layer markings for trees / walls are preserved).
 *   - Overwrites only cells where slope >= min_slope_deg (default 5°),
 *     correcting false-positive obstacle markings on traversable ramps.
 *
 * Cost table (slope_cost_table in costmap_common_params.yaml):
 *   slope <  min_slope_deg  →  NO_INFORMATION  (transparent)
 *   slope  5°–10° →  10
 *   slope 10°–20° →  30
 *   slope 20°–30° →  50
 *   slope 30°–35° →  120
 *   slope >= lethal_slope_deg  →  LETHAL_OBSTACLE (254)
 */
class TraversabilityLayer : public costmap_2d::CostmapLayer {
public:
    TraversabilityLayer();
    virtual ~TraversabilityLayer() = default;

    // ── costmap_2d::Layer interface ──────────────────────────────────────
    virtual void onInitialize() override;
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                               double* min_x, double* min_y,
                               double* max_x, double* max_y) override;
    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
                              int min_i, int min_j,
                              int max_i, int max_j) override;
    virtual void matchSize() override;
    virtual void reset() override;

private:
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
    uint8_t slopeToCost(double slope_deg) const;

    ros::Subscriber map_sub_;
    grid_map::GridMap elevation_map_;
    bool map_received_;
    mutable std::mutex map_mutex_;

    // Parameters (loaded in onInitialize from costmap_common_params.yaml)
    std::string elevation_topic_;
    double lethal_slope_deg_;
    double min_slope_deg_;
    double update_radius_;   // max distance from robot for costmap updates (m)
    float  min_hits_;        // minimum hit count before a cell's elevation is trusted
    bool   preserve_lethal_; // if true, never downgrade a LETHAL cell from ObstacleLayer
    // Each entry: (max_slope_deg, cost). Loaded from slope_cost_table param.
    std::vector<std::pair<double, uint8_t>> slope_cost_table_;
};

}  // namespace bunker_traversability
