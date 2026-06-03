/**
 * bunker_elevation_map_node
 * =========================
 * Lightweight elevation map builder for slope-aware navigation.
 *
 * Subscribes to /velodyne_points (PointCloud2), transforms each scan to the
 * map frame, accumulates a running-mean height per grid cell, and publishes
 * the result as a grid_map_msgs/GridMap on /bunker/elevation_map.
 *
 * The TraversabilityLayer costmap plugin subscribes to this topic and derives
 * per-cell slope costs from the "elevation" layer.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <mutex>
#include <cmath>

class ElevationMapNode {
public:
    ElevationMapNode()
        : tf_listener_(tf_buffer_), initialized_(false) {
        ros::NodeHandle nh, pnh("~");

        // ── Parameters ──────────────────────────────────────────────────
        pnh.param("map_frame",    map_frame_,    std::string("map"));
        pnh.param("resolution",   resolution_,   0.15);
        pnh.param("map_length_x", map_length_x_, 70.0);  // covers 25×25 + exterior
        pnh.param("map_length_y", map_length_y_, 70.0);
        pnh.param("center_x",     center_x_,     12.5);  // centre du carré 25×25
        pnh.param("center_y",     center_y_,     12.5);
        pnh.param("min_height",   min_height_,   -1.0);
        pnh.param("max_height",   max_height_,    3.0);
        pnh.param("publish_rate", publish_rate_,  2.0);

        // ── Init grid_map (fixed, non-rolling) ──────────────────────────
        map_.setGeometry(
            grid_map::Length(map_length_x_, map_length_y_),
            resolution_,
            grid_map::Position(center_x_, center_y_)
        );
        map_.setFrameId(map_frame_);
        map_.add("elevation", NAN);
        map_.add("hits",      0.0f);

        // ── ROS interfaces ───────────────────────────────────────────────
        pc_sub_  = nh.subscribe("/velodyne_points", 1,
                                 &ElevationMapNode::pointcloudCallback, this);
        map_pub_ = nh.advertise<grid_map_msgs::GridMap>("/bunker/elevation_map",
                                                         1, /*latch=*/false);
        timer_   = nh.createTimer(
            ros::Duration(1.0 / publish_rate_),
            &ElevationMapNode::publishCallback, this
        );

        ROS_INFO("[ElevationMapNode] Map %.0f×%.0f m @ %.2f m/cell  "
                 "center=(%.1f, %.1f)  frame=%s",
                 map_length_x_, map_length_y_, resolution_,
                 center_x_, center_y_, map_frame_.c_str());
    }

private:
    // ── Point cloud callback ─────────────────────────────────────────────
    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Transform point cloud into the map frame
        sensor_msgs::PointCloud2 cloud_map;
        try {
            sensor_msgs::PointCloud2 msg_latest = *msg;
            msg_latest.header.stamp = ros::Time(0);
            tf_buffer_.transform(msg_latest, cloud_map, map_frame_, ros::Duration(0.3));
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "[ElevationMapNode] TF: %s", ex.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        pcl::fromROSMsg(cloud_map, pcl_cloud);

        std::lock_guard<std::mutex> lock(map_mutex_);

        for (const auto& pt : pcl_cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) ||
                !std::isfinite(pt.z))
                continue;
            if (pt.z < min_height_ || pt.z > max_height_) continue;

            const grid_map::Position pos(pt.x, pt.y);
            if (!map_.isInside(pos)) continue;

            grid_map::Index idx;
            if (!map_.getIndex(pos, idx)) continue;

            float& elev = map_.at("elevation", idx);
            float& hits = map_.at("hits",      idx);

            // Ground-model: keep the MINIMUM observed height per cell.
            // Rationale: a tree generates returns at multiple heights in the
            // same (x,y) column (ground bounce ≈0 m, trunk 0.3–1 m, …).
            // Running-mean elevation for such a cell would be ~0.5 m, creating
            // a fake ≈70° slope relative to the adjacent flat ground.
            // With minimum, the tree cell converges to its ground-level return
            // (≈0 m), matching the surrounding ground → slope ≈ 0° → the
            // TraversabilityLayer stays transparent there and lets the
            // ObstacleLayer keep its lethal marking intact.
            // For genuine ramp cells, all returns are on the physical surface,
            // so minimum converges to the true surface height. ✓
            hits += 1.0f;
            if (std::isnan(elev)) {
                elev = pt.z;
            } else {
                elev = std::min(elev, static_cast<float>(pt.z));
            }
        }

        initialized_ = true;
    }

    // ── Publish callback ─────────────────────────────────────────────────
    void publishCallback(const ros::TimerEvent&) {
        if (!initialized_) return;

        std::lock_guard<std::mutex> lock(map_mutex_);
        map_.setTimestamp(ros::Time::now().toNSec());

        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(map_, msg);
        map_pub_.publish(msg);
    }

    // ── Members ──────────────────────────────────────────────────────────
    ros::Subscriber pc_sub_;
    ros::Publisher  map_pub_;
    ros::Timer      timer_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    grid_map::GridMap map_;
    std::mutex        map_mutex_;
    bool              initialized_;

    std::string map_frame_;
    double resolution_, map_length_x_, map_length_y_;
    double center_x_, center_y_;
    double min_height_, max_height_, publish_rate_;
};

// ── main ─────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "bunker_elevation_map");
    ElevationMapNode node;
    ros::spin();
    return 0;
}
