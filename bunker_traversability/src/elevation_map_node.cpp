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
        pnh.param("min_height",        min_height_,        -1.0);
        pnh.param("max_height",        max_height_,         3.0);
        pnh.param("publish_rate",      publish_rate_,       2.0);
        // Décroissance temporelle :
        //   decay_time_ground : terrain — réinitialise après N s sans scan
        //   decay_time_max    : objets   — réinitialise plus vite (obstacles mobiles)
        pnh.param("decay_time_ground", decay_time_ground_, 90.0);
        pnh.param("decay_time_max",    decay_time_max_,    20.0);

        // ── Init grid_map (fixed, non-rolling) ──────────────────────────
        map_.setGeometry(
            grid_map::Length(map_length_x_, map_length_y_),
            resolution_,
            grid_map::Position(center_x_, center_y_)
        );
        map_.setFrameId(map_frame_);
        map_.add("elevation",          NAN);   // min vu  → sol (terrain)
        map_.add("elevation_max",      NAN);   // max vu  → surface (objets inclus)
        map_.add("hits",               0.0f);
        map_.add("last_update_ground", 0.0f);  // temps relatif dernière mise à jour sol
        map_.add("last_update_max",    0.0f);  // temps relatif dernière mise à jour max

        start_time_ = ros::Time::now();

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

            float& elev     = map_.at("elevation",          idx);
            float& elev_max = map_.at("elevation_max",      idx);
            float& hits     = map_.at("hits",               idx);
            float& last_g   = map_.at("last_update_ground", idx);
            float& last_m   = map_.at("last_update_max",    idx);

            // Temps écoulé depuis le démarrage du nœud (float suffisant pour <3600 s).
            const float t = static_cast<float>((ros::Time::now() - start_time_).toSec());
            last_g = t;  // rafraîchit la cellule — réinitialisation decay clock
            last_m = t;
            hits  += 1.0f;

            // Sol (minimum) : converge vers la surface réelle du terrain.
            // Un arbre génère des retours à plusieurs hauteurs ; le minimum
            // donnera finalement le retour sol (≈0 m) → pente ≈ 0° → la
            // TraversabilityLayer calcule correctement la traversabilité.
            if (std::isnan(elev))
                elev = pt.z;
            else
                elev = std::min(elev, static_cast<float>(pt.z));

            // Surface max : maximum vu dans la fenêtre temporelle.
            // delta = elevation_max - elevation → détecte les objets au-dessus du sol.
            if (std::isnan(elev_max))
                elev_max = pt.z;
            else
                elev_max = std::max(elev_max, static_cast<float>(pt.z));
        }

        initialized_ = true;
    }

    // ── Publish callback ─────────────────────────────────────────────────
    void publishCallback(const ros::TimerEvent&) {
        if (!initialized_) return;

        std::lock_guard<std::mutex> lock(map_mutex_);

        // Décroissance temporelle : réinitialise les cellules non observées récemment.
        // Les cellules terrain (elevation) expirent après decay_time_ground_ secondes.
        // Les cellules objets (elevation_max) expirent après decay_time_max_ secondes.
        const float t_now   = static_cast<float>((ros::Time::now() - start_time_).toSec());
        const float decay_g = static_cast<float>(decay_time_ground_);
        const float decay_m = static_cast<float>(decay_time_max_);

        for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
            const float lg = map_.at("last_update_ground", *it);
            if (lg > 0.0f && (t_now - lg) > decay_g) {
                map_.at("elevation",          *it) = NAN;
                map_.at("hits",               *it) = 0.0f;
                map_.at("last_update_ground", *it) = 0.0f;
            }
            const float lm = map_.at("last_update_max", *it);
            if (lm > 0.0f && (t_now - lm) > decay_m) {
                map_.at("elevation_max",    *it) = NAN;
                map_.at("last_update_max",  *it) = 0.0f;
            }
        }

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

    ros::Time   start_time_;

    std::string map_frame_;
    double resolution_, map_length_x_, map_length_y_;
    double center_x_, center_y_;
    double min_height_, max_height_, publish_rate_;
    double decay_time_ground_, decay_time_max_;
};

// ── main ─────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "bunker_elevation_map");
    ElevationMapNode node;
    ros::spin();
    return 0;
}
