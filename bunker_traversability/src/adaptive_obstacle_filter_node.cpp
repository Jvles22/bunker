/**
 * adaptive_obstacle_filter_node (C++)
 * ====================================
 * Filtre adaptatif de hauteur pour les points LiDAR.
 *
 * Principe (identique à l'ancienne version Python, sans le GIL) :
 *   - Transforme /velodyne_points dans le repère map (TF au temps "latest")
 *   - Récupère robot_z = hauteur de base_footprint dans map
 *   - Ne conserve que les points dans [robot_z + min_rel, robot_z + max_rel]
 *   - Publie le nuage filtré sur /velodyne_points_filtered (repère map)
 *
 * Gain vs Python : suppression de la boucle Python point-par-point (~30 k
 * itérations / scan @ 10 Hz), des allocations de tuples et du GIL.
 * La boucle C++ est ≈ 20-50× plus rapide ; pas de retard accumulé.
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
#include <cmath>

class AdaptiveObstacleFilter {
public:
    AdaptiveObstacleFilter()
        : tf_listener_(tf_buffer_) {
        ros::NodeHandle nh, pnh("~");

        pnh.param("min_rel_height", min_rel_height_, 0.03);
        pnh.param("max_rel_height", max_rel_height_, 1.55);
        pnh.param("map_frame",      map_frame_,   std::string("map"));
        pnh.param("robot_frame",    robot_frame_, std::string("base_footprint"));

        pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                    "/velodyne_points_filtered", 2);
        sub_ = nh.subscribe("/velodyne_points", 2,
                             &AdaptiveObstacleFilter::cloudCb, this);

        ROS_INFO("[AdaptiveFilter] Démarré (C++) : fenêtre [%.2f, %.2f] m / robot_z",
                 min_rel_height_, max_rel_height_);
    }

private:
    // ── Callback principal ──────────────────────────────────────────────────
    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {

        // 1. Hauteur du robot dans map (TF la plus récente — sûr avec sim_time)
        double robot_z = 0.0;
        try {
            const auto tf_robot = tf_buffer_.lookupTransform(
                map_frame_, robot_frame_,
                ros::Time(0), ros::Duration(0.1));
            robot_z = tf_robot.transform.translation.z;
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "[AdaptiveFilter] TF robot_z indisponible : %s",
                              ex.what());
            return;
        }

        // 2. Transformer le nuage dans map (TF la plus récente, pas le timestamp
        //    du message — même comportement que l'ancienne version Python)
        sensor_msgs::PointCloud2 cloud_map;
        try {
            const auto tf_cloud = tf_buffer_.lookupTransform(
                map_frame_, msg->header.frame_id,
                ros::Time(0), ros::Duration(0.1));
            tf2::doTransform(*msg, cloud_map, tf_cloud);
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "[AdaptiveFilter] TF nuage indisponible : %s",
                              ex.what());
            return;
        }

        // 3. Convertir en PCL et filtrer en C++ (pas de GIL, pas de boucle Python)
        pcl::PointCloud<pcl::PointXYZ> pcl_in, pcl_out;
        pcl::fromROSMsg(cloud_map, pcl_in);

        const float z_min = static_cast<float>(robot_z + min_rel_height_);
        const float z_max = static_cast<float>(robot_z + max_rel_height_);

        pcl_out.reserve(pcl_in.size());
        for (const auto& pt : pcl_in.points) {
            if (!std::isfinite(pt.z)) continue;
            if (pt.z >= z_min && pt.z <= z_max)
                pcl_out.push_back(pt);
        }

        // 4. Publier le nuage filtré
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(pcl_out, out_msg);
        out_msg.header          = cloud_map.header;   // frame_id = map
        out_msg.header.stamp    = ros::Time::now();   // timestamp frais pour le costmap

        pub_.publish(out_msg);
    }

    // ── Membres ─────────────────────────────────────────────────────────────
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double      min_rel_height_, max_rel_height_;
    std::string map_frame_, robot_frame_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "adaptive_obstacle_filter");
    AdaptiveObstacleFilter node;
    ros::spin();
    return 0;
}
