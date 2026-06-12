/**
 * adaptive_obstacle_filter_node (C++)
 * ====================================
 * Filtre adaptatif de hauteur pour les points LiDAR, avec correction
 * d'inclinaison du sol via IMU.
 *
 * Principe :
 *   - Transforme /velodyne_points dans le repère map (TF au temps "latest")
 *   - Récupère robot_z = hauteur de base_footprint dans map
 *   - Récupère l'orientation IMU (quaternion, base_link == imu_link, offset nul)
 *   - Calcule la normale du plan "sol local" = (0,0,1) tournée par le quaternion IMU
 *   - Pour chaque point (x,y,z), calcule z_ground(x,y) = altitude du plan incliné
 *     passant par (robot_x, robot_y, robot_z) avec cette normale
 *   - Ne conserve que les points dans [z_ground + min_rel, z_ground + max_rel]
 *   - Publie le nuage filtré sur /velodyne_points_filtered (repère map)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <mutex>

class AdaptiveObstacleFilter {
public:
    AdaptiveObstacleFilter()
        : tf_listener_(tf_buffer_), have_imu_(false) {
        ros::NodeHandle nh, pnh("~");

        pnh.param("min_rel_height", min_rel_height_, 0.03);
        pnh.param("max_rel_height", max_rel_height_, 1.55);
        pnh.param("map_frame",      map_frame_,   std::string("map"));
        pnh.param("robot_frame",    robot_frame_, std::string("base_footprint"));
        pnh.param("imu_topic",      imu_topic_,   std::string("/imu/data"));

        // Minimum |normal.z| before falling back to the flat-plane model.
        // Guards against numerical blow-up near 90° tilt (should never
        // happen in practice, but keeps the node robust).
        pnh.param("min_normal_z",   min_normal_z_, 0.2);

        pub_ = nh.advertise<sensor_msgs::PointCloud2>(
                    "/velodyne_points_filtered", 2);
        sub_ = nh.subscribe("/velodyne_points", 2,
                             &AdaptiveObstacleFilter::cloudCb, this);
        sub_imu_ = nh.subscribe(imu_topic_, 50,
                             &AdaptiveObstacleFilter::imuCb, this);

        ROS_INFO("[AdaptiveFilter] Demarre (C++) : fenetre [%.2f, %.2f] m "
                 "/ plan incline (IMU: %s)",
                 min_rel_height_, max_rel_height_, imu_topic_.c_str());
    }

private:
    // ── Callback IMU ─────────────────────────────────────────────────────
    // imu_link == base_link, offset extrinsèque nul → le quaternion IMU
    // représente directement l'orientation de base_link/base_footprint
    // dans le monde. Pas de correction extrinsèque à appliquer.
    void imuCb(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        last_imu_orientation_ = msg->orientation;
        have_imu_ = true;
    }

    // ── Callback principal ──────────────────────────────────────────────────
    void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {

        // 1. Position du robot dans map
        double robot_x = 0.0, robot_y = 0.0, robot_z = 0.0;
        try {
            const auto tf_robot = tf_buffer_.lookupTransform(
                map_frame_, robot_frame_,
                ros::Time(0), ros::Duration(0.1));
            robot_x = tf_robot.transform.translation.x;
            robot_y = tf_robot.transform.translation.y;
            robot_z = tf_robot.transform.translation.z;
        } catch (const tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(5.0, "[AdaptiveFilter] TF robot indisponible : %s",
                              ex.what());
            return;
        }

        // 2. Normale du plan "sol local" = (0,0,1) tournée par l'orientation IMU.
        //    Fallback sur le plan horizontal (ancien comportement) si l'IMU
        //    n'a encore rien publié.
        double nx = 0.0, ny = 0.0, nz = 1.0;
        bool have_imu_now;
        sensor_msgs::Imu::_orientation_type orientation;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            have_imu_now = have_imu_;
            orientation  = last_imu_orientation_;
        }
        if (have_imu_now) {
            tf2::Quaternion q(orientation.x, orientation.y,
                              orientation.z, orientation.w);
            q.normalize();
            const tf2::Vector3 normal_robot(0.0, 0.0, 1.0);
            const tf2::Vector3 normal_map = tf2::quatRotate(q, normal_robot);

            // Guard against near-degenerate planes (nz ~ 0, e.g. robot on
            // its side). Fall back to flat-plane model in that case.
            if (std::fabs(normal_map.z()) >= min_normal_z_) {
                nx = normal_map.x();
                ny = normal_map.y();
                nz = normal_map.z();
            }
        }

        // 3. Transformer le nuage dans map (TF la plus récente, pas le timestamp
        //    du message — même comportement que l'ancienne version)
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

        // 4. Convertir en PCL et filtrer en C++ avec le plan incline
        pcl::PointCloud<pcl::PointXYZ> pcl_in, pcl_out;
        pcl::fromROSMsg(cloud_map, pcl_in);
        pcl_out.reserve(pcl_in.size());

        for (const auto& pt : pcl_in.points) {
            if (!std::isfinite(pt.z)) continue;

            // z_ground(x,y) = robot_z - (nx*(x-robot_x) + ny*(y-robot_y)) / nz
            const double z_ground = robot_z
                - (nx * (pt.x - robot_x) + ny * (pt.y - robot_y)) / nz;

            const double z_min = z_ground + min_rel_height_;
            const double z_max = z_ground + max_rel_height_;

            if (pt.z >= z_min && pt.z <= z_max)
                pcl_out.push_back(pt);
        }

        // 5. Publier le nuage filtre
        sensor_msgs::PointCloud2 out_msg;
        pcl::toROSMsg(pcl_out, out_msg);
        out_msg.header          = cloud_map.header;   // frame_id = map
        out_msg.header.stamp    = ros::Time::now();   // timestamp frais pour le costmap

        pub_.publish(out_msg);
    }

    // ── Membres ─────────────────────────────────────────────────────────────
    ros::Subscriber sub_;
    ros::Subscriber sub_imu_;
    ros::Publisher  pub_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double      min_rel_height_, max_rel_height_;
    double      min_normal_z_;
    std::string map_frame_, robot_frame_, imu_topic_;

    std::mutex                       imu_mutex_;
    bool                              have_imu_;
    sensor_msgs::Imu::_orientation_type last_imu_orientation_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "adaptive_obstacle_filter");
    AdaptiveObstacleFilter node;
    ros::spin();
    return 0;
}