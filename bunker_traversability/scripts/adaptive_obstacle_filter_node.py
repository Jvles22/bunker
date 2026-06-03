#!/usr/bin/env python3
"""adaptive_obstacle_filter_node.py

Filtre adaptatif de hauteur pour les points LiDAR.

Principe :
  - Transforme /velodyne_points dans le repère map
  - Récupère la hauteur courante du robot (z de base_footprint dans map)
  - Ne conserve que les points dans [robot_z + MIN_REL, robot_z + MAX_REL]
  - Publie le nuage filtré sur /velodyne_points_filtered (repère map)

Cela évite que la canopée et le sol loin sous le robot soient marqués
comme obstacles, et que la surface de la rampe sous le robot soit
re-marquée comme obstacle quand le robot est en hauteur.
"""

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs  # noqa: F401  (enregistre le type PointCloud2)
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import struct

# Hauteur relative au robot : fenêtre d'obstacles valides
MIN_REL_HEIGHT = 0.03   # m au-dessus du sol du robot (ignore le sol plat)
MAX_REL_HEIGHT = 1.55   # m au-dessus du sol du robot (ignore canopée + ramp surface below)

INPUT_TOPIC    = "/velodyne_points"
OUTPUT_TOPIC   = "/velodyne_points_filtered"
MAP_FRAME      = "map"
ROBOT_FRAME    = "base_footprint"
QUEUE_SIZE     = 2


class AdaptiveObstacleFilter:
    def __init__(self):
        rospy.init_node("adaptive_obstacle_filter_node")

        self.tf_buffer   = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher(OUTPUT_TOPIC, PointCloud2, queue_size=QUEUE_SIZE)
        self.sub = rospy.Subscriber(INPUT_TOPIC, PointCloud2, self.cloud_cb,
                                    queue_size=QUEUE_SIZE)

        rospy.loginfo("[adaptive_filter] Démarré : %s → %s (fenêtre [%.2f, %.2f] m / robot_z)",
                      INPUT_TOPIC, OUTPUT_TOPIC, MIN_REL_HEIGHT, MAX_REL_HEIGHT)

    # ------------------------------------------------------------------
    def _get_robot_z(self, stamp):
        """Retourne la coordonnée z de base_footprint dans map, ou None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, ROBOT_FRAME, stamp,
                timeout=rospy.Duration(0.3)
            )
            return tf.transform.translation.z
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "[adaptive_filter] TF robot_z indisponible : %s", e)
            return None

    # ------------------------------------------------------------------
    def cloud_cb(self, msg: PointCloud2):
        # 1. Transformer le nuage dans map en utilisant la dernière TF disponible
        #    (rospy.Time(0) = latest) pour éviter les désynchronisations sim_time
        try:
            msg.header.stamp = rospy.Time(0)
            cloud_map = self.tf_buffer.transform(msg, MAP_FRAME,
                                                  timeout=rospy.Duration(0.3))
        except Exception as e:
            rospy.logwarn_throttle(5.0, "[adaptive_filter] TF nuage indisponible : %s", e)
            return

        # 2. Hauteur du robot dans map (latest TF)
        robot_z = self._get_robot_z(rospy.Time(0))
        if robot_z is None:
            return

        z_min = robot_z + MIN_REL_HEIGHT
        z_max = robot_z + MAX_REL_HEIGHT

        # 3. Filtrer les points
        kept = []
        for pt in pc2.read_points(cloud_map, field_names=("x", "y", "z"),
                                   skip_nans=True):
            if z_min <= pt[2] <= z_max:
                kept.append(pt)

        # 4. Reconstruire et publier le nuage filtré
        #    Estampiller avec le temps courant pour que le costmap le voit comme frais
        out = pc2.create_cloud_xyz32(cloud_map.header, kept)
        out.header.stamp = rospy.Time.now()
        self.pub.publish(out)


# ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        node = AdaptiveObstacleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
