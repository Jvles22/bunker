#!/usr/bin/env python3
"""
Navigation autonome par waypoints pour le robot Bunker.
VFH avec détour dynamique progressif.
"""

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2

# ==================== WAYPOINTS (coordonnées Gazebo) ====================
SPAWN_X = 0.0
SPAWN_Y = -23.0

WAYPOINTS = [
    (  0.0,  -18.0, "Entree foret"),
    ( 12.0,  -12.0, "Quadrant sud-est"),
    ( 18.0,    0.0, "Bordure est"),
    ( 12.0,   12.0, "Quadrant nord-est"),
    (  0.0,   18.0, "Bordure nord"),
    (-12.0,   12.0, "Quadrant nord-ouest"),
    (-18.0,    0.0, "Bordure ouest"),
    (-12.0,  -12.0, "Quadrant sud-ouest"),
    (  0.0,  -15.0, "Anneau intermediaire sud"),
    ( 10.0,    0.0, "Anneau intermediaire est"),
    (  0.0,   10.0, "Anneau intermediaire nord"),
    (-10.0,    0.0, "Anneau intermediaire ouest"),
    (  0.0,    0.0, "Centre de la foret"),
    (  0.0,  -23.0, "Retour spawn"),
]

# ==================== PARAMETRES ====================
DIST_THRESHOLD   = 1.2
ANGLE_THRESHOLD  = 0.08
LINEAR_SPEED     = 0.35
ANGULAR_SPEED    = 0.6
KP_ANGULAR       = 1.5
MAX_RETRIES      = 3
STUCK_TIMEOUT    = 12.0
STUCK_DIST       = 0.25

# VFH
OBSTACLE_DIST       = 1.5
VFH_SECTORS         = 72
VFH_THRESHOLD       = 0.2
LIDAR_MIN_Z         = -0.1
LIDAR_MAX_Z         = 0.3
VFH_MIN_DETOUR      = 6    # détour minimum initial = 30° (6 secteurs x 5°)
VFH_MAX_DETOUR      = 18   # détour maximum = 90°
VFH_DETOUR_INCREMENT = 3   # +15° à chaque blocage

class BunkerNavigator:
    def __init__(self):
        rospy.init_node('bunker_navigator', anonymous=False)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.origin_x = None
        self.origin_y = None

        self.vfh_histogram = np.zeros(VFH_SECTORS)
        self.obstacle_ahead = False
        self.lidar_received = False

        # Détour dynamique — augmente progressivement si bloqué
        self.consecutive_blocks = 0
        self.dynamic_detour = VFH_MIN_DETOUR

        self.current_wp = 0
        self.retries = 0
        self.last_progress_time = rospy.Time.now()
        self.last_progress_x = 0.0
        self.last_progress_y = 0.0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/lio_sam/mapping/odometry', Odometry,
                         self.odom_callback, queue_size=10)
        rospy.Subscriber('/velodyne_points', PointCloud2,
                         self.lidar_callback, queue_size=1)

        self.rate = rospy.Rate(20)
        rospy.loginfo("=== Bunker Navigator VFH dynamique démarré ===")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.odom_received = True

    def lidar_callback(self, msg):
        histogram = np.zeros(VFH_SECTORS)
        try:
            for point in pc2.read_points(msg, field_names=("x", "y", "z"),
                                          skip_nans=True):
                px, py, pz = point
                if pz < LIDAR_MIN_Z or pz > LIDAR_MAX_Z:
                    continue
                dist = math.sqrt(px**2 + py**2)
                if dist > OBSTACLE_DIST or dist < 0.15:
                    continue
                angle_deg = math.degrees(math.atan2(py, px)) % 360
                sector = int(angle_deg / (360.0 / VFH_SECTORS)) % VFH_SECTORS
                weight = (OBSTACLE_DIST - dist) / OBSTACLE_DIST
                histogram[sector] += weight
        except Exception as e:
            rospy.logwarn_throttle(5, f"Lidar error: {e}")

        self.vfh_histogram = histogram

        # Zone frontale ±45°
        front_sectors = list(range(0, 10)) + list(range(VFH_SECTORS - 9, VFH_SECTORS))
        self.obstacle_ahead = any(
            self.vfh_histogram[s] > VFH_THRESHOLD for s in front_sectors
        )
        self.lidar_received = True

    def vfh_best_direction(self, target_angle_robot, min_detour):
        """
        Cherche le secteur libre le plus proche du waypoint,
        avec un détour minimum imposé (min_detour secteurs).
        Si rien trouvé avec contrainte → fallback sans contrainte.
        """
        target_deg = math.degrees(target_angle_robot) % 360
        target_sector = int(target_deg / (360.0 / VFH_SECTORS)) % VFH_SECTORS

        best_sector = None
        best_cost = float('inf')

        # 1ère passe — avec détour minimum imposé
        for s in range(VFH_SECTORS):
            if self.vfh_histogram[s] <= VFH_THRESHOLD:
                diff = abs(s - target_sector)
                diff = min(diff, VFH_SECTORS - diff)
                if self.obstacle_ahead and diff < min_detour:
                    continue
                if diff < best_cost:
                    best_cost = diff
                    best_sector = s

        # 2ème passe — fallback sans contrainte si rien trouvé
        if best_sector is None:
            best_cost = float('inf')
            for s in range(VFH_SECTORS):
                if self.vfh_histogram[s] <= VFH_THRESHOLD:
                    diff = abs(s - target_sector)
                    diff = min(diff, VFH_SECTORS - diff)
                    if diff < best_cost:
                        best_cost = diff
                        best_sector = s

        if best_sector is None:
            return None

        best_deg = best_sector * (360.0 / VFH_SECTORS)
        best_rad = math.radians(best_deg)
        if best_rad > math.pi:
            best_rad -= 2 * math.pi
        return best_rad

    def gazebo_to_liosam(self, gx, gy):
        return (self.origin_x + gx - SPAWN_X,
                self.origin_y + gy - SPAWN_Y)

    def distance_to(self, tx, ty):
        return math.sqrt((tx - self.x)**2 + (ty - self.y)**2)

    def angle_to_world(self, tx, ty):
        desired = math.atan2(ty - self.y, tx - self.x)
        error = desired - self.yaw
        while error >  math.pi: error -= 2 * math.pi
        while error < -math.pi: error += 2 * math.pi
        return error

    def stop(self):
        self.cmd_pub.publish(Twist())

    def is_stuck(self):
        dist = math.sqrt(
            (self.x - self.last_progress_x)**2 +
            (self.y - self.last_progress_y)**2
        )
        elapsed = (rospy.Time.now() - self.last_progress_time).to_sec()
        if dist > STUCK_DIST:
            self.last_progress_x = self.x
            self.last_progress_y = self.y
            self.last_progress_time = rospy.Time.now()
            return False
        return elapsed > STUCK_TIMEOUT

    def navigate_to(self, tx, ty, description):
        rospy.loginfo(
            f"→ WP [{self.current_wp+1}/{len(WAYPOINTS)}] : "
            f"{description} → ({tx:.1f}, {ty:.1f})"
        )
        self.last_progress_time = rospy.Time.now()
        self.last_progress_x = self.x
        self.last_progress_y = self.y
        self.consecutive_blocks = 0
        self.dynamic_detour = VFH_MIN_DETOUR

        while not rospy.is_shutdown():
            dist         = self.distance_to(tx, ty)
            angle_target = self.angle_to_world(tx, ty)

            if dist < DIST_THRESHOLD:
                rospy.loginfo(f"✅ Atteint : {description}")
                self.stop()
                return True

            if self.is_stuck():
                rospy.logwarn(f"⚠️  Bloqué vers {description}")
                self.stop()
                return False

            cmd = Twist()

            if self.lidar_received and self.obstacle_ahead:
                best_angle = self.vfh_best_direction(angle_target, self.dynamic_detour)

                if best_angle is None:
                    # Aucune voie libre — reculer et augmenter le détour
                    rospy.logwarn_throttle(2, "VFH : aucune voie libre, recul")
                    cmd.linear.x = -0.2
                    cmd.angular.z = 0.4
                    self.consecutive_blocks += 1
                    self.dynamic_detour = min(
                        VFH_MAX_DETOUR,
                        VFH_MIN_DETOUR + self.consecutive_blocks * VFH_DETOUR_INCREMENT
                    )
                else:
                    detour_deg = self.dynamic_detour * (360.0 / VFH_SECTORS)
                    rospy.loginfo_throttle(2,
                        f"VFH : cible={math.degrees(angle_target):.0f}° "
                        f"→ choisi={math.degrees(best_angle):.0f}° "
                        f"(détour min={detour_deg:.0f}°)"
                    )
                    # Voie trouvée → réinitialiser le compteur de blocages
                    self.consecutive_blocks = 0
                    self.dynamic_detour = VFH_MIN_DETOUR

                    cmd.angular.z = max(-ANGULAR_SPEED,
                                        min(ANGULAR_SPEED, KP_ANGULAR * best_angle))
                    front_density = max(
                        self.vfh_histogram[s]
                        for s in list(range(0, 4)) + list(range(VFH_SECTORS-3, VFH_SECTORS))
                    )
                    speed_factor = max(0.1, 1.0 - front_density * 2)
                    cmd.linear.x = LINEAR_SPEED * speed_factor

            else:
                # Navigation directe vers waypoint
                if abs(angle_target) > ANGLE_THRESHOLD:
                    cmd.angular.z = max(-ANGULAR_SPEED,
                                        min(ANGULAR_SPEED, KP_ANGULAR * angle_target))
                    if abs(angle_target) < math.pi / 4:
                        cmd.linear.x = LINEAR_SPEED * (1 - abs(angle_target)/(math.pi/4))
                else:
                    cmd.linear.x = LINEAR_SPEED
                    cmd.angular.z = KP_ANGULAR * angle_target * 0.5

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

        self.stop()
        return False

    def run(self):
        rospy.loginfo("En attente de l'odométrie LIO-SAM...")
        timeout = rospy.Time.now() + rospy.Duration(30)
        while not self.odom_received and not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logerr("Timeout : pas d'odométrie reçue !")
                return
            rospy.sleep(0.5)

        rospy.loginfo("En attente du LiDAR...")
        timeout2 = rospy.Time.now() + rospy.Duration(15)
        while not self.lidar_received and not rospy.is_shutdown():
            if rospy.Time.now() > timeout2:
                rospy.logwarn("Timeout LiDAR — navigation sans évitement")
                break
            rospy.sleep(0.5)

        self.origin_x = self.x
        self.origin_y = self.y
        rospy.loginfo(f"Origine LIO-SAM : ({self.origin_x:.2f}, {self.origin_y:.2f})")
        rospy.loginfo("Démarrage dans 3 secondes...")
        rospy.sleep(3.0)

        while self.current_wp < len(WAYPOINTS) and not rospy.is_shutdown():
            gx, gy, desc = WAYPOINTS[self.current_wp]
            tx, ty = self.gazebo_to_liosam(gx, gy)
            success = self.navigate_to(tx, ty, desc)

            if success:
                self.current_wp += 1
                self.retries = 0
            else:
                self.retries += 1
                rospy.logwarn(f"Tentative {self.retries}/{MAX_RETRIES} pour {desc}")
                if self.retries >= MAX_RETRIES:
                    rospy.logwarn(f"⏭️  Abandon de {desc}")
                    self.current_wp += 1
                    self.retries = 0
                else:
                    rospy.loginfo("Recul de secours...")
                    cmd = Twist()
                    cmd.linear.x = -0.2
                    cmd.angular.z = 0.3
                    for _ in range(30):
                        self.cmd_pub.publish(cmd)
                        self.rate.sleep()
                    self.stop()
                    rospy.sleep(1.0)

        self.stop()
        rospy.loginfo("=== Navigation terminée ! ===")
        rospy.loginfo(f"Position finale : ({self.x:.2f}, {self.y:.2f})")

if __name__ == '__main__':
    try:
        navigator = BunkerNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass