#!/usr/bin/env python3
"""
Traversée en carré — slopes_map  (v4.0 simplifié)
===================================================
Trajectoire directe de waypoint en waypoint :
(0,0) → (24,0) → (24,12) → (0,12) → (0,24) → (24,24)

Pas de points intermédiaires artificiels — move_base gère le chemin.
"""

import rospy
import actionlib
import csv
import os
import math
import time
import threading
import subprocess
import signal

import numpy as np
import sensor_msgs.point_cloud2 as pc2
import tf

from move_base_msgs.msg  import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg        import Odometry
from sensor_msgs.msg     import Imu, PointCloud2
from tf.transformations  import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv        import Empty as EmptySrv
from datetime            import datetime

try:
    from lio_sam.srv import save_map as LioSamSaveMap
    _HAS_LIO_SAM_SRV = True
except ImportError:
    _HAS_LIO_SAM_SRV = False


# ====================================================================== #
#                           CONFIGURATION                                  #
# ====================================================================== #

OUTPUT_DIR = "/home/projet_bunker/data_simu"
SPEED      = 0.5   # m/s
LOOPS      = 5     # nombre d'allers-retours (1 = aller + retour)

# Waypoints dans l'ordre de traversée : (x, y, label)
WAYPOINTS = [
    # (  0.0, -12.0, "wp 1"),    # Sud centre — plein Est depuis spawn
    ( 11.0, -12.0, "wp 2"),    # Coin SE
    ( 11.0,   0.0, "wp 3"),    # Milieu Est — passe près obstacle1_0
    ( -10.0,  0.0, "wp 5"),    # Nord centre — passe près obstacle2/4
    (-10.0,   12.0, "wp 6"),
    (11.0,   12.0, "wp 7"),    # Milieu Ouest — passe près obstacle5
    #(-10.0,   12.0, "wp 6"),
    #(-10.0, -12.0, "Spawn"),   # Retour spawn
]


# --- Paramètres de navigation ---
GOAL_TOLERANCE  = 0.5    # tolérance d'arrivée (m)
YAW_TOLERANCE   = 0.3
WP_TIMEOUT_S    = 2200.0  # timeout par waypoint (s)
WP_SKIP_MAX     = 2      # tentatives max avant abandon
STALL_TIME_S    = 15.0    # délai avant détection blocage (s)
STALL_VEL       = 0.03   # vitesse sous laquelle = bloqué (m/s)
CLEAR_ON_ABORT  = True

RECORD_HZ       = 10

# --- IMU ---
IMU_TOPIC      = "/imu/data"
MAX_SLOPE_DEG  = 35.0
EMERGENCY_DEBOUNCE = 5

# --- Courbe vitesse/pente ---
SLOPE_SPEED_TABLE = [
    ( 6.0, 0.4),
    (15.0, 0.3),
    (25.0, 0.2),
    (35.0, 0.1),
]
SLOPE_SPEED_DEBOUNCE = 3

# --- Enregistrement ---
LIDAR_SUBSAMPLE = 10   # 1 point Velodyne sur N → PCD final plus léger

# Throttle des topics lourds via topic_tools/throttle (C++)
# Format : (topic_in, msgs/s, topic_out)
COSTMAP_THROTTLE = [
    ("/move_base/global_costmap/costmap",        0.5, "/move_base/global_costmap/costmap_throttled"),
    ("/move_base/global_costmap/costmap_updates", 2.0, "/move_base/global_costmap/costmap_updates_throttled"),
    ("/move_base/local_costmap/costmap",          1.0, "/move_base/local_costmap/costmap_throttled"),
    ("/bunker/elevation_map",                     1.0, "/bunker/elevation_map_throttled"),
]

ROSBAG_TOPICS = [
    "/move_base/global_costmap/costmap_throttled",
    "/move_base/global_costmap/costmap_updates_throttled",
    "/bunker/elevation_map_throttled",
    "/move_base/local_costmap/costmap_throttled",
    "/odom",
    "/tf",
    "/tf_static",
    "/imu/data",
    "/cmd_vel",
    "/move_base/status",
]


# ====================================================================== #
#                     DÉTECTION DU PLANNER                                #
# ====================================================================== #

PLANNER_CONFIGS = {
    "dwa": {
        "dynreconf_ns": "/move_base/DWAPlannerROS",
        "speed_params": lambda s: {
            "max_vel_x": s, "max_vel_trans": s,
            "min_vel_trans": min(0.1, s * 0.2),
            "xy_goal_tolerance": GOAL_TOLERANCE,
            "yaw_goal_tolerance": YAW_TOLERANCE,
        },
    },
    "teb": {
        "dynreconf_ns": "/move_base/TebLocalPlannerROS",
        "speed_params": lambda s: {
            "max_vel_x": s,
            "max_vel_x_backwards": min(0.2, s * 0.4),
            "max_vel_theta": min(1.5, s * 2.0),
            "xy_goal_tolerance": GOAL_TOLERANCE,
            "yaw_goal_tolerance": YAW_TOLERANCE,
        },
    },
    "eband": {
        "dynreconf_ns": "/move_base/EBandPlannerROS",
        "speed_params": lambda s: {
            "max_vel_lin": s,
            "max_vel_th": min(1.5, s * 2.0),
            "xy_goal_tolerance": GOAL_TOLERANCE,
            "yaw_goal_tolerance": YAW_TOLERANCE,
        },
    },
}


def detect_planner():
    try:
        pc = rospy.get_param("/move_base/base_local_planner", "")
    except KeyError:
        pc = ""
    pcl = pc.lower()
    if "teb"   in pcl: return "teb"
    if "eband" in pcl: return "eband"
    rospy.logwarn(f"Planner inconnu : '{pc}', fallback DWA")
    return "dwa"


# ====================================================================== #
#                     CLASSE PRINCIPALE                                    #
# ====================================================================== #

class SquareTraverse:

    CSV_HEADER = [
        "timestamp_ros", "timestamp_real",
        "waypoint_label",
        "target_x", "target_y",
        "robot_x", "robot_y", "robot_yaw",
        "vel_x", "vel_y", "vel_theta",
        "distance_to_goal",
        "speed_setting",
        "pitch_deg", "roll_deg", "slope_deg",
        "planner",
    ]

    def __init__(self):
        rospy.init_node("square_traverse", anonymous=False)

        self.planner_name = detect_planner()
        self.planner_cfg  = PLANNER_CONFIGS[self.planner_name]
        rospy.loginfo(f"Planner : {self.planner_name.upper()}")

        # Odométrie
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.vth = 0.0
        self.odom_received = False

        # IMU
        self._pitch_deg = self._roll_deg = self._slope_deg = 0.0
        self._imu_received = False
        self._emergency_stop = False
        self._emergency_debounce_count = 0

        # Courbe vitesse/pente
        self._current_speed           = SPEED
        self._slope_speed_tier        = -1
        self._slope_speed_candidate   = -1
        self._slope_speed_candidate_count = 0

        # move_base
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Connexion move_base...")
        deadline = time.time() + 30.0
        connected = False
        while time.time() < deadline:
            if self.client.wait_for_server(timeout=rospy.Duration(0.5)):
                connected = True
                break
        if not connected:
            raise RuntimeError("move_base non disponible")
        rospy.loginfo("move_base OK")

        # Dynamic reconfigure
        self.dyn_client = None
        try:
            from dynamic_reconfigure.client import Client as DynClient
            self.dyn_client = DynClient(self.planner_cfg["dynreconf_ns"], timeout=10)
            rospy.loginfo("dynreconf OK")
        except Exception as e:
            rospy.logwarn(f"dynreconf indisponible ({e})")

        # clear_costmaps
        self._clear_costmaps = None
        if CLEAR_ON_ABORT:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=5.0)
                self._clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", EmptySrv)
            except rospy.ROSException:
                rospy.logwarn("clear_costmaps indisponible")

        # TF listener (PCD accumulation)
        self.tf_listener = tf.TransformListener()

        # Subscribers
        rospy.Subscriber("/lio_sam/mapping/odometry", Odometry, self._odom_cb, queue_size=10)
        rospy.Subscriber(IMU_TOPIC, Imu, self._imu_cb, queue_size=10)
        rospy.Subscriber("/velodyne_points", PointCloud2, self._lidar_cb, queue_size=1)

        # PCD accumulation
        self._pcd_points = []
        self._pcd_lock   = threading.Lock()
        self._pcd_active = False

        # ROSbag + throttle subprocesses
        self._bag_proc      = None
        self._throttle_procs = []

        # CSV
        self._csv_writer    = None
        self._csv_lock      = threading.Lock()
        self._record_active = False
        self._record_thread = None
        self._current_label = ""
        self._current_tx = self._current_ty = 0.0

        os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ------------------------------------------------------------------ #
    #  CALLBACKS                                                           #
    # ------------------------------------------------------------------ #

    def _odom_cb(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        q        = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.vx  = msg.twist.twist.linear.x
        self.vy  = msg.twist.twist.linear.y
        self.vth = msg.twist.twist.angular.z
        self.odom_received = True

    def _imu_cb(self, msg):
        q = msg.orientation
        roll, pitch, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._pitch_deg = math.degrees(pitch)
        self._roll_deg  = math.degrees(roll)
        self._slope_deg = max(abs(self._pitch_deg), abs(self._roll_deg))
        self._imu_received = True

        if self._emergency_stop:
            return
        if self._slope_deg > MAX_SLOPE_DEG:
            self._emergency_debounce_count += 1
            if self._emergency_debounce_count >= EMERGENCY_DEBOUNCE:
                self._emergency_stop = True
                rospy.logerr(f"ARRÊT D'URGENCE : pente {self._slope_deg:.1f}° > {MAX_SLOPE_DEG}°")
        else:
            self._emergency_debounce_count = 0

    # ------------------------------------------------------------------ #
    #  VITESSE / PENTE                                                     #
    # ------------------------------------------------------------------ #

    def _get_speed_for_slope(self, slope_deg):
        if slope_deg < SLOPE_SPEED_TABLE[0][0]:
            return SPEED
        for angle_limit, speed_limit in SLOPE_SPEED_TABLE:
            if slope_deg <= angle_limit:
                return speed_limit
        return SLOPE_SPEED_TABLE[-1][1]

    def _update_slope_speed(self):
        slope = self._slope_deg
        new_tier = len(SLOPE_SPEED_TABLE)
        for i, (angle_limit, _) in enumerate(SLOPE_SPEED_TABLE):
            if slope <= angle_limit:
                new_tier = i
                break

        if new_tier == self._slope_speed_tier:
            self._slope_speed_candidate = new_tier
            self._slope_speed_candidate_count = SLOPE_SPEED_DEBOUNCE
            return

        if new_tier == self._slope_speed_candidate:
            self._slope_speed_candidate_count += 1
        else:
            self._slope_speed_candidate = new_tier
            self._slope_speed_candidate_count = 1

        if self._slope_speed_candidate_count < SLOPE_SPEED_DEBOUNCE:
            return

        new_speed = self._get_speed_for_slope(slope)
        self._slope_speed_tier            = new_tier
        self._current_speed               = new_speed
        self._slope_speed_candidate_count = 0
        self._set_speed(new_speed)

    def _set_speed(self, speed):
        if self.dyn_client is None:
            return
        try:
            self.dyn_client.update_configuration(self.planner_cfg["speed_params"](speed))
        except Exception as e:
            rospy.logwarn(f"dynreconf : {e}")

    # ------------------------------------------------------------------ #
    #  CSV                                                                 #
    # ------------------------------------------------------------------ #

    def _record_loop(self):
        rate = rospy.Rate(RECORD_HZ)
        while self._record_active and not rospy.is_shutdown():
            self._update_slope_speed()
            dist = math.hypot(self._current_tx - self.x, self._current_ty - self.y)
            row = [
                rospy.Time.now().to_sec(), time.time(),
                self._current_label,
                self._current_tx, self._current_ty,
                round(self.x, 4), round(self.y, 4), round(self.yaw, 4),
                round(self.vx, 4), round(self.vy, 4), round(self.vth, 4),
                round(dist, 4),
                round(self._current_speed, 3),
                round(self._pitch_deg, 2), round(self._roll_deg, 2), round(self._slope_deg, 2),
                self.planner_name,
            ]
            with self._csv_lock:
                if self._csv_writer:
                    self._csv_writer.writerow(row)
            rate.sleep()

    def _start_recording(self, writer):
        self._csv_writer    = writer
        self._record_active = True
        self._record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self._record_thread.start()

    def _stop_recording(self):
        self._record_active = False
        if self._record_thread:
            self._record_thread.join(timeout=2.0)
        with self._csv_lock:
            self._csv_writer = None

    # ------------------------------------------------------------------ #
    #  NAVIGATION                                                          #
    # ------------------------------------------------------------------ #

    def _send_goal(self, x, y, yaw, label):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        self.client.send_goal(goal)
        self._current_label = label
        self._current_tx    = x
        self._current_ty    = y

    def _try_clear(self):
        if self._clear_costmaps:
            try:
                self._clear_costmaps()
                rospy.sleep(0.5)
            except rospy.ServiceException:
                pass

    def _navigate_to(self, x, y, label, yaw):
        """
        Envoie un goal à (x, y) et attend l'arrivée.
        Retourne (success, durée_s).
        """
        attempts = 0
        start    = time.time()
        rate     = rospy.Rate(10)

        self._send_goal(x, y, yaw, label)
        last_stall_check = time.time()

        while not rospy.is_shutdown():

            # Arrêt d'urgence
            if self._emergency_stop:
                self.client.cancel_goal()
                rospy.logerr(f"  {label} : ARRÊT D'URGENCE pente {self._slope_deg:.1f}°")
                return False, time.time() - start

            dist  = math.hypot(x - self.x, y - self.y)
            state = self.client.get_state()

            # Arrivée (distance ou confirmation move_base)
            if dist < GOAL_TOLERANCE or state == actionlib.GoalStatus.SUCCEEDED:
                self.client.cancel_goal()
                dur = time.time() - start
                rospy.loginfo(f"  {label} : OK  {dur:.1f}s  dist_finale={dist:.2f}m")
                return True, dur

            # Abort / rejet
            if state in (actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED):
                attempts += 1
                rospy.logwarn(f"  {label} : abort ({attempts}/{WP_SKIP_MAX})")
                if attempts >= WP_SKIP_MAX:
                    return False, time.time() - start
                self._try_clear()
                self._send_goal(x, y, yaw, label)
                last_stall_check = time.time()
                continue

            # Détection blocage
            speed_actual = math.hypot(self.vx, self.vy)
            if (time.time() - last_stall_check > STALL_TIME_S
                    and speed_actual < STALL_VEL
                    and state == actionlib.GoalStatus.ACTIVE):
                rospy.logwarn(f"  {label} : blocage — clear costmaps et retry")
                self._try_clear()
                self._send_goal(x, y, yaw, label)
                last_stall_check = time.time()

            # Timeout
            if time.time() - start > WP_TIMEOUT_S:
                attempts += 1
                rospy.logwarn(f"  {label} : timeout ({attempts}/{WP_SKIP_MAX})")
                if attempts >= WP_SKIP_MAX:
                    self.client.cancel_goal()
                    return False, time.time() - start
                self._try_clear()
                self._send_goal(x, y, yaw, label)
                last_stall_check = time.time()

            rate.sleep()

        self.client.cancel_goal()
        return False, time.time() - start

    # ------------------------------------------------------------------ #
    #  LIDAR / PCD / ROSBAG                                               #
    # ------------------------------------------------------------------ #

    def _lidar_cb(self, msg):
        if not self._pcd_active:
            return
        try:
            self.tf_listener.waitForTransform(
                "map", msg.header.frame_id,
                msg.header.stamp, rospy.Duration(0.05)
            )
        except Exception:
            return
        try:
            gen     = pc2.read_points(msg, field_names=("x", "y", "z"),
                                      skip_nans=True)
            raw_pts = np.array(list(gen), dtype=np.float32)[::LIDAR_SUBSAMPLE]
        except Exception:
            return
        if raw_pts.shape[0] == 0:
            return
        try:
            trans, rot = self.tf_listener.lookupTransform(
                "map", msg.header.frame_id, msg.header.stamp
            )
        except Exception:
            return
        T           = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3]   = trans
        n           = raw_pts.shape[0]
        xyz_hom     = np.ones((n, 4), dtype=np.float32)
        xyz_hom[:, 0:3] = raw_pts[:, 0:3]
        xyz_map     = (T @ xyz_hom.T).T[:, 0:3].astype(np.float32)
        with self._pcd_lock:
            self._pcd_points.append(xyz_map)

    def _save_pcd_accumulated(self, pcd_path):
        with self._pcd_lock:
            if not self._pcd_points:
                rospy.logwarn("  PCD accumulé : aucun point")
                return
            all_pts = np.vstack(self._pcd_points)
        N = all_pts.shape[0]
        header = (
            "# .PCD v0.7 — nuage accumulé (mode debug, repère map)\n"
            "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
            "COUNT 1 1 1\n"
            f"WIDTH {N}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {N}\nDATA ascii\n"
        )
        try:
            with open(pcd_path, "w") as f:
                f.write(header)
                for pt in all_pts:
                    f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")
            rospy.loginfo(f"  PCD accumulé ({N} pts) : {pcd_path}")
        except Exception as e:
            rospy.logwarn(f"  PCD accumulé : échec ({e})")

    def _start_throttles(self):
        for in_topic, rate, out_topic in COSTMAP_THROTTLE:
            cmd = ["rosrun", "topic_tools", "throttle", "messages",
                   in_topic, str(rate), out_topic]
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self._throttle_procs.append(proc)
        rospy.sleep(1.0)  # laisser les nœuds s'initialiser avant le bag
        rospy.loginfo(f"  Throttles démarrés ({len(COSTMAP_THROTTLE)} topics)")

    def _stop_throttles(self):
        for proc in self._throttle_procs:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        self._throttle_procs.clear()
        rospy.loginfo("  Throttles : fermés")

    def _start_rosbag(self, bag_path):
        self._start_throttles()
        cmd = ["rosbag", "record", "--lz4", "-O", bag_path] + ROSBAG_TOPICS
        self._bag_proc = subprocess.Popen(
            cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        rospy.loginfo(f"  ROSbag démarré : {bag_path}")

    def _stop_rosbag(self):
        if self._bag_proc and self._bag_proc.poll() is None:
            self._bag_proc.send_signal(signal.SIGINT)
            try:
                self._bag_proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                self._bag_proc.kill()
        self._bag_proc = None
        rospy.loginfo("  ROSbag : fermé")
        self._stop_throttles()

    # ------------------------------------------------------------------ #
    #  SAUVEGARDE PCD (LIO-SAM)                                           #
    # ------------------------------------------------------------------ #

    def _save_pcd_map(self, destination):
        if not _HAS_LIO_SAM_SRV:
            rospy.logwarn("lio_sam.srv introuvable — PCD ignoré")
            return None
        try:
            rospy.wait_for_service("/lio_sam/save_map", timeout=10.0)
        except rospy.ROSException:
            rospy.logwarn("/lio_sam/save_map indisponible")
            return None

        home     = os.path.expanduser("~")
        abs_dest = os.path.abspath(destination)
        rel_dest = abs_dest[len(home):] if abs_dest.startswith(home) else abs_dest

        try:
            os.makedirs(abs_dest, exist_ok=True)
            srv  = rospy.ServiceProxy("/lio_sam/save_map", LioSamSaveMap)
            resp = srv(resolution=0.0, destination=rel_dest)
            if resp.success:
                rospy.loginfo(f"  PCD sauvegardé : {abs_dest}")
                return abs_dest
            rospy.logwarn("  save_map : success=false")
        except rospy.ServiceException as e:
            rospy.logwarn(f"  PCD échoué : {e}")
        return None

    # ------------------------------------------------------------------ #
    #  MAIN                                                                #
    # ------------------------------------------------------------------ #

    def run(self):
        rospy.loginfo("Attente odométrie...")
        deadline = time.time() + 30.0
        while not self.odom_received and not rospy.is_shutdown():
            if time.time() > deadline:
                rospy.logfatal("Timeout odométrie !")
                return
            rospy.sleep(0.5)

        if not self._imu_received:
            rospy.logwarn(f"IMU non reçu sur '{IMU_TOPIC}'")

        rospy.loginfo(
            f"Départ ({self.x:.2f}, {self.y:.2f}) | "
            f"Planner : {self.planner_name.upper()} | "
            f"Vitesse : {SPEED} m/s"
        )

        self._set_speed(SPEED)
        rospy.sleep(2.0)

        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_tag  = f"traverse_{self.planner_name}_{ts}"
        filename = os.path.join(OUTPUT_DIR, f"{run_tag}.csv")
        pcd_path = os.path.join(OUTPUT_DIR, f"{run_tag}.pcd")
        bag_path = os.path.join(OUTPUT_DIR, f"{run_tag}.bag")
        pcd_dir  = os.path.join(OUTPUT_DIR, f"{run_tag}_lio_sam_pcd")

        # Démarrage enregistrements
        self._pcd_active = True
        self._start_rosbag(bag_path)

        results = []

        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.CSV_HEADER)
            self._start_recording(writer)

            prev_x, prev_y = self.x, self.y
            for loop_idx in range(LOOPS):
                # --- Aller ---
                rospy.loginfo(f"=== Loop {loop_idx + 1}/{LOOPS} — aller ===")
                for (wx, wy, label) in WAYPOINTS:
                    if rospy.is_shutdown() or self._emergency_stop:
                        break
                    lbl = f"L{loop_idx + 1}_fwd_{label}"
                    yaw = math.atan2(wy - prev_y, wx - prev_x)
                    rospy.loginfo(f"→ {lbl}  ({wx}, {wy})")
                    success, dur = self._navigate_to(wx, wy, lbl, yaw)
                    results.append({"label": lbl, "success": success, "dur": dur})
                    if not success:
                        rospy.logwarn(f"  {lbl} : FAIL — arrêt de la traversée")
                        break
                    prev_x, prev_y = wx, wy

                if rospy.is_shutdown() or self._emergency_stop or not results[-1]["success"]:
                    break

                # --- Retour ---
                rospy.loginfo(f"=== Loop {loop_idx + 1}/{LOOPS} — retour ===")
                for (wx, wy, label) in reversed(WAYPOINTS):
                    if rospy.is_shutdown() or self._emergency_stop:
                        break
                    lbl = f"L{loop_idx + 1}_rev_{label}"
                    yaw = math.atan2(wy - prev_y, wx - prev_x)
                    rospy.loginfo(f"→ {lbl}  ({wx}, {wy})")
                    success, dur = self._navigate_to(wx, wy, lbl, yaw)
                    results.append({"label": lbl, "success": success, "dur": dur})
                    if not success:
                        rospy.logwarn(f"  {lbl} : FAIL — arrêt de la traversée")
                        break
                    prev_x, prev_y = wx, wy

                if rospy.is_shutdown() or self._emergency_stop or not results[-1]["success"]:
                    break

            self._stop_recording()

        # Arrêt enregistrements + sauvegarde PCD
        self._pcd_active = False
        self._stop_rosbag()
        self._save_pcd_accumulated(pcd_path)  # toujours disponible (debug + LIO-SAM)
        self._save_pcd_map(pcd_dir)           # LIO-SAM uniquement (silencieux si absent)

        # Résumé
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"  RÉSUMÉ TRAVERSÉE  ({LOOPS} loop(s))")
        rospy.loginfo("=" * 50)
        if self._emergency_stop:
            rospy.logerr(f"  ARRÊT D'URGENCE — pente > {MAX_SLOPE_DEG}°")
        total = 0.0
        for r in results:
            rospy.loginfo(f"  {'OK  ' if r['success'] else 'FAIL'}  {r['label']:<20}  {r['dur']:5.1f}s")
            total += r["dur"]
        ok = sum(1 for r in results if r["success"])
        rospy.loginfo(f"  {ok}/{len(results)} waypoints atteints | {total:.1f}s total")
        rospy.loginfo(f"  CSV : {filename}")
        rospy.loginfo(f"  PCD : {pcd_path}")
        rospy.loginfo(f"  BAG : {bag_path}")
        rospy.loginfo("=" * 50)


# ====================================================================== #
if __name__ == "__main__":
    try:
        SquareTraverse().run()
    except rospy.ROSInterruptException:
        pass
    except RuntimeError as e:
        rospy.logfatal(str(e))
        import sys; sys.exit(1)