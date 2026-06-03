#!/usr/bin/env python3
"""
Benchmark de vitesse pour le robot Bunker — v6.0
==================================================
CHANGEMENTS vs v5.1 :
  - Détection automatique du planner (DWA / TEB / EBand)
    via le paramètre /move_base/base_local_planner
  - Dynamic reconfigure adapté à chaque planner
  - Logs nettoyés : résultat par segment + résumé final uniquement
  - Suppression des logs répétitifs costmap/TF/warnings
  - Progression claire : [Run X/Y] sans flood

ARCHITECTURE « move_base reste maître » (v5+) :
  1. Découpe chaque côté du carré en waypoints intermédiaires denses
  2. Envoie séquentiellement chaque waypoint comme goal move_base
  3. Avance au waypoint suivant dès proximité < WP_ADVANCE_M
  4. Si move_base ABORT → clear costmaps, skip, passer au suivant
"""

import rospy
import actionlib
import csv
import os
import math
import time
import threading
import logging
import numpy as np
from datetime import datetime

from move_base_msgs.msg         import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg               import Odometry, Path
from geometry_msgs.msg          import PoseStamped
from sensor_msgs.msg            import PointCloud2
from tf.transformations         import euler_from_quaternion, quaternion_from_euler
from std_srvs.srv               import Empty as EmptySrv
import sensor_msgs.point_cloud2 as pc2
import tf

# Réduire le bruit des loggers ROS internes
logging.getLogger('rosout').setLevel(logging.WARNING)


# ====================================================================== #
#                           CONFIGURATION                                  #
# ====================================================================== #

SPEEDS          = [0.7,  0.8, 1.0]
REPEATS         = 2
OUTPUT_DIR      = os.path.expanduser("~/bunker_benchmark")

SQUARE_CORNERS  = [
    (  0.0,   0.0),
    ( 25.0,   0.0),
    ( 25.0,  25.0),
    (  0.0,  25.0),
    (  0.0,   0.0),
]

REF_STEP_M      = 0.3
WP_STEP_M       = 3.0
WP_ADVANCE_M    = 1.5
WP_TIMEOUT_S    = 30.0
WP_SKIP_MAX     = 3

CLEAR_ON_ABORT  = True
STALL_TIME_S    = 5.0
STALL_VEL       = 0.03

CORNER_TOL_M    = 0.6
GOAL_TOLERANCE  = 0.35
YAW_TOLERANCE   = 0.25

CTE_SEARCH_WINDOW = 8
RECORD_HZ       = 10
LIDAR_SUBSAMPLE = 5
SEG_LABELS      = ["S1_Est", "S2_Nord", "S3_Ouest", "S4_Sud"]


# ====================================================================== #
#                     PLANNER DETECTION & CONFIG                           #
# ====================================================================== #

PLANNER_CONFIGS = {
    "dwa": {
        "dynreconf_ns": "/move_base/DWAPlannerROS",
        "speed_params": lambda speed: {
            "max_vel_x":          speed,
            "max_vel_trans":      speed,
            "min_vel_trans":      min(0.1, speed * 0.2),
            "xy_goal_tolerance":  GOAL_TOLERANCE,
            "yaw_goal_tolerance": YAW_TOLERANCE,
            "path_distance_bias": 80.0,
            "goal_distance_bias": 20.0,
            "occdist_scale":      0.05,
        },
    },
    "teb": {
        "dynreconf_ns": "/move_base/TebLocalPlannerROS",
        "speed_params": lambda speed: {
            "max_vel_x":          speed,
            "max_vel_x_backwards": min(0.2, speed * 0.4),
            "max_vel_theta":      min(1.5, speed * 2.0),
            "xy_goal_tolerance":  GOAL_TOLERANCE,
            "yaw_goal_tolerance": YAW_TOLERANCE,
            "weight_optimaltime": 1.0,
            "weight_obstacle":    50.0,
        },
    },
    "eband": {
        "dynreconf_ns": "/move_base/EBandPlannerROS",
        "speed_params": lambda speed: {
            "max_vel_lin":        speed,
            "max_vel_th":         min(1.5, speed * 2.0),
            "xy_goal_tolerance":  GOAL_TOLERANCE,
            "yaw_goal_tolerance": YAW_TOLERANCE,
        },
    },
}


def detect_planner():
    """Détecte le planner chargé via le paramètre ROS."""
    try:
        planner_class = rospy.get_param("/move_base/base_local_planner", "")
    except KeyError:
        planner_class = ""

    planner_class_lower = planner_class.lower()
    if "teb" in planner_class_lower:
        return "teb"
    elif "eband" in planner_class_lower:
        return "eband"
    elif "dwa" in planner_class_lower:
        return "dwa"
    elif "trajectory" in planner_class_lower:
        return "dwa"  # TrajectoryPlannerROS → mêmes params que DWA
    else:
        rospy.logwarn(f"Planner inconnu: '{planner_class}', fallback DWA")
        return "dwa"


# ====================================================================== #
#                     GÉNÉRATION DES CHEMINS                               #
# ====================================================================== #

def _interpolate_segment(x0, y0, x1, y1, step_m, label, include_end=False):
    seg_len = math.hypot(x1 - x0, y1 - y0)
    yaw     = math.atan2(y1 - y0, x1 - x0)
    n_steps = max(1, int(math.ceil(seg_len / step_m)))
    pts     = []
    for k in range(n_steps):
        t = k / n_steps
        pts.append((
            x0 + t * (x1 - x0),
            y0 + t * (y1 - y0),
            f"{label}_p{k}",
            yaw,
        ))
    if include_end:
        pts.append((x1, y1, f"{label}_end", yaw))
    return pts


def generate_reference_path(corners=SQUARE_CORNERS, step_m=REF_STEP_M):
    ref_pts       = []
    seg_start_idx = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        label  = SEG_LABELS[i % len(SEG_LABELS)]
        seg_start_idx.append(len(ref_pts))
        pts = _interpolate_segment(x0, y0, x1, y1, step_m, label,
                                   include_end=(i == len(corners) - 2))
        ref_pts.extend(pts)
    return ref_pts, seg_start_idx


def generate_nav_waypoints(corners=SQUARE_CORNERS, step_m=WP_STEP_M):
    segments = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        label  = SEG_LABELS[i % len(SEG_LABELS)]
        pts = _interpolate_segment(x0, y0, x1, y1, step_m, label,
                                   include_end=True)
        segments.append(pts)
    return segments


REF_PATH, REF_SEG_START = generate_reference_path()
NAV_SEGMENTS            = generate_nav_waypoints()


# ====================================================================== #
#                     GÉOMÉTRIE CTE                                        #
# ====================================================================== #

def _project_on_segment(px, py, ax, ay, bx, by):
    dx, dy   = bx - ax, by - ay
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-9:
        return ax, ay, 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_len2))
    return ax + t * dx, ay + t * dy, t


def compute_cte(robot_x, robot_y, ref_pts, seg_hint=0):
    lo = max(0, seg_hint - CTE_SEARCH_WINDOW)
    hi = min(len(ref_pts) - 2, seg_hint + CTE_SEARCH_WINDOW)

    best_dist = float("inf")
    best_seg  = seg_hint

    for i in range(lo, hi + 1):
        ax, ay = ref_pts[i][0], ref_pts[i][1]
        bx, by = ref_pts[i + 1][0], ref_pts[i + 1][1]
        fx, fy, _ = _project_on_segment(robot_x, robot_y, ax, ay, bx, by)
        d = math.hypot(robot_x - fx, robot_y - fy)
        if d < best_dist:
            best_dist = d
            best_seg  = i

    ax, ay = ref_pts[best_seg][0], ref_pts[best_seg][1]
    bx, by = ref_pts[best_seg + 1][0], ref_pts[best_seg + 1][1]
    fx, fy, _ = _project_on_segment(robot_x, robot_y, ax, ay, bx, by)
    cross = (bx - ax) * (robot_y - fy) - (by - ay) * (robot_x - fx)

    return best_seg, math.copysign(best_dist, cross)


# ====================================================================== #
#                     CLASSE PRINCIPALE                                    #
# ====================================================================== #

class SpeedBenchmark:

    def __init__(self):
        rospy.init_node("speed_benchmark", anonymous=False)

        # --- Détection du planner ---
        self.planner_name = detect_planner()
        self.planner_cfg  = PLANNER_CONFIGS[self.planner_name]
        rospy.loginfo(f"Planner détecté: {self.planner_name.upper()}")

        # --- État robot ---
        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.vth = 0.0
        self.odom_received  = False
        self.origin_x = self.origin_y = None

        # --- LiDAR ---
        self.lidar_point_count = 0
        self.lidar_received    = False
        self._lidar_lock       = threading.Lock()
        self._pcd_points       = []
        self._pcd_active       = False

        # --- Métriques CTE ---
        self._current_cte      = 0.0
        self._current_seg_idx  = 0
        self._cte_seg_hint     = 0

        # --- TF ---
        self.tf_listener = tf.TransformListener()

        # --- Move base ---
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Connexion move_base...")
        if not self.client.wait_for_server(timeout=rospy.Duration(30)):
            rospy.logfatal("move_base non disponible!")
            raise RuntimeError("move_base timeout")
        rospy.loginfo("move_base OK")

        # --- Dynamic reconfigure (adapté au planner) ---
        self.dyn_client = None
        dynreconf_ns = self.planner_cfg["dynreconf_ns"]
        try:
            rospy.loginfo(f"Connexion dynreconf: {dynreconf_ns}")
            from dynamic_reconfigure.client import Client as DynClient
            self.dyn_client = DynClient(dynreconf_ns, timeout=10)
            rospy.loginfo("dynreconf OK")
        except Exception as e:
            rospy.logwarn(f"dynreconf indisponible ({e}) — vitesse fixe")

        # --- Service clear_costmaps ---
        self._clear_costmaps = None
        if CLEAR_ON_ABORT:
            try:
                rospy.wait_for_service("/move_base/clear_costmaps", timeout=5.0)
                self._clear_costmaps = rospy.ServiceProxy(
                    "/move_base/clear_costmaps", EmptySrv
                )
            except rospy.ROSException:
                rospy.logwarn("clear_costmaps indisponible")

        # --- Publisher chemin de référence ---
        self.path_pub = rospy.Publisher(
            "/reference_path", Path, queue_size=1, latch=True
        )

        # --- Subscribers ---
        rospy.Subscriber("/lio_sam/mapping/odometry", Odometry,
                         self.odom_callback, queue_size=10)
        rospy.Subscriber("/velodyne_points", PointCloud2,
                         self.lidar_callback, queue_size=1)

        # --- Dossiers ---
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        os.makedirs(os.path.join(OUTPUT_DIR, "pcd"), exist_ok=True)

        # --- État CSV ---
        self._csv_writer      = None
        self._csv_lock        = threading.Lock()
        self._current_wp_idx  = 0
        self._current_wp_name = ""
        self._current_tx      = 0.0
        self._current_ty      = 0.0
        self._record_active   = False
        self._record_thread   = None
        self._current_speed   = 0.0

        # --- Compteur de clears (pour log condensé) ---
        self._clear_count     = 0

    # ------------------------------------------------------------------ #
    #  RÉFÉRENCE RVIZ                                                      #
    # ------------------------------------------------------------------ #

    def _publish_reference_path(self):
        path_msg                 = Path()
        path_msg.header.stamp    = rospy.Time.now()
        path_msg.header.frame_id = "map"
        for wx, wy, _, wyaw in REF_PATH:
            ps                    = PoseStamped()
            ps.header             = path_msg.header
            ps.pose.position.x    = wx
            ps.pose.position.y    = wy
            q                     = quaternion_from_euler(0.0, 0.0, wyaw)
            ps.pose.orientation.x = q[0]
            ps.pose.orientation.y = q[1]
            ps.pose.orientation.z = q[2]
            ps.pose.orientation.w = q[3]
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    # ------------------------------------------------------------------ #
    #  CALLBACKS                                                           #
    # ------------------------------------------------------------------ #

    def odom_callback(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        q        = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.vx  = msg.twist.twist.linear.x
        self.vy  = msg.twist.twist.linear.y
        self.vth = msg.twist.twist.angular.z
        self.odom_received = True

    def lidar_callback(self, msg):
        self.lidar_point_count = msg.width * msg.height
        self.lidar_received    = True
        if not self._pcd_active:
            return
        try:
            self.tf_listener.waitForTransform(
                "map", msg.header.frame_id,
                msg.header.stamp, rospy.Duration(0.05)
            )
        except (tf.Exception, tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException):
            return
        try:
            gen     = pc2.read_points(msg,
                                      field_names=("x", "y", "z", "intensity"),
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
        T               = tf.transformations.quaternion_matrix(rot)
        T[0:3, 3]       = trans
        n               = raw_pts.shape[0]
        xyz_hom         = np.ones((n, 4), dtype=np.float32)
        xyz_hom[:, 0:3] = raw_pts[:, 0:3]
        xyz_map         = (T @ xyz_hom.T).T
        intensity       = (raw_pts[:, 3] if raw_pts.shape[1] >= 4
                           else np.zeros(n, dtype=np.float32))
        batch = np.column_stack([xyz_map[:, 0:3], intensity]).astype(np.float32)
        with self._lidar_lock:
            self._pcd_points.append(batch)

    # ------------------------------------------------------------------ #
    #  CSV CONTINU                                                         #
    # ------------------------------------------------------------------ #

    CSV_HEADER = [
        "timestamp_ros", "timestamp_real",
        "waypoint_index", "waypoint_name",
        "target_x", "target_y",
        "robot_x", "robot_y", "robot_yaw",
        "vel_x", "vel_y", "vel_theta",
        "distance_to_goal",
        "lidar_point_count",
        "speed_setting",
        "cross_track_error",
        "nearest_seg_idx",
        "segment_label",
        "planner",
    ]

    def _record_loop(self):
        rate = rospy.Rate(RECORD_HZ)
        while self._record_active and not rospy.is_shutdown():
            seg_idx, cte = compute_cte(
                self.x, self.y, REF_PATH, self._cte_seg_hint
            )
            if seg_idx >= self._cte_seg_hint:
                self._cte_seg_hint = seg_idx
            self._current_cte     = cte
            self._current_seg_idx = seg_idx

            seg_label = ""
            for si in range(len(REF_SEG_START) - 1, -1, -1):
                if seg_idx >= REF_SEG_START[si]:
                    seg_label = SEG_LABELS[si % len(SEG_LABELS)]
                    break

            dist = math.hypot(self._current_tx - self.x,
                              self._current_ty - self.y)
            row = [
                rospy.Time.now().to_sec(), time.time(),
                self._current_wp_idx, self._current_wp_name,
                self._current_tx, self._current_ty,
                self.x, self.y, self.yaw,
                self.vx, self.vy, self.vth,
                dist, self.lidar_point_count, self._current_speed,
                round(self._current_cte, 4),
                self._current_seg_idx,
                seg_label,
                self.planner_name,
            ]
            with self._csv_lock:
                if self._csv_writer is not None:
                    self._csv_writer.writerow(row)
            rate.sleep()

    def _start_continuous_recording(self, writer, speed):
        self._csv_writer    = writer
        self._current_speed = speed
        self._record_active = True
        self._record_thread = threading.Thread(
            target=self._record_loop, daemon=True
        )
        self._record_thread.start()

    def _stop_continuous_recording(self):
        self._record_active = False
        if self._record_thread:
            self._record_thread.join(timeout=2.0)
            self._record_thread = None
        with self._csv_lock:
            self._csv_writer = None

    # ------------------------------------------------------------------ #
    #  PCD                                                                 #
    # ------------------------------------------------------------------ #

    def _save_pcd(self, speed, repeat_num):
        with self._lidar_lock:
            if not self._pcd_points:
                return
            all_pts          = np.vstack(self._pcd_points)
            self._pcd_points = []
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        pcd_path = os.path.join(
            OUTPUT_DIR, "pcd",
            f"lidar_{self.planner_name}_speed{speed:.1f}_rep{repeat_num}_{ts}.pcd"
        )
        N      = all_pts.shape[0]
        header = (
            f"# .PCD v0.7\n"
            f"# planner={self.planner_name} speed={speed:.1f} m/s rep={repeat_num}\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            f"WIDTH {N}\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {N}\n"
            "DATA ascii\n"
        )
        with open(pcd_path, "w") as f:
            f.write(header)
            np.savetxt(f, all_pts, fmt="%.4f")

    # ------------------------------------------------------------------ #
    #  VITESSE + TUNING PLANNER                                            #
    # ------------------------------------------------------------------ #

    def set_speed(self, speed):
        """Règle la vitesse via dynamic_reconfigure, adapté au planner."""
        if self.dyn_client is None:
            return False
        try:
            params = self.planner_cfg["speed_params"](speed)
            self.dyn_client.update_configuration(params)
            return True
        except Exception as e:
            rospy.logwarn(f"dynreconf error: {e}")
            return False

    # ------------------------------------------------------------------ #
    #  ENVOI GOAL + CLEAR COSTMAPS                                        #
    # ------------------------------------------------------------------ #

    def _send_mb_goal(self, x, y, yaw):
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

    def _try_clear_costmaps(self):
        if self._clear_costmaps is not None:
            try:
                self._clear_costmaps()
                self._clear_count += 1
                rospy.sleep(0.5)
            except rospy.ServiceException:
                pass

    # ------------------------------------------------------------------ #
    #  NAVIGATION SEGMENT                                                  #
    # ------------------------------------------------------------------ #

    def _navigate_segment(self, seg_i, nav_wps):
        if not nav_wps:
            return True, 0, 0.0

        seg_name   = SEG_LABELS[seg_i % len(SEG_LABELS)]
        n_wps      = len(nav_wps)
        wp_idx     = 0
        skip_count = 0
        seg_start  = time.time()

        if seg_i < len(REF_SEG_START):
            self._cte_seg_hint = REF_SEG_START[seg_i]

        def _send_wp(idx):
            wx, wy, wname, wyaw = nav_wps[idx]
            self._send_mb_goal(wx, wy, wyaw)
            self._current_wp_name = wname
            self._current_tx      = wx
            self._current_ty      = wy
            self._current_wp_idx  = idx
            return time.time()

        wp_start = _send_wp(wp_idx)
        rate     = rospy.Rate(10)

        while not rospy.is_shutdown():
            wx, wy = nav_wps[wp_idx][0], nav_wps[wp_idx][1]
            dist   = math.hypot(wx - self.x, wy - self.y)

            is_last  = (wp_idx == n_wps - 1)
            tol      = CORNER_TOL_M if is_last else WP_ADVANCE_M
            elapsed  = time.time() - wp_start

            if dist < tol:
                if is_last:
                    self.client.cancel_goal()
                    dur = time.time() - seg_start
                    rospy.loginfo(
                        f"  {seg_name}: OK  {dur:.1f}s  "
                        f"CTE={self._current_cte:+.3f}m  "
                        f"skips={skip_count}"
                    )
                    return True, skip_count, dur
                else:
                    wp_idx  += 1
                    wp_start = _send_wp(wp_idx)
                    continue

            state = self.client.get_state()

            if state == actionlib.GoalStatus.SUCCEEDED:
                if is_last:
                    dur = time.time() - seg_start
                    rospy.loginfo(
                        f"  {seg_name}: OK  {dur:.1f}s  "
                        f"CTE={self._current_cte:+.3f}m  "
                        f"skips={skip_count}"
                    )
                    return True, skip_count, dur
                wp_idx  += 1
                wp_start = _send_wp(wp_idx)
                continue

            if state in (actionlib.GoalStatus.ABORTED,
                         actionlib.GoalStatus.REJECTED):
                skip_count += 1
                self._try_clear_costmaps()
                if skip_count > WP_SKIP_MAX:
                    dur = time.time() - seg_start
                    rospy.logwarn(
                        f"  {seg_name}: FAIL  {dur:.1f}s  "
                        f"(abandon après {WP_SKIP_MAX} skips)"
                    )
                    self.client.cancel_goal()
                    return False, skip_count, dur
                if not is_last:
                    wp_idx += 1
                wp_start = _send_wp(wp_idx)
                continue

            speed_actual = math.hypot(self.vx, self.vy)
            if (elapsed > STALL_TIME_S and speed_actual < STALL_VEL
                    and state == actionlib.GoalStatus.ACTIVE):
                self._try_clear_costmaps()
                wp_start = _send_wp(wp_idx)

            if elapsed > WP_TIMEOUT_S:
                skip_count += 1
                self._try_clear_costmaps()
                if skip_count > WP_SKIP_MAX:
                    dur = time.time() - seg_start
                    rospy.logwarn(
                        f"  {seg_name}: FAIL  {dur:.1f}s  "
                        f"(timeout, {skip_count} skips)"
                    )
                    self.client.cancel_goal()
                    return False, skip_count, dur
                if not is_last:
                    wp_idx += 1
                wp_start = _send_wp(wp_idx)
                continue

            rate.sleep()

        self.client.cancel_goal()
        return False, skip_count, time.time() - seg_start

    # ------------------------------------------------------------------ #
    #  RUN COMPLET                                                         #
    # ------------------------------------------------------------------ #

    def record_run(self, speed, repeat_num, run_n, total_runs):
        ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            OUTPUT_DIR,
            f"run_{self.planner_name}_speed{speed:.1f}_rep{repeat_num}_{ts}.csv"
        )

        rospy.loginfo(
            f"[Run {run_n}/{total_runs}] "
            f"{self.planner_name.upper()} @ {speed} m/s  rep {repeat_num}/{REPEATS}"
        )

        seg_results = []
        self._clear_count = 0

        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(self.CSV_HEADER)

            with self._lidar_lock:
                self._pcd_points = []
            self._pcd_active      = True
            self._current_cte     = 0.0
            self._current_seg_idx = 0
            self._cte_seg_hint    = 0

            self._start_continuous_recording(writer, speed)

            for seg_i, seg_wps in enumerate(NAV_SEGMENTS):
                success, skips, dur = self._navigate_segment(seg_i, seg_wps)

                corner_x, corner_y = seg_wps[-1][0], seg_wps[-1][1]
                dist_f = math.hypot(corner_x - self.x, corner_y - self.y)
                seg_results.append({
                    "name":       SEG_LABELS[seg_i % len(SEG_LABELS)],
                    "success":    success,
                    "duration":   dur,
                    "dist_final": dist_f,
                    "skips":      skips,
                })

                if rospy.is_shutdown():
                    break

            self._stop_continuous_recording()
            self._pcd_active = False

        self._save_pcd(speed, repeat_num)

        # Résumé condensé du run
        ok = sum(1 for r in seg_results if r["success"])
        total_dur = sum(r["duration"] for r in seg_results)
        rospy.loginfo(
            f"  → {ok}/{len(seg_results)} segments OK  "
            f"total={total_dur:.1f}s  "
            f"clears={self._clear_count}"
        )

        return seg_results

    # ------------------------------------------------------------------ #
    #  UTILITAIRES                                                         #
    # ------------------------------------------------------------------ #

    def return_to_origin(self):
        self._send_mb_goal(self.origin_x, self.origin_y, 0.0)
        rate  = rospy.Rate(2)
        start = time.time()
        while not rospy.is_shutdown():
            d = math.hypot(self.origin_x - self.x, self.origin_y - self.y)
            if d < CORNER_TOL_M:
                self.client.cancel_goal()
                break
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                break
            if state in (actionlib.GoalStatus.ABORTED,
                         actionlib.GoalStatus.REJECTED):
                self._try_clear_costmaps()
                self._send_mb_goal(self.origin_x, self.origin_y, 0.0)
            if time.time() - start > 120:
                self.client.cancel_goal()
                rospy.logwarn("Retour origine timeout")
                break
            rate.sleep()
        rospy.sleep(2.0)

    def save_summary(self, all_results):
        path = os.path.join(OUTPUT_DIR, f"summary_{self.planner_name}.csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                "planner", "speed", "repeat", "segment", "success",
                "duration_s", "dist_final_m", "skips",
                "success_rate_run",
            ])
            for speed, repeat, seg_list in all_results:
                sr = (sum(1 for r in seg_list if r["success"]) / len(seg_list)
                      if seg_list else 0)
                for r in seg_list:
                    w.writerow([
                        self.planner_name,
                        speed, repeat, r["name"], r["success"],
                        round(r["duration"], 2),
                        round(r["dist_final"], 3),
                        r["skips"], round(sr, 3),
                    ])

    # ------------------------------------------------------------------ #
    #  MAIN                                                                #
    # ------------------------------------------------------------------ #

    def run(self):
        rospy.loginfo("Attente odometrie...")
        deadline = rospy.Time.now() + rospy.Duration(30)
        while not self.odom_received and not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                rospy.logfatal("Timeout odometrie!")
                return
            rospy.sleep(0.5)

        self.origin_x, self.origin_y = self.x, self.y
        self._publish_reference_path()

        total = len(SPEEDS) * REPEATS
        rospy.loginfo(
            f"=== BENCHMARK {self.planner_name.upper()} ===  "
            f"{len(SPEEDS)} vitesses x {REPEATS} reps = {total} runs"
        )
        rospy.sleep(3.0)

        all_results = []
        n = 0

        for speed in SPEEDS:
            self.set_speed(speed)
            rospy.sleep(1.0)
            for rep in range(1, REPEATS + 1):
                if rospy.is_shutdown():
                    break
                n += 1
                seg_results = self.record_run(speed, rep, n, total)
                all_results.append((speed, rep, seg_results))
                if n < total:
                    self.return_to_origin()

        self.save_summary(all_results)

        # --- Résumé final ---
        rospy.loginfo("=" * 55)
        rospy.loginfo(f"  RÉSUMÉ FINAL — {self.planner_name.upper()}")
        rospy.loginfo("=" * 55)
        for speed in SPEEDS:
            runs   = [(r, segs) for s, r, segs in all_results if s == speed]
            all_sg = [sg for _, sgs in runs for sg in sgs]
            if all_sg:
                sr = sum(1 for s in all_sg if s["success"]) / len(all_sg) * 100
                ad = sum(s["duration"]   for s in all_sg) / len(all_sg)
                ap = sum(s["dist_final"] for s in all_sg) / len(all_sg)
                sk = sum(s["skips"]      for s in all_sg)
                rospy.loginfo(
                    f"  {speed:.1f} m/s  "
                    f"succès={sr:.0f}%  dur={ad:.1f}s  "
                    f"err={ap:.2f}m  skips={sk}"
                )
        rospy.loginfo(f"  Fichiers: {OUTPUT_DIR}")
        rospy.loginfo("=" * 55)


# ====================================================================== #
if __name__ == "__main__":
    try:
        SpeedBenchmark().run()
    except rospy.ROSInterruptException:
        pass