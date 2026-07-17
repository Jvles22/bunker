#!/usr/bin/env python3
"""
collect_waypoints_sim.py
-------------------------
Version simulation de collect_waypoints_liosam.py.

Enregistre la pose actuelle du robot (depuis LIO-SAM, ou son relais en mode
debug) dans un fichier waypoints.txt à chaque appui sur la touche 'S'.
Utile pour collecter des waypoints en pilotant le robot au teleop.

Crée automatiquement un nouveau dossier run_XXX dans DATA_DIR.

Usage :
    rosrun bunker_slam collect_waypoints_sim.py

Commandes :
    S — enregistrer la position actuelle
    D — supprimer le dernier waypoint
    L — afficher la liste des waypoints
    Q — sauvegarder et quitter

Topics :
    Subscribed : /lio_sam/mapping/odometry (nav_msgs/Odometry)
        — pose LIO-SAM en mode normal, relayée depuis /odom (EKF) en mode
          debug (slam:=false), cf. navigation_debug.launch.

Fichier de sortie :
    /media/user/Jules/data_simu/waypoints/run_XXX/waypoints.txt
    Format : x y z yaw_deg
"""

import rospy
import math
import sys
import os
import tty
import termios
import tf
from nav_msgs.msg import Odometry

# ── Paramètres ────────────────────────────────────────────────────────────────
DATA_DIR   = os.path.expanduser("/media/user/Jules/data_simu/waypoints")
ODOM_TOPIC = "/lio_sam/mapping/odometry"
# ──────────────────────────────────────────────────────────────────────────────

def get_next_run_dir():
    """Trouve le prochain dossier run_XXX disponible."""
    os.makedirs(DATA_DIR, exist_ok=True)
    existing = [d for d in os.listdir(DATA_DIR)
                if d.startswith("run_") and
                os.path.isdir(os.path.join(DATA_DIR, d))]
    indices = []
    for d in existing:
        try:
            indices.append(int(d.split("_")[1]))
        except (IndexError, ValueError):
            pass
    next_idx = max(indices) + 1 if indices else 1
    run_dir = os.path.join(DATA_DIR, f"run_{next_idx:03d}")
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


class WaypointCollector:
    def __init__(self):
        rospy.init_node('collect_waypoints_sim', anonymous=False)

        self.x   = 0.0
        self.y   = 0.0
        self.z   = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.waypoints = []

        # Créer le dossier run
        self.run_dir = get_next_run_dir()
        self.waypoints_file = os.path.join(self.run_dir, "waypoints.txt")

        rospy.Subscriber(ODOM_TOPIC, Odometry, self._odom_cb)

        rospy.loginfo(f"Collecteur de waypoints (simu) démarré.")
        rospy.loginfo(f"Topic  : {ODOM_TOPIC}")
        rospy.loginfo(f"Run    : {self.run_dir}")
        rospy.loginfo(f"Fichier: {self.waypoints_file}")
        rospy.loginfo("Commandes :")
        rospy.loginfo("  S — enregistrer la position actuelle")
        rospy.loginfo("  D — supprimer le dernier waypoint")
        rospy.loginfo("  L — afficher la liste des waypoints")
        rospy.loginfo("  Q — sauvegarder et quitter")

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w])
        self.odom_received = True

    def _get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return key.lower()

    def _save_to_file(self):
        with open(self.waypoints_file, 'w') as f:
            f.write("# x          y          z          yaw_deg\n")
            for (x, y, z, yaw) in self.waypoints:
                yaw_deg = math.degrees(yaw)
                f.write(f"{x:.6f}  {y:.6f}  {z:.6f}  {yaw_deg:.4f}\n")
        rospy.loginfo(f"✓ {len(self.waypoints)} waypoints sauvegardés")
        rospy.loginfo(f"  → {self.waypoints_file}")
        rospy.loginfo(f"  → Pour naviguer : --run {os.path.basename(self.run_dir)}")

    def run(self):
        rospy.loginfo(f"En attente de l'odométrie sur {ODOM_TOPIC}...")
        rate = rospy.Rate(10)
        while not self.odom_received and not rospy.is_shutdown():
            rate.sleep()
        rospy.loginfo("✓ Odométrie reçue — prêt à collecter.")
        print(f"\n--- Run : {os.path.basename(self.run_dir)} ---")
        print("--- Appuie sur S pour enregistrer, Q pour quitter ---\n")

        while not rospy.is_shutdown():
            key = self._get_key()

            if key == 's':
                wp = (self.x, self.y, self.z, self.yaw)
                self.waypoints.append(wp)
                idx = len(self.waypoints)
                yaw_deg = math.degrees(self.yaw)
                print(f"[WP {idx:03d}] x={self.x:.3f}  y={self.y:.3f}  "
                      f"z={self.z:.3f}  yaw={yaw_deg:.1f}°")

            elif key == 'd':
                if self.waypoints:
                    removed = self.waypoints.pop()
                    print(f"✗ Dernier waypoint supprimé : "
                          f"x={removed[0]:.3f} y={removed[1]:.3f}")
                else:
                    print("Aucun waypoint à supprimer.")

            elif key == 'l':
                print(f"\n--- Liste ({len(self.waypoints)} waypoints) ---")
                for i, (x, y, z, yaw) in enumerate(self.waypoints):
                    print(f"  [{i+1:03d}] x={x:.3f}  y={y:.3f}  "
                          f"z={z:.3f}  yaw={math.degrees(yaw):.1f}°")
                print()

            elif key == 'q':
                if self.waypoints:
                    self._save_to_file()
                else:
                    print("Aucun waypoint — dossier run supprimé.")
                    os.rmdir(self.run_dir)
                break


if __name__ == '__main__':
    try:
        collector = WaypointCollector()
        collector.run()
    except rospy.ROSInterruptException:
        pass
