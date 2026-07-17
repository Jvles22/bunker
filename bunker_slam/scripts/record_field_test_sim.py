#!/usr/bin/env python3
"""
record_field_test_sim.py
--------------------------
Version simulation de record_field_test.py.

Enregistre une session de test en conduite manuelle (teleop) : AUCUNE
navigation autonome n'est lancée par ce script.

Produit, par run :
  - un rosbag complet (tous les topics actifs, rosbag record -a)
  - un nuage de points global recalé, via le service LIO-SAM /lio_sam/save_map
    (même mécanisme que GlobalMap.pcd dans square_ramp_traverse.py —
    recalage SLAM, pas une simple accumulation TF de /velodyne_points brut)

Prérequis : LIO-SAM déjà démarré (le service /lio_sam/save_map doit exister).
En mode debug (slam:=false, sans LIO-SAM), le service n'existe pas — le
script continue de fonctionner et se contente de sauter la sauvegarde PCD
avec un avertissement (cf. _save_pcd_map).

Usage :
    rosrun bunker_slam record_field_test_sim.py
    Ctrl+C pour arrêter proprement — ferme le bag et sauvegarde le nuage.

Structure de sortie :
    ~/data_simu/data_reglage/run_XXX/
    ├── recording.bag
    └── GlobalMap.pcd
"""

import rospy
import os
import subprocess
import signal
import glob
import shutil

try:
    from lio_sam.srv import save_map as LioSamSaveMap
    _HAS_LIO_SAM_SRV = True
except ImportError:
    _HAS_LIO_SAM_SRV = False

# Note : on désactive les signal handlers par défaut de rospy et on gère
# SIGINT nous-mêmes. Avec le comportement par défaut, Ctrl+C déclenche le
# shutdown rospy AVANT notre code de nettoyage (finally) — l'appel au
# service /lio_sam/save_map part alors que la couche de comm du nœud est
# déjà en train de se fermer, et échoue avec "returned no response". En
# gérant le signal nous-mêmes, rospy reste pleinement actif jusqu'à la fin
# du nettoyage (bag + service), et on ne signale l'arrêt qu'à la toute fin.

# ── Paramètres ────────────────────────────────────────────────────────────────
DATA_DIR = os.path.expanduser("/home/projet_bunker/data_simu/data_reglage")
# ──────────────────────────────────────────────────────────────────────────────


def get_next_run_dir():
    """Trouve le prochain dossier run_XXX disponible (même convention que
    collect_waypoints_sim.py)."""
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


class FieldTestRecorder:
    def __init__(self):
        rospy.init_node('record_field_test_sim', anonymous=False,
                         disable_signals=True)

        self._stop_requested = False
        signal.signal(signal.SIGINT,  self._request_stop)
        signal.signal(signal.SIGTERM, self._request_stop)

        self.run_dir  = get_next_run_dir()
        self.bag_path = os.path.join(self.run_dir, "recording.bag")
        self.bag_proc = None

        self.bag_proc = subprocess.Popen(
            f"rosbag record -a -O {self.bag_path}",
            shell=True, preexec_fn=os.setsid
        )

        rospy.loginfo("Enregistrement terrain (simu) démarré — conduite au "
                       "teleop, aucune navigation autonome lancée.")
        rospy.loginfo(f"Run : {self.run_dir}")
        rospy.loginfo(f"Bag : {self.bag_path}")
        rospy.loginfo("Ctrl+C pour arrêter : ferme le bag et sauvegarde "
                       "le nuage global (LIO-SAM), si disponible.")

    def _request_stop(self, signum, frame):
        # Handler minimal : on se contente de lever le drapeau, tout le
        # travail de nettoyage se fait dans la boucle principale, avec
        # rospy encore pleinement actif (cf. note sur disable_signals).
        self._stop_requested = True

    def _stop(self):
        if self.bag_proc:
            rospy.loginfo("Arrêt du rosbag...")
            os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)
            self.bag_proc.wait()
            rospy.loginfo("✓ Bag fermé")

        self._save_pcd_map()

        rospy.loginfo(f"✓ Session terminée : {self.run_dir}")
        rospy.signal_shutdown("record_field_test_sim terminé")

    def _save_pcd_map(self):
        if not _HAS_LIO_SAM_SRV:
            rospy.logwarn("lio_sam.srv introuvable — PCD ignoré")
            return
        try:
            rospy.wait_for_service("/lio_sam/save_map", timeout=10.0)
        except rospy.ROSException:
            rospy.logwarn("/lio_sam/save_map indisponible (LIO-SAM non "
                          "lancé ? mode debug ?) — PCD ignoré")
            return

        try:
            save_pcd = rospy.ServiceProxy('/lio_sam/save_map', LioSamSaveMap)
            resp = save_pcd(resolution=0.0, destination=self.run_dir + '/')
            if resp.success:
                self._relocate_pcd()
            else:
                rospy.logwarn("  save_map : success=false")
        except rospy.ServiceException as e:
            rospy.logwarn(f"Service /lio_sam/save_map échoué : {e}")

    def _relocate_pcd(self):
        """Le service LIO-SAM save_map concatène $HOME + destination tel
        quel (pas un join de chemin absolu, cf. mapOptimization.cpp) : un
        destination commençant par '/' finit donc sous $HOME/<destination>
        au lieu de <destination>. On cherche les .pcd à cet endroit et on
        les déplace dans run_dir si besoin."""
        if glob.glob(os.path.join(self.run_dir, '*.pcd')):
            rospy.loginfo("✓ Nuage global sauvegardé (GlobalMap.pcd)")
            return

        home = os.environ.get('HOME', '')
        quirky_dir = home + self.run_dir + '/'
        found = glob.glob(os.path.join(quirky_dir, '*.pcd'))
        if not found:
            rospy.logwarn(f"Aucun .pcd trouvé ni dans {self.run_dir} ni "
                          f"dans {quirky_dir} — vérifier manuellement "
                          f"(ex: find {home} -iname '*.pcd' -mmin -5).")
            return

        for f in found:
            shutil.move(f, self.run_dir)
        rospy.loginfo(f"✓ {len(found)} fichier(s) .pcd déplacé(s) depuis "
                      f"{quirky_dir} vers {self.run_dir}")

    def run(self):
        rate = rospy.Rate(1)
        elapsed = 0
        while not self._stop_requested:
            rate.sleep()
            elapsed += 1
            if elapsed % 30 == 0:
                rospy.loginfo(f"... enregistrement en cours "
                              f"({elapsed // 60}min{elapsed % 60:02d}s)")
        self._stop()


if __name__ == '__main__':
    recorder = FieldTestRecorder()
    recorder.run()
