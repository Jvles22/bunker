#!/usr/bin/env python3
"""
remove_ground.py — Supprime les points au sol d'un fichier PCD et lance pcl_viewer.

Méthodes disponibles :
  ransac  : détecte le plan sol dominant par RANSAC et retire ses inliers.
  height  : retire tous les points sous un seuil de hauteur absolu.
  both    : RANSAC d'abord, puis filtre de hauteur en complément.

Usage :
  python3 remove_ground.py                               # GlobalMap.pcd par défaut
  python3 remove_ground.py /chemin/vers/run.pcd
  python3 remove_ground.py run.pcd -m height --min-z 0.05
  python3 remove_ground.py run.pcd -m both -d 0.12 --min-z 0.08
  python3 remove_ground.py run.pcd --no-view             # ne pas ouvrir pcl_viewer

Dépendance : pip install open3d
"""

import argparse
import os
import subprocess
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("[Erreur] open3d non installé. Lancez :")
    print("         pip install open3d")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────────
# Méthodes de suppression du sol
# ─────────────────────────────────────────────────────────────────────────────

def remove_ground_ransac(pcd, distance_threshold: float, num_iterations: int = 1000):
    """
    Détecte le plan sol dominant par RANSAC et supprime ses inliers.

    distance_threshold : distance maximale d'un point au plan pour être
                         considéré comme appartenant au sol (en mètres).
    """
    n_before = len(pcd.points)
    plane_model, inliers = pcd.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=num_iterations,
    )
    a, b, c, d = plane_model
    print(f"  Plan détecté  : {a:+.4f}x {b:+.4f}y {c:+.4f}z {d:+.4f} = 0")
    print(f"  Normale       : ({a:.3f}, {b:.3f}, {c:.3f})  "
          f"→ inclinaison par rapport à Z = "
          f"{np.degrees(np.arccos(abs(c) / np.sqrt(a**2 + b**2 + c**2))):.1f}°")

    filtered = pcd.select_by_index(inliers, invert=True)
    print(f"  Points retirés (RANSAC)    : {len(inliers):,}  "
          f"({100 * len(inliers) / n_before:.1f} %)")
    print(f"  Points conservés           : {len(filtered.points):,}")
    return filtered


def remove_ground_height(pcd, min_z: float):
    """
    Retire tous les points dont la coordonnée Z est inférieure à min_z.

    Utile pour nettoyer les inliers résiduels après RANSAC, ou en remplacement
    si la carte est bien nivelée (sol ≈ z = 0).
    """
    n_before = len(pcd.points)
    points = np.asarray(pcd.points)
    keep = np.where(points[:, 2] > min_z)[0]
    filtered = pcd.select_by_index(keep)
    removed = n_before - len(filtered.points)
    print(f"  Points retirés (z ≤ {min_z:.3f} m) : {removed:,}  "
          f"({100 * removed / n_before:.1f} %)")
    print(f"  Points conservés              : {len(filtered.points):,}")
    return filtered


# ─────────────────────────────────────────────────────────────────────────────
# Point d'entrée
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Supprime les points au sol d'un PCD et lance pcl_viewer.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="/home/jules/tmp/bunker_maps/GlobalMap.pcd",
        help="Fichier PCD d'entrée  (défaut : /home/jules/tmp/bunker_maps/GlobalMap.pcd)",
    )
    parser.add_argument(
        "--output", "-o",
        default=None,
        metavar="FICHIER",
        help="Fichier PCD de sortie  (défaut : <input>_no_ground.pcd, même dossier)",
    )
    parser.add_argument(
        "--method", "-m",
        choices=["ransac", "height", "both"],
        default="ransac",
        help="Méthode : ransac | height | both  (défaut : ransac)",
    )
    parser.add_argument(
        "--distance-threshold", "-d",
        type=float,
        default=0.15,
        metavar="METRES",
        help="Seuil RANSAC : épaisseur de la couche sol à retirer [m]  (défaut : 0.15)",
    )
    parser.add_argument(
        "--min-z",
        type=float,
        default=0.10,
        metavar="METRES",
        help="Seuil de hauteur pour la méthode 'height' / 'both' [m]  (défaut : 0.10)",
    )
    parser.add_argument(
        "--ransac-iterations",
        type=int,
        default=1000,
        metavar="N",
        help="Nombre d'itérations RANSAC  (défaut : 1000)",
    )
    parser.add_argument(
        "--no-view",
        action="store_true",
        help="Ne pas lancer pcl_viewer après le traitement",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    input_path = os.path.expanduser(args.input)
    if not os.path.isfile(input_path):
        print(f"[Erreur] Fichier introuvable : {input_path}")
        sys.exit(1)

    # Chemin de sortie : même dossier que l'entrée, suffixe _no_ground
    if args.output is None:
        base, ext = os.path.splitext(input_path)
        output_path = base + "_no_ground" + ext
    else:
        output_path = os.path.expanduser(args.output)

    # ── Chargement ──────────────────────────────────────────────────────────
    print(f"\n[1/3] Chargement de : {input_path}")
    pcd = o3d.io.read_point_cloud(input_path)
    n_total = len(pcd.points)
    if n_total == 0:
        print("[Erreur] Le nuage de points est vide.")
        sys.exit(1)
    print(f"  Points chargés : {n_total:,}")

    # Statistiques d'élévation (utile pour choisir --min-z)
    pts = np.asarray(pcd.points)
    print(f"  Z min / moy / max : "
          f"{pts[:, 2].min():.3f} / {pts[:, 2].mean():.3f} / {pts[:, 2].max():.3f} m")

    # ── Suppression du sol ──────────────────────────────────────────────────
    print(f"\n[2/3] Suppression du sol  (méthode : {args.method})")

    if args.method == "ransac":
        pcd = remove_ground_ransac(
            pcd,
            distance_threshold=args.distance_threshold,
            num_iterations=args.ransac_iterations,
        )
    elif args.method == "height":
        pcd = remove_ground_height(pcd, min_z=args.min_z)
    elif args.method == "both":
        print("  — Passe 1 : RANSAC")
        pcd = remove_ground_ransac(
            pcd,
            distance_threshold=args.distance_threshold,
            num_iterations=args.ransac_iterations,
        )
        print("  — Passe 2 : seuil de hauteur")
        pcd = remove_ground_height(pcd, min_z=args.min_z)

    # ── Sauvegarde ──────────────────────────────────────────────────────────
    print(f"\n[3/3] Sauvegarde dans : {output_path}")
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    success = o3d.io.write_point_cloud(output_path, pcd)
    if not success:
        print("[Erreur] Échec de l'écriture du fichier PCD.")
        sys.exit(1)
    print("  OK")

    # ── Visualisation ───────────────────────────────────────────────────────
    if not args.no_view:
        print(f"\nLancement de pcl_viewer sur : {output_path}")
        try:
            subprocess.run(["pcl_viewer", output_path], check=True)
        except FileNotFoundError:
            print("[Erreur] pcl_viewer introuvable. Installez pcl-tools :")
            print("         sudo apt install pcl-tools")
            sys.exit(1)


if __name__ == "__main__":
    main()
