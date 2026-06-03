#!/usr/bin/env python3
"""
param_tuner.py  —  Tuning TEB en 3 phases découplées
=====================================================

PHASE 1 — Générer la grille (sans ROS, avant la simulation) :
    python3 param_tuner.py --generate
    python3 param_tuner.py --generate --full       # grille étendue (×4 combos)

PHASE 2 — Lancer les tests via l'orchestrateur (recommandé) :
    bash tuning_run.sh                             # gère les 3 batches + restart auto

    Ou manuellement batch par batch (9 combos chacun) :
    python3 param_tuner.py --run --batch 0         # combos 1-9
    python3 param_tuner.py --run --batch 1         # combos 10-18
    python3 param_tuner.py --run --batch 2         # combos 19-27

PHASE 3 — Fusionner les batches et sélectionner :
    python3 param_tuner.py --merge                 # affiche classement interactif
    python3 param_tuner.py --merge --auto-select   # sélectionne #1 automatiquement
    python3 param_tuner.py --select                # sélection sur un CSV existant
    python3 param_tuner.py --select --apply        # applique aussi via dynreconf
"""

import argparse
import csv
import itertools
import json
import math
import os
import sys
import time
from datetime import datetime

# ====================================================================== #
#                          CONFIGURATION GLOBALE                          #
# ====================================================================== #

OUTPUT_DIR   = os.path.expanduser("~/bunker_benchmark/tuning")
BATCH_SIZE   = 9     # combos par run (27 total → 3 batches de 9)
TEB_YAML     = os.path.expanduser(
    "~/projet_bunker/src/bunker_slam/config/move_base/teb_params.yaml"
)

# Vitesse fixe pendant le tuning (identique pour toutes les combinaisons)
SPEED_TEST   = 0.5   # m/s

# Parcours de test
SQUARE_SIZE  = 15.0  # m — carré plus petit pour aller plus vite
WP_STEP_M    = 3.0
WP_ADVANCE_M = 1.5
WP_TIMEOUT_S = 90.0   # 1.5 min wall ≈ 27–45 s sim à RTF 0.3–0.5
CORNER_TOL_M = 0.6
SEG_LABELS   = ["S1_Est", "S2_Nord", "S3_Ouest", "S4_Sud"]

# ====================================================================== #
#            GRILLE TEB — PARAMÈTRES FIXES ET PARAMÈTRES CHERCHÉS         #
# ====================================================================== #

# Paramètres maintenus constants (contraintes physiques ou déjà calibrés)
TEB_FIXED = {
    "max_vel_x":             SPEED_TEST,
    "max_vel_x_backwards":   0.05,     # quasi nul : évite le recul sur pente
    "max_vel_theta":         0.6,      # rotations douces
    "acc_lim_x":             0.8,      # couple de montée
    "acc_lim_theta":         0.8,
    "xy_goal_tolerance":     1.0,
    "yaw_goal_tolerance":    0.5,
    "weight_kinematics_nh":  1000.0,   # contrainte non-holonome stricte
    "no_inner_iterations":   5,
    "no_outer_iterations":   4,
}

# ── Grille CORE (27 combinaisons, ~15 min en simulation) ──────────────────
# Les 3 paramètres les plus impactants pour la navigation en forêt/terrain varié.
TEB_SEARCH_CORE = {
    # Poids time-optimality : 0.5 = trajectoire lisse/prudente,
    #                         2.0 = trajectoire plus directe sans sacrifier l'évitement.
    # 5.0 écarté : trop agressif en forêt dense, sacrifie la marge aux obstacles.
    "weight_optimaltime": [0.5, 1.0, 2.0],

    # Poids évitement obstacles : centré autour de la valeur courante (50).
    # 25.0 écarté : trop faible pour des couloirs avec poteaux/clôtures.
    # 100.0 écarté : bloque le planificateur dans les passages étroits.
    "weight_obstacle": [30.0, 50.0, 75.0],

    # Poids marche avant : utile en pente, moins critique sur terrain plat.
    # 20.0 écarté : surtaxe les virages sur terrain plat, cause des blocages locaux.
    "weight_kinematics_forward_drive": [5.0, 10.0, 15.0],
}

# ── Paramètres additionnels pour grille FULL (×4, soit 108 combinaisons) ──
TEB_SEARCH_EXTRA = {
    # Distance minimale aux obstacles : 0.2 = plus agressif près des rampes,
    #                                   0.3 = plus de marge (peut contourner)
    "min_obstacle_dist": [0.2, 0.3],

    # Résolution temporelle de la trajectoire :
    # 0.25 = trajectoire fine / réactive, 0.35 = plus lisse / moins de calcul
    "dt_ref": [0.25, 0.35],
}

# Annotations lisibles pour l'affichage de la phase --generate
PARAM_ANNOTATIONS = {
    "weight_optimaltime": (
        "Poids time-optimality",
        "Trajectoire directe (haut) vs lisse/prudente (bas). "
        "Valeur courante : 1.0  |  Max recommandé forêt : 2.0"
    ),
    "weight_obstacle": (
        "Poids évitement obstacles",
        "Marge autour des obstacles. Trop fort → blocage dans couloirs étroits. "
        "Valeur courante : 50.0  |  Plage forêt : 30–75"
    ),
    "weight_kinematics_forward_drive": (
        "Poids marche avant",
        "Contraint la marche avant. Moins critique sur terrain plat que sur rampe. "
        "Valeur courante : 10.0  |  Plage recommandée : 5–15"
    ),
    "min_obstacle_dist": (
        "Distance min aux obstacles (m)",
        "Proximité tolérée aux obstacles. Valeur courante : 0.25 m"
    ),
    "dt_ref": (
        "Résolution temporelle trajectoire (s)",
        "Finesse de l'horizon TEB. Valeur courante : 0.30 s"
    ),
}

# ====================================================================== #
#                         UTILITAIRES AFFICHAGE                           #
# ====================================================================== #

RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[32m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
RED    = "\033[31m"
DIM    = "\033[2m"


def _latest_file(pattern_prefix, ext):
    """Retourne le fichier le plus récent matching OUTPUT_DIR/<prefix>*<ext>."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    files = [
        os.path.join(OUTPUT_DIR, f)
        for f in os.listdir(OUTPUT_DIR)
        if f.startswith(pattern_prefix) and f.endswith(ext)
    ]
    return max(files, key=os.path.getmtime) if files else None


def _fmt_score(score, best_score):
    if score == best_score:
        return f"{GREEN}{BOLD}{score:8.1f}{RESET}"
    elif score < best_score * 1.1:
        return f"{GREEN}{score:8.1f}{RESET}"
    elif score < best_score * 1.3:
        return f"{YELLOW}{score:8.1f}{RESET}"
    return f"{score:8.1f}"


# ====================================================================== #
#                         PHASE 1 — GENERATE                              #
# ====================================================================== #

def cmd_generate(full=False):
    """
    Génère la grille de paramètres TEB et la sauvegarde en JSON.
    Ne nécessite pas que ROS soit lancé.
    """
    search = dict(TEB_SEARCH_CORE)
    if full:
        search.update(TEB_SEARCH_EXTRA)

    param_names  = list(search.keys())
    param_values = [search[k] for k in param_names]
    combos       = list(itertools.product(*param_values))

    print(f"\n{BOLD}{'='*66}{RESET}")
    print(f"{BOLD}  TUNING TEB — PHASE 1 : GÉNÉRATION DE LA GRILLE{RESET}")
    print(f"{BOLD}{'='*66}{RESET}")
    print(f"  Grille : {'FULL' if full else 'CORE'}  "
          f"({len(param_names)} paramètres × {len(combos)} combinaisons)")
    print()

    # ── Tableau des paramètres cherchés ──────────────────────────────────
    print(f"{BOLD}  Paramètres de recherche :{RESET}")
    print(f"  {'Paramètre':<40} {'Valeurs testées':<30} {'Rôle'}")
    print(f"  {'-'*40} {'-'*30} {'-'*35}")

    for name in param_names:
        vals  = search[name]
        title, desc = PARAM_ANNOTATIONS.get(name, (name, ""))
        vals_str = "  ".join(str(v) for v in vals)
        print(f"  {CYAN}{name:<40}{RESET} {vals_str:<30} {DIM}{title}{RESET}")
        print(f"  {'':<40} {'':<30} {DIM}{desc}{RESET}")
        print()

    # ── Paramètres fixes ─────────────────────────────────────────────────
    print(f"{BOLD}  Paramètres fixes (non variés) :{RESET}")
    for k, v in TEB_FIXED.items():
        print(f"  {DIM}{k:<40} {v}{RESET}")

    print()
    print(f"  Vitesse de test fixe : {SPEED_TEST} m/s")
    print(f"  Parcours             : carré {SQUARE_SIZE} m")
    print()

    # ── Liste de toutes les combinaisons ─────────────────────────────────
    print(f"{BOLD}  Toutes les combinaisons ({len(combos)}) :{RESET}")
    print(f"  {'#':<4}  " + "  ".join(f"{n:<38}" for n in param_names))
    print(f"  {'-'*4}  " + "  ".join('-'*38 for _ in param_names))
    for i, combo in enumerate(combos, 1):
        row = "  ".join(f"{v:<38}" for v in combo)
        print(f"  {i:<4}  {row}")

    # ── Estimation de durée ───────────────────────────────────────────────
    # ~3 min par combo (carré 15m à 0.5 m/s + retour à l'origine)
    estimated_min = len(combos) * 3
    print()
    print(f"  Durée estimée : ~{estimated_min} min "
          f"({len(combos)} combos × ~3 min)")

    # ── Sauvegarde JSON ───────────────────────────────────────────────────
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts       = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile  = os.path.join(OUTPUT_DIR, f"teb_grid_{ts}.json")

    grid_data = {
        "generated_at": ts,
        "mode": "full" if full else "core",
        "speed_test": SPEED_TEST,
        "square_size": SQUARE_SIZE,
        "fixed_params": TEB_FIXED,
        "search_params": {k: list(v) for k, v in search.items()},
        "combos": [dict(zip(param_names, c)) for c in combos],
        "total_combos": len(combos),
    }

    with open(outfile, "w") as f:
        json.dump(grid_data, f, indent=2)

    print()
    print(f"{BOLD}{'='*66}{RESET}")
    print(f"{GREEN}  Grille sauvegardée : {outfile}{RESET}")
    print(f"  → Lancez la simulation, puis :")
    print(f"  {CYAN}python3 param_tuner.py --run{RESET}")
    print(f"{BOLD}{'='*66}{RESET}\n")


# ====================================================================== #
#                         PHASE 2 — RUN                                   #
# ====================================================================== #

def cmd_run(grid_file=None, batch=None):
    """
    Charge la grille JSON et exécute chaque combinaison en simulation.
    batch=None  → exécute tous les combos en une fois
    batch=N     → exécute seulement les combos du batch N (0-indexé, BATCH_SIZE combos)
    Nécessite la stack complète (tuning_liosam + tuning_movebase déjà lancés).
    """
    # Trouver la grille
    if grid_file is None:
        grid_file = _latest_file("teb_grid_", ".json")
        if grid_file is None:
            print(f"{RED}Aucune grille trouvée dans {OUTPUT_DIR}.{RESET}")
            print(f"Lancez d'abord : python3 param_tuner.py --generate")
            sys.exit(1)
        print(f"Grille chargée : {grid_file}")

    with open(grid_file) as f:
        grid = json.load(f)

    combos_all  = grid["combos"]
    fixed       = grid["fixed_params"]
    square_size = grid.get("square_size", SQUARE_SIZE)

    # Découpage en batch
    if batch is not None:
        start  = batch * BATCH_SIZE
        end    = start + BATCH_SIZE
        combos = combos_all[start:end]
        if not combos:
            print(f"{RED}Batch {batch} vide (grille : {len(combos_all)} combos, "
                  f"BATCH_SIZE={BATCH_SIZE}).{RESET}")
            sys.exit(1)
        batch_tag  = f"_b{batch}"
        batch_info = f"batch {batch+1}/3  (combos {start+1}–{min(end, len(combos_all))} "  \
                     f"sur {len(combos_all)})"
    else:
        combos     = combos_all
        batch_tag  = ""
        batch_info = f"run complet ({len(combos)} combos)"

    # ── Imports ROS (seulement ici, simulation active) ───────────────────
    import rospy
    import actionlib
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from nav_msgs.msg       import Odometry
    from tf.transformations import euler_from_quaternion, quaternion_from_euler
    from std_srvs.srv       import Empty as EmptySrv

    rospy.init_node("teb_param_tuner", anonymous=True)

    runner = _TebRunner(fixed, square_size)

    rospy.loginfo(f"Grille     : {os.path.basename(grid_file)}")
    rospy.loginfo(f"Exécution  : {batch_info}")
    rospy.loginfo(f"Vitesse    : {grid.get('speed_test', SPEED_TEST)} m/s")

    # Attente odométrie
    rospy.loginfo("Attente odométrie...")
    deadline = time.time() + 30.0
    while not runner.odom_ok and not rospy.is_shutdown():
        if time.time() > deadline:
            rospy.logfatal("Timeout odométrie !")
            return
        rospy.sleep(0.5)

    # ── Vérification pré-navigation : LIO-SAM doit s'être initialisé près de (0,0).
    # Le robot vient d'être respawné à Gazebo (0,0) et LIO-SAM vient de démarrer.
    # Si l'odométrie initiale est loin de (0,0), LIO-SAM a divergé lors de son init
    # (spike IMU de téléportation non absorbé) : naviguer vers (0,0) via move_base
    # enverrait le robot physiquement hors de la map → abort immédiat.
    INIT_POS_ABORT_M = 3.0   # seuil d'abandon — au-delà, la divergence est trop forte
    INIT_POS_WARN_M  = 1.5   # seuil d'avertissement — légère dérive acceptable
    init_err = math.hypot(runner.x, runner.y)
    if init_err > INIT_POS_ABORT_M:
        rospy.logfatal(
            f"[INIT ABORT] LIO-SAM s'est initialisé à ({runner.x:.2f}, {runner.y:.2f}) "
            f"— erreur={init_err:.2f} m > {INIT_POS_ABORT_M} m. "
            f"LIO-SAM a divergé (spike IMU de téléportation ?). "
            f"Batch abandonné pour éviter d'envoyer le robot hors de la map. "
            f"Relancez tuning_run.sh avec un LIOSAM_READY_SLEEP plus grand."
        )
        return
    elif init_err > INIT_POS_WARN_M:
        rospy.logwarn(
            f"[INIT WARN] LIO-SAM s'est initialisé à ({runner.x:.2f}, {runner.y:.2f}) "
            f"— erreur={init_err:.2f} m. Légère dérive, le batch continue "
            f"mais les résultats peuvent être légèrement biaisés."
        )
    else:
        rospy.loginfo(f"LIO-SAM init OK : ({runner.x:.2f}, {runner.y:.2f})  err={init_err:.2f} m")

    # Retour à l'origine (0, 0) pour que chaque batch commence au même point.
    rospy.loginfo("Retour à l'origine (0.0, 0.0) avant le batch...")
    runner.return_to_origin(0.0, 0.0)
    rospy.sleep(3.0)

    origin_x, origin_y = runner.x, runner.y
    pos_err = math.hypot(origin_x, origin_y)
    if pos_err > INIT_POS_WARN_M:
        rospy.logwarn(
            f"[POST-RETURN WARN] Après return_to_origin, LIO-SAM estime "
            f"({origin_x:.2f}, {origin_y:.2f}) — erreur={pos_err:.2f} m. "
            f"Les résultats de ce batch peuvent être biaisés."
        )
    else:
        rospy.loginfo(f"Origine confirmée : ({origin_x:.2f}, {origin_y:.2f})  err={pos_err:.2f} m")

    segments, ref_pts = _generate_course(origin_x, origin_y, square_size)

    # ── CSV résultats ─────────────────────────────────────────────────────
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts        = datetime.now().strftime("%Y%m%d_%H%M%S")
    grid_tag  = os.path.basename(grid_file).replace("teb_grid_", "").replace(".json", "")
    csv_path  = os.path.join(OUTPUT_DIR, f"teb_results_{grid_tag}{batch_tag}_{ts}.csv")

    param_names = list(combos[0].keys()) if combos else []

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["combo_id"] + param_names +
            ["duration_s", "cte_avg_m", "cte_max_m",
             "success_rate", "distance_m", "seg_ok", "seg_total",
             "collision", "score"]
        )

        for i, combo in enumerate(combos):
            if rospy.is_shutdown():
                break

            params = dict(fixed)
            params.update(combo)

            combo_str = "  ".join(f"{k}={v}" for k, v in combo.items())
            rospy.loginfo("")
            rospy.loginfo(f"[{i+1}/{len(combos)}] {combo_str}")

            if not runner.apply_params(params):
                rospy.logwarn("  Skip — params non appliqués")
                continue

            runner.clear()
            rospy.sleep(1.0)

            metrics = runner.run_test(segments, ref_pts)

            # Collision → gains invalides, score disqualifiant
            if metrics.get("collision"):
                score = 9999.0
                rospy.logwarn(
                    f"  ✗ COLLISION — gains invalides  "
                    f"(dur={metrics['duration']:.1f}s, "
                    f"seg={metrics['seg_ok']}/{metrics['seg_total']})"
                )
            else:
                # Score : plus bas = mieux
                score = (
                    metrics["duration"] +
                    metrics["cte_avg"] * 200.0 +
                    (1.0 - metrics["success_rate"]) * 1000.0
                )
                rospy.loginfo(
                    f"  ✓ dur={metrics['duration']:.1f}s  "
                    f"CTE={metrics['cte_avg']:.3f}m (max={metrics['cte_max']:.3f}m)  "
                    f"OK={metrics['seg_ok']}/{metrics['seg_total']}  "
                    f"score={score:.1f}"
                )
            metrics["score"] = round(score, 2)

            writer.writerow(
                [i + 1] + [combo[k] for k in param_names] + [
                    metrics["duration"], metrics["cte_avg"], metrics["cte_max"],
                    metrics["success_rate"], metrics["distance"],
                    metrics["seg_ok"], metrics["seg_total"],
                    int(metrics.get("collision", False)), metrics["score"],
                ]
            )
            f.flush()

            if i < len(combos) - 1:
                if metrics.get("collision"):
                    # Après une collision : NE PAS téléporter — la téléportation
                    # casse la cohérence LIO-SAM (LIO-SAM ignore le saut Gazebo,
                    # self.x/y stagnent → STUCK_TIMEOUT immédiat sur le combo suivant).
                    # Vider les costmaps pour débloquer le planificateur local,
                    # puis naviguer normalement vers l'origine.
                    rospy.loginfo("  Retour à l'origine (navigation, pas de respawn)...")
                    runner.clear()
                    rospy.sleep(1.5)
                runner.return_to_origin(origin_x, origin_y)

    rospy.loginfo("")
    rospy.loginfo(f"{'='*60}")
    rospy.loginfo(f"  Résultats sauvegardés : {csv_path}")
    rospy.loginfo(f"  → Pour sélectionner :")
    rospy.loginfo(f"    python3 param_tuner.py --select")
    rospy.loginfo(f"{'='*60}")


# ====================================================================== #
#                   PHASE 2b — MERGE (entre batches)                      #
# ====================================================================== #

def cmd_merge(auto_select=False, apply=False):
    """
    Fusionne tous les CSV de batches issus de la dernière grille,
    puis lance la sélection (interactive ou automatique).
    """
    grid_file = _latest_file("teb_grid_", ".json")
    if grid_file is None:
        print(f"{RED}Aucune grille trouvée dans {OUTPUT_DIR}.{RESET}")
        sys.exit(1)

    grid_tag = (os.path.basename(grid_file)
                .replace("teb_grid_", "").replace(".json", ""))

    # Trouver tous les CSV de batches pour cette grille
    batch_files = sorted(
        os.path.join(OUTPUT_DIR, f)
        for f in os.listdir(OUTPUT_DIR)
        if f.startswith(f"teb_results_{grid_tag}_b") and f.endswith(".csv")
    )

    if not batch_files:
        print(f"{RED}Aucun CSV de batch trouvé pour la grille '{grid_tag}'.{RESET}")
        print(f"Fichiers attendus : teb_results_{grid_tag}_b0_*.csv, etc.")
        sys.exit(1)

    print(f"\n{BOLD}Fusion de {len(batch_files)} batch(es) :{RESET}")
    all_rows = []
    header   = None
    for bf in batch_files:
        with open(bf, newline="") as f:
            reader = csv.DictReader(f)
            rows   = list(reader)
            if header is None:
                header = reader.fieldnames
            all_rows.extend(rows)
        print(f"  {os.path.basename(bf)}  ({len(rows)} combos)")

    if not all_rows:
        print(f"{RED}Aucun résultat à fusionner.{RESET}")
        sys.exit(1)

    # Sauvegarder le CSV fusionné
    ts          = datetime.now().strftime("%Y%m%d_%H%M%S")
    merged_path = os.path.join(OUTPUT_DIR, f"teb_results_{grid_tag}_merged_{ts}.csv")
    with open(merged_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writeheader()
        writer.writerows(all_rows)

    print(f"\n{GREEN}CSV fusionné : {merged_path}{RESET}")
    print(f"Total         : {len(all_rows)} combinaisons\n")

    if auto_select:
        # Sélection automatique : #1 sans interaction
        _auto_select_best(merged_path, apply=apply)
    else:
        cmd_select(merged_path, apply=apply)


def _auto_select_best(results_file, apply=False):
    """Sélectionne automatiquement le meilleur combo (score le plus bas)."""
    with open(results_file, newline="") as f:
        reader = csv.DictReader(f)
        rows   = list(reader)

    if not rows:
        print(f"{RED}Fichier résultats vide.{RESET}")
        return

    metric_cols = ["duration_s", "cte_avg_m", "cte_max_m",
                   "success_rate", "distance_m", "seg_ok", "seg_total", "score"]
    fixed_meta  = {"combo_id"}
    param_names = [c for c in rows[0].keys()
                   if c not in metric_cols and c not in fixed_meta]

    for row in rows:
        for col in metric_cols + param_names:
            try:
                row[col] = float(row[col])
            except (ValueError, KeyError):
                pass

    rows.sort(key=lambda r: r["score"])
    best = rows[0]

    print(f"{BOLD}{'='*60}{RESET}")
    print(f"{BOLD}  SÉLECTION AUTOMATIQUE — MEILLEUR COMBO{RESET}")
    print(f"{BOLD}{'='*60}{RESET}\n")

    changed_params = {}
    current_yaml   = _read_current_yaml()

    print(f"  {'Paramètre':<42} {'Valeur':>15}  {'YAML actuel':>15}")
    print(f"  {'-'*42} {'-'*15}  {'-'*15}")
    for name in param_names:
        val = best[name]
        cur = current_yaml.get(name, "?")
        changed_params[name] = val
        marker = f" {YELLOW}←{RESET}" if str(val) != str(cur) else ""
        print(f"  {name:<42} {val:>15}  {str(cur):>15}{marker}")

    print(f"\n  Score   : {best['score']:.1f}")
    print(f"  Durée   : {best['duration_s']:.1f} s")
    print(f"  CTE moy : {best['cte_avg_m']:.3f} m")
    print(f"  Succès  : {float(best['success_rate'])*100:.0f}%\n")

    _save_to_yaml(changed_params, current_yaml)
    if apply:
        _apply_via_dynreconf(changed_params)

    print(f"\n{GREEN}  Paramètres optimaux sauvegardés dans teb_params.yaml.{RESET}\n")


# ====================================================================== #
#                         PHASE 3 — SELECT                                #
# ====================================================================== #

def cmd_select(results_file=None, apply=False):
    """
    Charge les résultats CSV, affiche un classement interactif et
    laisse l'utilisateur choisir le meilleur paramétrage.
    Propose ensuite de sauvegarder dans teb_params.yaml.
    Ne nécessite pas ROS (sauf si --apply).
    """
    if results_file is None:
        results_file = _latest_file("teb_results_", ".csv")
        if results_file is None:
            print(f"{RED}Aucun résultat trouvé dans {OUTPUT_DIR}.{RESET}")
            print("Lancez d'abord : python3 param_tuner.py --run")
            sys.exit(1)
        print(f"Résultats chargés : {results_file}")

    # ── Charger le CSV ────────────────────────────────────────────────────
    with open(results_file, newline="") as f:
        reader = csv.DictReader(f)
        rows   = list(reader)

    if not rows:
        print(f"{RED}Fichier vide.{RESET}")
        sys.exit(1)

    # Colonnes de métriques
    metric_cols = ["duration_s", "cte_avg_m", "cte_max_m",
                   "success_rate", "distance_m", "seg_ok", "seg_total", "score"]
    fixed_meta  = {"combo_id"}
    param_names = [c for c in rows[0].keys()
                   if c not in metric_cols and c not in fixed_meta]

    # Convertir en numérique
    for row in rows:
        for col in metric_cols + param_names:
            try:
                row[col] = float(row[col])
            except (ValueError, KeyError):
                pass

    # Trier par score croissant
    rows.sort(key=lambda r: r["score"])
    best_score = rows[0]["score"]

    # ── Affichage du classement complet ───────────────────────────────────
    print(f"\n{BOLD}{'='*70}{RESET}")
    print(f"{BOLD}  TUNING TEB — PHASE 3 : CLASSEMENT DES RÉSULTATS{RESET}")
    print(f"{BOLD}{'='*70}{RESET}")
    print(f"  Fichier : {os.path.basename(results_file)}")
    print(f"  Combos  : {len(rows)}\n")

    # En-tête du tableau
    pname_width = max(len(n) for n in param_names) + 2
    p_header    = "  ".join(f"{n:<{pname_width}}" for n in param_names)
    print(f"  {'#':<4}  {p_header}  {'dur(s)':<8}  {'CTE moy':<9}  "
          f"{'CTE max':<9}  {'OK%':<6}  {'score':<9}")
    print(f"  {'-'*4}  {'-'*(pname_width*len(param_names) + 2*(len(param_names)-1))}"
          f"  {'-'*8}  {'-'*9}  {'-'*9}  {'-'*6}  {'-'*9}")

    for rank, row in enumerate(rows, 1):
        p_vals  = "  ".join(f"{row[n]:<{pname_width}}" for n in param_names)
        ok_pct  = f"{row['success_rate']*100:.0f}%"
        score_s = _fmt_score(row["score"], best_score)

        prefix = f"{GREEN}{BOLD}▶ {RESET}" if rank <= 3 else "  "
        print(
            f"{prefix}{rank:<4}  {p_vals}  "
            f"{row['duration_s']:<8.1f}  "
            f"{row['cte_avg_m']:<9.3f}  "
            f"{row['cte_max_m']:<9.3f}  "
            f"{ok_pct:<6}  "
            f"{score_s}"
        )

    # ── Légende du score ──────────────────────────────────────────────────
    print(f"\n  {DIM}Score = durée(s) + CTE_moy×200 + (1-succès)×1000  "
          f"[plus bas = mieux]{RESET}")

    # ── Sélection interactive ─────────────────────────────────────────────
    print(f"\n{BOLD}  Choisissez un paramétrage :{RESET}")
    print(f"  Tapez un numéro (1–{len(rows)}), "
          f"ou {CYAN}Enter{RESET} pour le meilleur (#1), "
          f"ou {CYAN}q{RESET} pour quitter.")

    while True:
        try:
            raw = input(f"\n  Votre choix > ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\n  Annulé.")
            return

        if raw.lower() in ("q", "quit", "exit"):
            print("  Annulé.")
            return

        if raw == "":
            choice = 1
        else:
            try:
                choice = int(raw)
            except ValueError:
                print(f"  {RED}Entrez un nombre entre 1 et {len(rows)}.{RESET}")
                continue

        if 1 <= choice <= len(rows):
            break
        print(f"  {RED}Numéro hors plage (1–{len(rows)}).{RESET}")

    selected = rows[choice - 1]

    # ── Résumé du choix ───────────────────────────────────────────────────
    print(f"\n{BOLD}{'='*66}{RESET}")
    print(f"{BOLD}  PARAMÉTRAGE SÉLECTIONNÉ  (#{choice}){RESET}")
    print(f"{BOLD}{'='*66}{RESET}\n")

    print(f"  {'Paramètre':<42} {'Valeur sélectionnée':>20}  {'Valeur YAML actuelle':>22}")
    print(f"  {'-'*42} {'-'*20}  {'-'*22}")

    # Lire valeurs actuelles du YAML pour comparaison
    current_yaml = _read_current_yaml()

    changed_params = {}
    for name in param_names:
        val      = selected[name]
        cur      = current_yaml.get(name, "?")
        changed  = (str(val) != str(cur))
        changed_params[name] = val
        marker = f" {YELLOW}←{RESET}" if changed else ""
        print(f"  {name:<42} {val:>20}  {str(cur):>22}{marker}")

    print(f"\n  {'Métrique':<30} {'Valeur'}")
    print(f"  {'-'*30} {'-'*15}")
    print(f"  {'Durée':<30} {selected['duration_s']:.1f} s")
    print(f"  {'CTE moyen':<30} {selected['cte_avg_m']:.3f} m")
    print(f"  {'CTE max':<30} {selected['cte_max_m']:.3f} m")
    print(f"  {'Segments réussis':<30} {selected['success_rate']*100:.0f}%")
    print(f"  {'Score':<30} {selected['score']:.1f}")

    # ── Options post-sélection ────────────────────────────────────────────
    print(f"\n{BOLD}  Que faire avec ce paramétrage ?{RESET}")
    print(f"  {CYAN}s{RESET}  — Sauvegarder dans teb_params.yaml (permanent)")
    if apply:
        print(f"  {CYAN}a{RESET}  — Appliquer via dynreconf (session en cours)")
        print(f"  {CYAN}b{RESET}  — Les deux (sauvegarder + appliquer)")
    print(f"  {CYAN}n{RESET}  — Rien, juste afficher la commande rosrun")

    try:
        action = input(f"\n  Votre choix > ").strip().lower()
    except (KeyboardInterrupt, EOFError):
        action = "n"

    if action in ("s", "b"):
        _save_to_yaml(changed_params, current_yaml)

    if apply and action in ("a", "b"):
        _apply_via_dynreconf(changed_params)

    # Toujours afficher la commande equivalente
    print(f"\n{BOLD}  Équivalent rosrun (session courante) :{RESET}")
    for name, val in changed_params.items():
        ns = "/move_base/TebLocalPlannerROS"
        print(f"  {DIM}rosrun dynamic_reconfigure dynparam set "
              f"{ns} {name} {val}{RESET}")

    print(f"\n{GREEN}  Terminé.{RESET}\n")


# ====================================================================== #
#                    HELPERS PHASE 3 — YAML / DYNRECONF                   #
# ====================================================================== #

def _read_current_yaml():
    """Lit teb_params.yaml et retourne un dict {param: valeur}."""
    result = {}
    try:
        with open(TEB_YAML) as f:
            for line in f:
                line = line.split("#")[0].strip()
                if ":" in line:
                    k, _, v = line.partition(":")
                    k = k.strip()
                    v = v.strip()
                    if k and v and not k.startswith("-"):
                        try:
                            result[k] = float(v)
                        except ValueError:
                            result[k] = v
    except FileNotFoundError:
        pass
    return result


def _save_to_yaml(new_params, current_yaml):
    """Met à jour teb_params.yaml avec les nouvelles valeurs."""
    try:
        with open(TEB_YAML) as f:
            lines = f.readlines()
    except FileNotFoundError:
        print(f"  {RED}YAML introuvable : {TEB_YAML}{RESET}")
        return

    # Backup
    backup = TEB_YAML + ".bak"
    with open(backup, "w") as f:
        f.writelines(lines)

    updated = []
    changed = []
    for line in lines:
        stripped = line.split("#")[0].strip()
        if ":" in stripped:
            k = stripped.split(":")[0].strip()
            if k in new_params:
                old_val = current_yaml.get(k, "?")
                new_val = new_params[k]
                indent  = len(line) - len(line.lstrip())
                comment = ""
                if "#" in line:
                    comment = "  " + line[line.index("#"):].rstrip()
                updated.append(" " * indent + f"{k}: {new_val}{comment}\n")
                changed.append((k, old_val, new_val))
                continue
        updated.append(line)

    with open(TEB_YAML, "w") as f:
        f.writelines(updated)

    print(f"\n  {GREEN}teb_params.yaml mis à jour :{RESET}")
    for k, old, new in changed:
        print(f"    {k:<42} {str(old):>10}  →  {GREEN}{new}{RESET}")
    print(f"  {DIM}(backup : {backup}){RESET}")


def _apply_via_dynreconf(params):
    """Applique les paramètres via dynamic_reconfigure (ROS actif requis)."""
    try:
        import rospy
        from dynamic_reconfigure.client import Client as DynClient
        if not rospy.core.is_initialized():
            rospy.init_node("teb_param_selector", anonymous=True)
        dc = DynClient("/move_base/TebLocalPlannerROS", timeout=10)
        dc.update_configuration(params)
        print(f"\n  {GREEN}Paramètres appliqués via dynreconf.{RESET}")
    except Exception as e:
        print(f"\n  {RED}dynreconf échoué : {e}{RESET}")
        print(f"  (Simulation non active ? Utilisez --apply uniquement si ROS tourne.)")


# ====================================================================== #
#                    HELPERS PHASE 2 — NAVIGATION                         #
# ====================================================================== #

def _generate_course(ox, oy, size):
    """Génère un parcours carré et le chemin de référence pour le CTE."""
    corners = [
        (ox,        oy),
        (ox + size, oy),
        (ox + size, oy + size),
        (ox,        oy + size),
        (ox,        oy),
    ]
    segments = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        yaw    = math.atan2(y1 - y0, x1 - x0)
        n      = max(1, int(math.ceil(math.hypot(x1-x0, y1-y0) / WP_STEP_M)))
        pts    = []
        for k in range(n):
            t = k / n
            pts.append((x0 + t*(x1-x0), y0 + t*(y1-y0),
                        f"{SEG_LABELS[i]}_p{k}", yaw))
        pts.append((x1, y1, f"{SEG_LABELS[i]}_end", yaw))
        segments.append(pts)

    ref_pts = []
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        n = max(1, int(math.ceil(math.hypot(x1-x0, y1-y0) / 0.3)))
        for k in range(n):
            t = k / n
            ref_pts.append((x0 + t*(x1-x0), y0 + t*(y1-y0)))
        ref_pts.append((x1, y1))

    return segments, ref_pts


def _cte(rx, ry, ref_pts):
    """Distance minimale au chemin de référence."""
    best = float("inf")
    for i in range(len(ref_pts) - 1):
        ax, ay = ref_pts[i]
        bx, by = ref_pts[i+1]
        dx, dy = bx - ax, by - ay
        l2     = dx*dx + dy*dy
        if l2 < 1e-9:
            d = math.hypot(rx - ax, ry - ay)
        else:
            t  = max(0.0, min(1.0, ((rx-ax)*dx + (ry-ay)*dy) / l2))
            d  = math.hypot(rx - ax - t*dx, ry - ay - t*dy)
        if d < best:
            best = d
    return best


class _TebRunner:
    """Exécute les tests de navigation et mesure les métriques."""

    def __init__(self, fixed_params, square_size):
        import rospy
        import actionlib
        from move_base_msgs.msg import MoveBaseAction
        from nav_msgs.msg       import Odometry
        from tf.transformations import euler_from_quaternion
        from std_srvs.srv       import Empty as EmptySrv
        from dynamic_reconfigure.client import Client as DynClient

        self.x = self.y = self.yaw = 0.0
        self.vx = self.vy = self.vth = 0.0
        self.odom_ok = False

        rospy.Subscriber("/lio_sam/mapping/odometry", Odometry,
                         self._odom_cb, queue_size=10)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Connexion move_base...")
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if self.client.wait_for_server(timeout=rospy.Duration(0.5)):
                break
        else:
            rospy.logfatal("move_base non disponible !")
            raise RuntimeError("move_base timeout")

        self.dyn = DynClient("/move_base/TebLocalPlannerROS", timeout=10)
        rospy.loginfo("dynreconf TEB OK")

        self._clear_srv = None
        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=5.0)
            from std_srvs.srv import Empty as EmptySrv
            self._clear_srv = rospy.ServiceProxy(
                "/move_base/clear_costmaps", EmptySrv)
        except Exception:
            pass

        self._fixed = fixed_params
        self._euler = euler_from_quaternion
        self._square = square_size

        # Service respawn Gazebo
        self._respawn_srv = None
        try:
            from gazebo_msgs.srv import SetModelState
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
            self._respawn_srv = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState)
            rospy.loginfo("Respawn Gazebo OK")
        except Exception:
            rospy.logwarn("Service /gazebo/set_model_state indisponible — "
                          "respawn désactivé")

    def _odom_cb(self, msg):
        from tf.transformations import euler_from_quaternion
        self.x  = msg.pose.pose.position.x
        self.y  = msg.pose.pose.position.y
        q       = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.vx  = msg.twist.twist.linear.x
        self.vy  = msg.twist.twist.linear.y
        self.vth = msg.twist.twist.angular.z
        self.odom_ok = True

    def apply_params(self, params):
        import rospy
        try:
            self.dyn.update_configuration(params)
            rospy.sleep(0.5)
            return True
        except Exception as e:
            rospy.logwarn(f"  dynreconf : {e}")
            return False

    def clear(self):
        import rospy
        if self._clear_srv:
            try:
                self._clear_srv()
                rospy.sleep(0.3)
            except Exception:
                pass

    def _respawn_robot(self, x=0.0, y=0.0, z=0.2):
        """Téléporte le robot à la position de spawn via Gazebo."""
        import rospy
        if self._respawn_srv is None:
            rospy.logwarn("  Respawn impossible — service non disponible")
            return
        try:
            from gazebo_msgs.msg import ModelState
            state          = ModelState()
            state.model_name      = "bunker"
            state.reference_frame = "world"
            state.pose.position.x = x
            state.pose.position.y = y
            state.pose.position.z = z
            state.pose.orientation.w = 1.0
            state.twist.linear.x  = 0.0
            state.twist.angular.z = 0.0
            self._respawn_srv(state)
            rospy.sleep(2.0)   # laisse Gazebo stabiliser la physique
            self.clear()
            rospy.loginfo(f"  Robot respawné en ({x}, {y}, {z})")
        except Exception as e:
            rospy.logwarn(f"  Respawn échoué : {e}")

    def _send_goal(self, x, y, yaw):
        import rospy
        from move_base_msgs.msg import MoveBaseGoal
        from tf.transformations import quaternion_from_euler
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

    def run_test(self, segments, ref_pts):
        import rospy
        import actionlib

        # ── Seuils de détection de collision / blocage ───────────────────
        # Tous les timeouts sont en wall-clock (time.time()).
        # À RTF 0.3 : 1 s wall = 0.3 s sim  →  150 s wall ≈ 45–75 s sim
        STUCK_TIMEOUT_S  = 150.0  # ~2.5 min wall — immobile (ni avance ni tourne) → collision
        STUCK_MIN_DIST   = 0.08   # déplacement minimal pour ne pas être "bloqué"
        STUCK_MIN_VTH    = 0.05   # rad/s — en rotation active → pas bloqué
        COLLISION_SKIPS  = 4      # ABORTs consécutifs → collision

        cte_vals       = []
        total_dur      = 0.0
        segs_ok        = 0
        total_dist     = 0.0
        prev_x, prev_y = self.x, self.y
        collision      = False
        tour_start     = time.time()

        for seg_i, seg_wps in enumerate(segments):
            seg_name  = SEG_LABELS[seg_i % len(SEG_LABELS)]
            n_wps     = len(seg_wps)
            wp_idx    = 0
            skips     = 0
            seg_start = time.time()

            # État du détecteur de blocage pour ce segment
            stuck_ref_x, stuck_ref_y = self.x, self.y
            stuck_ref_t = time.time()

            self._send_goal(*seg_wps[wp_idx][:2], seg_wps[wp_idx][3])
            wp_start = time.time()
            rate     = rospy.Rate(10)
            seg_ok   = False

            while not rospy.is_shutdown():
                wx, wy  = seg_wps[wp_idx][0], seg_wps[wp_idx][1]
                dist    = math.hypot(wx - self.x, wy - self.y)
                is_last = (wp_idx == n_wps - 1)
                tol     = CORNER_TOL_M if is_last else WP_ADVANCE_M
                elapsed = time.time() - wp_start

                # Accumulation distance et CTE
                d_step = math.hypot(self.x - prev_x, self.y - prev_y)
                if d_step < 1.0:
                    total_dist += d_step
                prev_x, prev_y = self.x, self.y
                cte_vals.append(_cte(self.x, self.y, ref_pts))

                # ── Détection collision / blocage ─────────────────────────
                moved_since_ref = math.hypot(self.x - stuck_ref_x,
                                             self.y - stuck_ref_y)
                if moved_since_ref > STUCK_MIN_DIST or abs(self.vth) > STUCK_MIN_VTH:
                    # Le robot a bougé ou tourne activement → réinitialiser le chrono
                    if moved_since_ref > STUCK_MIN_DIST:
                        stuck_ref_x, stuck_ref_y = self.x, self.y
                    stuck_ref_t = time.time()
                elif time.time() - stuck_ref_t > STUCK_TIMEOUT_S:
                    rospy.logwarn(
                        f"  ✗ COLLISION/BLOCAGE ({seg_name} wp{wp_idx}) — "
                        f"immobile depuis {STUCK_TIMEOUT_S:.0f}s"
                    )
                    self.client.cancel_goal()
                    collision = True
                    break

                # ── Waypoint atteint ──────────────────────────────────────
                if dist < tol:
                    if is_last:
                        self.client.cancel_goal()
                        seg_ok = True
                        break
                    wp_idx  += 1
                    self._send_goal(*seg_wps[wp_idx][:2], seg_wps[wp_idx][3])
                    wp_start = time.time()
                    # Réinitialiser stuck sur chaque nouveau wp
                    stuck_ref_x, stuck_ref_y = self.x, self.y
                    stuck_ref_t = time.time()
                    continue

                mb_state = self.client.get_state()
                if mb_state == actionlib.GoalStatus.SUCCEEDED:
                    if is_last:
                        seg_ok = True
                        break
                    wp_idx  += 1
                    self._send_goal(*seg_wps[wp_idx][:2], seg_wps[wp_idx][3])
                    wp_start = time.time()
                    stuck_ref_x, stuck_ref_y = self.x, self.y
                    stuck_ref_t = time.time()
                    continue

                if mb_state in (actionlib.GoalStatus.ABORTED,
                                actionlib.GoalStatus.REJECTED):
                    skips += 1
                    self.clear()
                    if skips >= COLLISION_SKIPS:
                        rospy.logwarn(
                            f"  ✗ COLLISION/BLOCAGE ({seg_name}) — "
                            f"{skips} ABORTs consécutifs"
                        )
                        self.client.cancel_goal()
                        collision = True
                        break
                    if not is_last:
                        wp_idx += 1
                    self._send_goal(*seg_wps[wp_idx][:2], seg_wps[wp_idx][3])
                    wp_start = time.time()
                    stuck_ref_x, stuck_ref_y = self.x, self.y
                    stuck_ref_t = time.time()
                    continue

                # Stall prolongé → clear et réessayer le même waypoint.
                # Seuil à 30s wall (≈9s sim à RTF 0.3) pour ne pas interférer
                # avec la planification TEB normale.
                # On NE reset PAS wp_start ici pour que WP_TIMEOUT_S reste actif.
                # On vérifie aussi vth : si le robot tourne, il n'est pas bloqué.
                if (elapsed > 30.0
                        and math.hypot(self.vx, self.vy) < 0.03
                        and abs(self.vth) < STUCK_MIN_VTH):
                    self.clear()
                    self._send_goal(*seg_wps[wp_idx][:2], seg_wps[wp_idx][3])

                if elapsed > WP_TIMEOUT_S:
                    skips += 1
                    self.clear()
                    if skips >= COLLISION_SKIPS:
                        rospy.logwarn(
                            f"  ✗ COLLISION/BLOCAGE ({seg_name}) — timeout ×{skips}"
                        )
                        self.client.cancel_goal()
                        collision = True
                        break
                    if not is_last:
                        wp_idx += 1
                    self._send_goal(*seg_wps[wp_idx][:2], seg_wps[wp_idx][3])
                    wp_start = time.time()
                    stuck_ref_x, stuck_ref_y = self.x, self.y
                    stuck_ref_t = time.time()
                    continue

                rate.sleep()

            seg_dur    = time.time() - seg_start
            total_dur += seg_dur

            if collision:
                break

            if seg_ok:
                segs_ok += 1
                rospy.loginfo(
                    f"  ✓ Segment {seg_i+1}/{len(segments)} ({seg_name}) "
                    f"terminé en {seg_dur:.1f}s"
                )
            else:
                rospy.logwarn(
                    f"  ✗ Segment {seg_i+1}/{len(segments)} ({seg_name}) "
                    f"échoué ({seg_dur:.1f}s)"
                )

            if rospy.is_shutdown():
                break

        # ── Log fin de tour complet ───────────────────────────────────────
        if not collision and segs_ok == len(segments):
            tour_dur = time.time() - tour_start
            cte_avg_ = sum(cte_vals)/len(cte_vals) if cte_vals else 999.0
            rospy.loginfo(
                f"  ★ Tour complet en {tour_dur:.1f}s — "
                f"CTE moy={cte_avg_:.3f}m — "
                f"dist={total_dist:.1f}m"
            )

        return {
            "duration":      round(total_dur, 2),
            "cte_avg":       round(sum(cte_vals)/len(cte_vals), 4) if cte_vals else 999.0,
            "cte_max":       round(max(cte_vals), 4)               if cte_vals else 999.0,
            "success_rate":  round(segs_ok / len(segments), 3)     if segments  else 0.0,
            "distance":      round(total_dist, 2),
            "seg_ok":        segs_ok,
            "seg_total":     len(segments),
            "collision":     collision,
        }

    def return_to_origin(self, ox, oy):
        import rospy
        import actionlib
        self._send_goal(ox, oy, 0.0)
        start = time.time()
        rate  = rospy.Rate(2)
        while not rospy.is_shutdown():
            if math.hypot(ox - self.x, oy - self.y) < CORNER_TOL_M:
                self.client.cancel_goal()
                break
            state = self.client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                break
            if state in (actionlib.GoalStatus.ABORTED,
                         actionlib.GoalStatus.REJECTED):
                self.clear()
                self._send_goal(ox, oy, 0.0)
            if time.time() - start > 120:   # 2 min wall ≈ 36–60 s sim
                self.client.cancel_goal()
                break
            rate.sleep()
        rospy.sleep(2.0)


# ====================================================================== #
#                              MAIN                                        #
# ====================================================================== #

def main():
    parser = argparse.ArgumentParser(
        description="Tuning TEB en 3 phases : generate → run → merge/select",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    grp = parser.add_mutually_exclusive_group(required=True)
    grp.add_argument(
        "--generate", action="store_true",
        help="Phase 1 : générer la grille de paramètres (sans ROS)"
    )
    grp.add_argument(
        "--run", nargs="?", const="__latest__", metavar="GRID.json",
        help="Phase 2 : lancer un batch de tests (simulation active)"
    )
    grp.add_argument(
        "--merge", action="store_true",
        help="Phase 3a : fusionner les batches et sélectionner le meilleur"
    )
    grp.add_argument(
        "--select", nargs="?", const="__latest__", metavar="RESULTS.csv",
        help="Phase 3b : sélectionner sur un CSV existant"
    )
    parser.add_argument(
        "--full", action="store_true",
        help="(--generate) Inclure les paramètres étendus (×4 combos)"
    )
    parser.add_argument(
        "--batch", type=int, default=None, metavar="N",
        help="(--run) Numéro de batch 0-indexé (0, 1, 2). "
             f"Chaque batch = {BATCH_SIZE} combos."
    )
    parser.add_argument(
        "--auto-select", action="store_true",
        help="(--merge) Sélectionne #1 automatiquement sans interaction"
    )
    parser.add_argument(
        "--apply", action="store_true",
        help="(--select / --merge) Appliquer aussi via dynreconf si ROS actif"
    )
    args = parser.parse_args()

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    if args.generate:
        cmd_generate(full=args.full)

    elif args.run is not None:
        grid_file = None if args.run == "__latest__" else args.run
        if grid_file and not os.path.isabs(grid_file):
            grid_file = os.path.join(OUTPUT_DIR, grid_file)
        cmd_run(grid_file, batch=args.batch)

    elif args.merge:
        cmd_merge(auto_select=args.auto_select, apply=args.apply)

    elif args.select is not None:
        results_file = None if args.select == "__latest__" else args.select
        if results_file and not os.path.isabs(results_file):
            results_file = os.path.join(OUTPUT_DIR, results_file)
        cmd_select(results_file, apply=args.apply)


if __name__ == "__main__":
    main()
