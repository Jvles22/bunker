#!/usr/bin/env python3
"""analyze_traverse.py — Analyse d'une run square_ramp_traverse.py

Usage :
    python3 analyze_traverse.py <chemin_vers_fichier.csv>

Génère 4 figures PNG dans le même dossier que le CSV, et les affiche.
"""

import sys
import os
import math

import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
# from matplotlib.collections import LineCollection
# from matplotlib.colors import BoundaryNorm, ListedColormap

# ── Couleurs par segment ──────────────────────────────────────────────────────
SEG_COLORS = {
    "S1_Sud":    "#2196F3",   # bleu
    "S2_Est":    "#4CAF50",   # vert
    "S3_Nord":   "#FF9800",   # orange
    "S4_Ouest":  "#E91E63",   # rose
}
UNKNOWN_COLOR = "#9E9E9E"

# ── Seuils de pente (cohérents avec CLAUDE.md) ────────────────────────────────
SLOPE_THRESHOLDS = [8, 10, 20, 30, 35]
SLOPE_LABELS     = ["<8°", "8-10°", "10-20°", "20-30°", "30-35°", ">35°"]
SLOPE_COLORS_MAP = ["#4CAF50", "#CDDC39", "#FF9800", "#FF5722", "#B71C1C", "#000000"]

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Temps écoulé depuis le début de la run (secondes ROS)
    df["elapsed_s"] = df["timestamp_ros"] - df["timestamp_ros"].iloc[0]
    # Distance incrémentale parcourue
    dx = df["robot_x"].diff().fillna(0)
    dy = df["robot_y"].diff().fillna(0)
    df["dist_step"] = np.sqrt(dx**2 + dy**2)
    df["dist_cum"]  = df["dist_step"].cumsum()
    # Vitesse effective estimée (norme du vecteur vel)
    df["speed_eff"] = np.sqrt(df["vel_x"]**2 + df["vel_y"]**2)
    # Segment ordinal pour le fond de couleur
    seg_order = df["segment"].unique().tolist()
    df["seg_idx"] = df["segment"].map({s: i for i, s in enumerate(seg_order)})
    return df


def seg_color(seg: str) -> str:
    return SEG_COLORS.get(seg, UNKNOWN_COLOR)


def make_segment_spans(ax, df: pd.DataFrame, x_col: str = "elapsed_s",
                       alpha: float = 0.08):
    """Trace des bandes de fond colorées par segment."""
    segments = df["segment"].unique()
    for seg in segments:
        mask = df["segment"] == seg
        x_vals = df.loc[mask, x_col]
        if x_vals.empty:
            continue
        ax.axvspan(x_vals.iloc[0], x_vals.iloc[-1],
                   color=seg_color(seg), alpha=alpha, label=seg)


def segment_legend(ax):
    patches = [mpatches.Patch(color=c, label=s) for s, c in SEG_COLORS.items()]
    ax.legend(handles=patches, fontsize=8, loc="best")


def save_fig(fig, csv_path: str, suffix: str):
    base = os.path.splitext(csv_path)[0]
    out  = f"{base}_{suffix}.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"  → {out}")


def ramp_rows(df: pd.DataFrame) -> pd.DataFrame:
    return df[df["waypoint_name"].str.endswith("_ramp", na=False)]


# ─────────────────────────────────────────────────────────────────────────────
# Figure 1 — Vue d'ensemble
# ─────────────────────────────────────────────────────────────────────────────

def fig_overview(df: pd.DataFrame, csv_path: str):
    fig = plt.figure(figsize=(16, 10))
    gs  = fig.add_gridspec(2, 2, hspace=0.38, wspace=0.30)
    fig.suptitle(f"Vue d'ensemble — {os.path.basename(csv_path)}", fontsize=13)

    axes = [
        fig.add_subplot(gs[0, 0]),   # 1a trajectoire
        fig.add_subplot(gs[0, 1]),   # 1b pente
        fig.add_subplot(gs[1, :]),   # 1c vitesse — pleine largeur
    ]

    # ── 1a. Trajectoire 2D ──────────────────────────────────────────────
    ax = axes[0]
    for seg, grp in df.groupby("segment", sort=False):
        ax.plot(grp["robot_x"].values, grp["robot_y"].values,
                color=seg_color(seg), linewidth=1.5, label=seg)
    # Marqueurs de waypoints ramp
    # # ramps = ramp_rows(df).drop_duplicates("waypoint_name")
    # ax.scatter(ramps["robot_x"].values, ramps["robot_y"].values,
    #            marker="^", s=120, color="red", zorder=5, label="Rampe")
    # # Slip events
    # slips = df[df["slip_detected"] == 1]
    # if not slips.empty:
    #     ax.scatter(slips["robot_x"].values, slips["robot_y"].values,
    #                marker="x", s=60, color="black", zorder=6, label="Patinage")
    # Départ / arrivée
    ax.scatter(df["robot_x"].iloc[0],  df["robot_y"].iloc[0],
               marker="o", s=100, color="lime",  zorder=7, label="Départ")
    ax.scatter(df["robot_x"].iloc[-1], df["robot_y"].iloc[-1],
               marker="s", s=100, color="purple", zorder=7, label="Arrivée")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Trajectoire 2D")
    ax.set_aspect("equal"); ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc="best")

    # ── 1b. Pente au fil du temps ────────────────────────────────────────
    ax = axes[1]
    make_segment_spans(ax, df)
    ax.plot(df["elapsed_s"].values, df["slope_deg"].values, color="#FF5722",
            linewidth=0.8, alpha=0.9, label="slope_deg")
    ax.axhline(35, color="black", linestyle="--", linewidth=1, label="Seuil létal 35°")
    ax.axhline(30, color="red",   linestyle=":",  linewidth=1, label="Seuil 30°")
    # Marqueurs ramp passages
    for _, r in ramp_rows(df).drop_duplicates("waypoint_name").iterrows():
        ax.axvline(r["elapsed_s"], color="red", linewidth=1.2, linestyle="-.")
    ax.set_xlabel("Temps écoulé (s)"); ax.set_ylabel("Pente (°)")
    ax.set_title("Pente mesurée (slope_deg)")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)

    # ── 1c. Vitesse commandée et effective ──────────────────────────────
    ax = axes[2]
    make_segment_spans(ax, df)
    ax.step(df["elapsed_s"].values, df["speed_setting"].values, color="#2196F3",
            linewidth=1.5, label="speed_setting (cmd)", where="post")
    ax.plot(df["elapsed_s"].values, df["speed_eff"].values, color="#FF9800",
            linewidth=0.8, alpha=0.85, label="vitesse effective (odom)")
    ax.set_xlabel("Temps écoulé (s)"); ax.set_ylabel("Vitesse (m/s)")
    ax.set_title("Vitesse commandée vs effective")
    ax.legend(fontsize=7); ax.grid(True, alpha=0.3)
    segment_legend(ax)

    save_fig(fig, csv_path, "fig1_overview")
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# Figure 2 — Données IMU détaillées
# ─────────────────────────────────────────────────────────────────────────────

def fig_imu(df: pd.DataFrame, csv_path: str):
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    fig.suptitle(f"Données IMU — {os.path.basename(csv_path)}", fontsize=13)

    titles  = ["Pitch (°)", "Roll (°)", "Slope = max(|pitch|,|roll|) (°)"]
    cols    = ["pitch_deg", "roll_deg", "slope_deg"]
    colors  = ["#3F51B5", "#009688", "#FF5722"]
    hrefs   = [None, None, [8, 20, 30, 35]]

    for ax, title, col, color, hrefs_list in zip(axes, titles, cols, colors, hrefs):
        make_segment_spans(ax, df, alpha=0.07)
        ax.plot(df["elapsed_s"].values, df[col].values, color=color,
                linewidth=0.8, alpha=0.9)
        ax.axhline(0, color="gray", linewidth=0.5)
        if hrefs_list:
            hcolors = ["#CDDC39", "#FF9800", "#FF5722", "#B71C1C"]
            hlabels = ["8° (min_slope)", "20°", "30°", "35° (létal)"]
            for h, hc, hl in zip(hrefs_list, hcolors, hlabels):
                ax.axhline(h,  color=hc, linestyle="--", linewidth=1.0, label=hl)
                ax.axhline(-h, color=hc, linestyle="--", linewidth=1.0)
            ax.legend(fontsize=7, loc="upper right")
        # Slip markers
        slips = df[df["slip_detected"] == 1]
        if not slips.empty:
            ax.scatter(slips["elapsed_s"].values, slips[col].values,
                       marker="x", s=40, color="black", zorder=5, label="Patinage")
        ax.set_ylabel(title)
        ax.grid(True, alpha=0.3)
        ax.set_title(title)

    axes[-1].set_xlabel("Temps écoulé (s)")
    segment_legend(axes[0])
    plt.tight_layout()
    save_fig(fig, csv_path, "fig2_imu")
    return fig


# ─────────────────────────────────────────────────────────────────────────────
# Figure 3 — Statistiques par segment
# ─────────────────────────────────────────────────────────────────────────────

def fig_segments(df: pd.DataFrame, csv_path: str):
    segments = list(df["segment"].unique())
    n = len(segments)

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(f"Statistiques par segment — {os.path.basename(csv_path)}", fontsize=13)
    bar_colors = [seg_color(s) for s in segments]

    # ── 3a. Pente max et moyenne par segment ────────────────────────────
    ax = axes[0, 0]
    means = [df[df["segment"] == s]["slope_deg"].mean() for s in segments]
    maxs  = [df[df["segment"] == s]["slope_deg"].max()  for s in segments]
    x = np.arange(n)
    w = 0.35
    ax.bar(x - w/2, means, w, color=bar_colors, alpha=0.7, label="Moy")
    ax.bar(x + w/2, maxs,  w, color=bar_colors, alpha=1.0, label="Max", edgecolor="black")
    ax.axhline(35, color="red", linestyle="--", linewidth=1, label="Létal 35°")
    ax.set_xticks(x); ax.set_xticklabels(segments, rotation=15)
    ax.set_ylabel("Pente (°)"); ax.set_title("Pente moy. / max par segment")
    ax.legend(fontsize=8); ax.grid(True, axis="y", alpha=0.3)

    # ── 3b. Durée par segment (secondes) ────────────────────────────────
    ax = axes[0, 1]
    durations = []
    for s in segments:
        t = df[df["segment"] == s]["elapsed_s"]
        durations.append(t.iloc[-1] - t.iloc[0] if len(t) > 1 else 0)
    ax.bar(segments, durations, color=bar_colors, edgecolor="black", alpha=0.85)
    ax.set_ylabel("Durée (s)"); ax.set_title("Temps par segment")
    ax.set_xticklabels(segments, rotation=15)
    ax.grid(True, axis="y", alpha=0.3)
    for i, d in enumerate(durations):
        ax.text(i, d + 0.5, f"{d:.0f}s", ha="center", fontsize=9)

    # ── 3c. Distance parcourue par segment ──────────────────────────────
    ax = axes[1, 0]
    distances = [df[df["segment"] == s]["dist_step"].sum() for s in segments]
    ax.bar(segments, distances, color=bar_colors, edgecolor="black", alpha=0.85)
    ax.set_ylabel("Distance (m)"); ax.set_title("Distance parcourue par segment")
    ax.set_xticklabels(segments, rotation=15)
    ax.grid(True, axis="y", alpha=0.3)
    for i, d in enumerate(distances):
        ax.text(i, d + 0.2, f"{d:.1f}m", ha="center", fontsize=9)


# ─────────────────────────────────────────────────────────────────────────────
# Figure 4 — Corrélations et distributions
# ─────────────────────────────────────────────────────────────────────────────


    # ── 4b. Trajectoire 2D colorée par pente ────────────────────────────
    # ax = ax_traj_c
    # # Utilise LineCollection pour colorier segment par segment selon slope_deg
    # slope_vals = df["slope_deg"].values
    # x = df["robot_x"].values
    # y = df["robot_y"].values
    # points = np.array([x, y]).T.reshape(-1, 1, 2)
    # segments_arr = np.concatenate([points[:-1], points[1:]], axis=1)
    # bounds = [0, 8, 10, 20, 30, 35, 100]
    # cmap = ListedColormap(SLOPE_COLORS_MAP)
    # norm = BoundaryNorm(bounds, cmap.N)
    # lc = LineCollection(segments_arr, cmap=cmap, norm=norm, linewidth=1.5)
    # lc.set_array((slope_vals[:-1] + slope_vals[1:]) / 2)
    # ax.add_collection(lc)
    # ax.autoscale()
    # cbar = plt.colorbar(lc, ax=ax, boundaries=bounds, ticks=[0, 8, 10, 20, 30, 35])
    # cbar.set_label("Pente (°)", fontsize=9)
    # ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    # ax.set_title("Trajectoire colorée par pente")
    # ax.set_aspect("equal"); ax.grid(True, alpha=0.3)


# ─────────────────────────────────────────────────────────────────────────────
# Résumé texte
# ─────────────────────────────────────────────────────────────────────────────

def print_summary(df: pd.DataFrame, csv_path: str):
    total_s   = df["elapsed_s"].iloc[-1]
    total_m   = df["dist_cum"].iloc[-1]
    slip_tot  = df["slip_detected"].sum()
    max_slope = df["slope_deg"].max()
    planner   = df["planner"].iloc[0] if "planner" in df.columns else "?"

    print("\n" + "═" * 60)
    print(f"  RÉSUMÉ — {os.path.basename(csv_path)}")
    print("═" * 60)
    print(f"  Planner         : {planner}")
    print(f"  Durée totale    : {total_s:.1f} s  ({total_s/60:.1f} min)")
    print(f"  Distance totale : {total_m:.1f} m")
    print(f"  Pente max       : {max_slope:.1f}°")
    print(f"  Événements slip : {int(slip_tot)}")
    print()
    for seg in df["segment"].unique():
        g = df[df["segment"] == seg]
        dur  = g["elapsed_s"].iloc[-1] - g["elapsed_s"].iloc[0]
        dist = g["dist_step"].sum()
        smax = g["slope_deg"].max()
        smoy = g["slope_deg"].mean()
        slip = int(g["slip_detected"].sum())
        print(f"  [{seg}]  {dur:5.0f}s  {dist:5.1f}m  "
              f"slope moy={smoy:.1f}° max={smax:.1f}°  slip={slip}")
    print("═" * 60 + "\n")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) != 2:
        print(f"Usage : python3 {os.path.basename(sys.argv[0])} <fichier.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    if not os.path.isfile(csv_path):
        print(f"Erreur : fichier introuvable — {csv_path}")
        sys.exit(1)

    print(f"\nChargement de {csv_path} …")
    df = load_csv(csv_path)
    print(f"  {len(df)} lignes, {df['segment'].nunique()} segments, "
          f"colonnes : {list(df.columns)}")

    print_summary(df, csv_path)

    print("Génération des figures …")
    fig1 = fig_overview(df, csv_path)
    fig2 = fig_imu(df, csv_path)
    fig3 = fig_segments(df, csv_path)
    # fig4 = fig_correlations(df, csv_path)

    plt.show()


if __name__ == "__main__":
    main()
