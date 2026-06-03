#!/bin/bash
# =============================================================================
# tuning_run.sh  —  Orchestrateur de tuning TEB
# =============================================================================
# Gère automatiquement les 3 batches de 9 combos, avec redémarrage complet de
# LIO-SAM et MoveBase entre chaque batch pour éviter les dérives de mémoire.
#
# Prérequis (lancer AVANT dans des terminaux séparés) :
#   Terminal 1 : roslaunch bunker_slam tuning_gazebo.launch
#   Terminal 2 (optionnel) : roslaunch bunker_slam tuning_rviz.launch
#
# Usage :
#   bash tuning_run.sh                    # utilise la dernière grille générée
#   bash tuning_run.sh teb_grid_XXX.json  # grille spécifique
# =============================================================================

set -eu

# ── Configuration ────────────────────────────────────────────────────────────
WS_SETUP=~/projet_bunker/devel/setup.bash
TUNING_DIR=~/bunker_benchmark/tuning
SCRIPTS_DIR=$(dirname "$(realpath "$0")")
PARAM_TUNER="$SCRIPTS_DIR/param_tuner.py"

N_BATCHES=3
LIOSAM_READY_SLEEP=35    # secondes d'attente après roslaunch liosam
MOVEBASE_READY_SLEEP=12  # secondes d'attente après roslaunch movebase
CLEANUP_SLEEP=12         # secondes après kill pour nettoyage ROS

# ── Couleurs terminal ─────────────────────────────────────────────────────────
BOLD="\033[1m"
GREEN="\033[32m"
YELLOW="\033[33m"
RED="\033[31m"
CYAN="\033[36m"
RESET="\033[0m"

# ── Fonctions utilitaires ─────────────────────────────────────────────────────

log_info()  { echo -e "${BOLD}[tuning_run]${RESET} $*"; }
log_ok()    { echo -e "${GREEN}[tuning_run]${RESET} $*"; }
log_warn()  { echo -e "${YELLOW}[tuning_run]${RESET} $*"; }
log_error() { echo -e "${RED}[tuning_run]${RESET} $*"; }

source_ws() {
    if [ -f "$WS_SETUP" ]; then
        # shellcheck disable=SC1090
        source "$WS_SETUP"
    else
        log_error "Workspace introuvable : $WS_SETUP"
        exit 1
    fi
}

latest_grid() {
    ls -t "$TUNING_DIR"/teb_grid_*.json 2>/dev/null | head -1
}

kill_ros_stack() {
    # Tue LIO-SAM et MoveBase proprement (laisse Gazebo intact)
    local pid_liosam=$1
    local pid_movebase=$2

    log_info "Arrêt LIO-SAM (PID $pid_liosam)..."
    kill "$pid_liosam" 2>/dev/null || true
    wait "$pid_liosam" 2>/dev/null || true

    log_info "Arrêt MoveBase (PID $pid_movebase)..."
    kill "$pid_movebase" 2>/dev/null || true
    wait "$pid_movebase" 2>/dev/null || true

    # Nettoyage des nœuds orphelins dans le master ROS
    # (echo y répond automatiquement à la question de confirmation)
    echo y | rosnode cleanup 2>/dev/null || true
    sleep "$CLEANUP_SLEEP"
    log_ok "Stack ROS arrêtée."
}

respawn_robot_at_origin() {
    # Téléporte le robot au spawn via Gazebo avant de relancer LIO-SAM.
    # Garantit que la position physique Gazebo correspond à la pose initiale
    # que LIO-SAM assume au démarrage (0, 0, 0.2).
    #
    # Double appel : le premier appel stoppe le robot (vitesse résiduelle de
    # la navigation précédente), le second garantit la position finale après
    # que la physique Gazebo s'est stabilisée.
    # La pause de 8 s laisse l'IMU évacuer ses spikes de téléportation avant
    # que LIO-SAM ne démarre (biais IMU estimé proprement dès le premier scan).
    local SET_MODEL="{model_state: {model_name: 'bunker', \
          pose: {position: {x: 0.0, y: 0.0, z: 0.2}, \
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
          twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, \
                  angular: {x: 0.0, y: 0.0, z: 0.0}}, \
          reference_frame: 'world'}}"

    log_info "Respawn du robot à l'origine Gazebo (double appel)..."

    rosservice call /gazebo/set_model_state "$SET_MODEL" > /dev/null 2>&1 \
        && log_info "  1er appel OK — arrêt vitesses résiduelles." \
        || log_warn "  1er appel échoué — /gazebo/set_model_state non disponible."

    sleep 3   # laisse la physique Gazebo absorber le choc

    rosservice call /gazebo/set_model_state "$SET_MODEL" > /dev/null 2>&1 \
        && log_ok "  2e appel OK — robot en (0, 0, 0.2), vitesse nulle." \
        || log_warn "  2e appel échoué."

    # Pause IMU : l'IMU Gazebo génère des spikes d'accélération à chaque
    # téléportation. On attend 8 s pour que les données soient propres
    # avant que LIO-SAM ne consomme les premiers messages.
    log_info "  Attente stabilisation IMU (8 s)..."
    sleep 8
}

wait_for_topic() {
    # Attend qu'un topic publie au moins 1 message.
    # Utilise "rostopic echo -n 1" avec timeout pour éviter le blocage.
    local topic=$1
    local max_wait=${2:-40}
    local t=0
    while [ $t -lt $max_wait ]; do
        if timeout 3 rostopic echo "$topic" -n 1 > /dev/null 2>&1; then
            return 0
        fi
        sleep 3
        t=$((t + 3))
    done
    return 1
}

# ── Vérifications initiales ──────────────────────────────────────────────────

source_ws

# Trouver la grille
if [ $# -ge 1 ]; then
    GRID_FILE="$1"
    if [ ! -f "$GRID_FILE" ]; then
        GRID_FILE="$TUNING_DIR/$1"
    fi
else
    GRID_FILE=$(latest_grid)
fi

if [ -z "$GRID_FILE" ] || [ ! -f "$GRID_FILE" ]; then
    log_error "Aucune grille trouvée. Lancez d'abord :"
    log_error "  python3 $PARAM_TUNER --generate"
    exit 1
fi

# Vérifier que Gazebo tourne
if ! rosnode list 2>/dev/null | grep -q "gazebo"; then
    log_error "Gazebo n'est pas lancé !"
    log_error "Lancez dans un autre terminal :"
    log_error "  roslaunch bunker_slam tuning_gazebo.launch"
    exit 1
fi

# ── Bannière de démarrage ────────────────────────────────────────────────────

echo ""
echo -e "${BOLD}============================================================${RESET}"
echo -e "${BOLD}  TUNING TEB — ORCHESTRATEUR${RESET}"
echo -e "${BOLD}============================================================${RESET}"
log_info "Grille  : $(basename "$GRID_FILE")"
log_info "Batches : $N_BATCHES × 9 combos"
log_info "Logs    : $TUNING_DIR"
echo ""

# Demander confirmation
read -r -p "$(echo -e "${CYAN}Démarrer le tuning ? [o/N]${RESET} ")" CONFIRM
if [[ ! "$CONFIRM" =~ ^[oOyY] ]]; then
    log_warn "Annulé."
    exit 0
fi

# ── Boucle principale : 3 batches ────────────────────────────────────────────

FAILED_BATCHES=()

for BATCH in 0 1 2; do
    BATCH_NUM=$((BATCH + 1))
    echo ""
    echo -e "${BOLD}============================================================${RESET}"
    echo -e "${BOLD}  BATCH $BATCH_NUM / $N_BATCHES  (combos $((BATCH*9+1))–$((BATCH*9+9)))${RESET}"
    echo -e "${BOLD}============================================================${RESET}"

    # ── Lancement LIO-SAM ────────────────────────────────────────────────────
    log_info "Lancement de LIO-SAM..."
    roslaunch bunker_slam tuning_liosam.launch \
        > "$TUNING_DIR/liosam_b${BATCH}.log" 2>&1 &
    LIOSAM_PID=$!
    log_info "LIO-SAM PID = $LIOSAM_PID  (log : liosam_b${BATCH}.log)"

    log_info "Attente initialisation LIO-SAM (${LIOSAM_READY_SLEEP}s)..."
    sleep "$LIOSAM_READY_SLEEP"

    # Vérifier que LIO-SAM publie l'odométrie
    if ! wait_for_topic /lio_sam/mapping/odometry 15; then
        log_warn "Topic /lio_sam/mapping/odometry non détecté — on continue quand même."
    else
        log_ok "LIO-SAM prêt."
    fi

    # ── Lancement MoveBase ───────────────────────────────────────────────────
    log_info "Lancement de MoveBase (TEB)..."
    roslaunch bunker_slam tuning_movebase.launch \
        > "$TUNING_DIR/movebase_b${BATCH}.log" 2>&1 &
    MOVEBASE_PID=$!
    log_info "MoveBase PID = $MOVEBASE_PID  (log : movebase_b${BATCH}.log)"

    log_info "Attente initialisation MoveBase (${MOVEBASE_READY_SLEEP}s)..."
    sleep "$MOVEBASE_READY_SLEEP"
    log_ok "MoveBase prêt."

    # ── Exécution du batch ───────────────────────────────────────────────────
    log_info "Démarrage du batch $BATCH_NUM..."
    set +e
    python3 "$PARAM_TUNER" --run "$GRID_FILE" --batch "$BATCH"
    BATCH_EXIT=$?
    set -e

    if [ $BATCH_EXIT -eq 0 ]; then
        log_ok "Batch $BATCH_NUM terminé avec succès."
    else
        log_warn "Batch $BATCH_NUM terminé avec code $BATCH_EXIT (partiel ou erreur)."
        FAILED_BATCHES+=("$BATCH_NUM")
    fi

    # ── Redémarrage (sauf après le dernier batch) ────────────────────────────
    if [ $BATCH -lt $((N_BATCHES - 1)) ]; then
        echo ""
        log_info "Redémarrage LIO-SAM + MoveBase avant le batch suivant..."
        kill_ros_stack "$LIOSAM_PID" "$MOVEBASE_PID"
        # Replacer le robot au spawn AVANT de relancer LIO-SAM :
        # LIO-SAM s'initialise en supposant robot à (0,0) — le robot doit
        # y être physiquement, sinon la localisation diverge dès le départ.
        respawn_robot_at_origin
    else
        # Dernier batch : on laisse tourner pour la sélection interactive
        log_info "Dernier batch — LIO-SAM + MoveBase laissés actifs."
    fi
done

# ── Résumé et sélection finale ───────────────────────────────────────────────

echo ""
echo -e "${BOLD}============================================================${RESET}"
echo -e "${BOLD}  TUNING TERMINÉ — SÉLECTION DU MEILLEUR PARAMÉTRAGE${RESET}"
echo -e "${BOLD}============================================================${RESET}"

if [ ${#FAILED_BATCHES[@]} -gt 0 ]; then
    log_warn "Batches avec erreurs : ${FAILED_BATCHES[*]}"
    log_warn "Les résultats partiels seront quand même inclus dans la fusion."
fi

log_info "Fusion des résultats et sélection automatique..."
python3 "$PARAM_TUNER" --merge --auto-select

echo ""
log_ok "Tous les résultats sont dans : $TUNING_DIR"
log_ok "Pour revoir le classement interactif :"
echo -e "  ${CYAN}python3 $PARAM_TUNER --select${RESET}"
echo ""
