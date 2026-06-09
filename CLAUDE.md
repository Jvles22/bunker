# Résumé de session — projet_bunker
**Date :** 2026-04-12

---

## 1. Correction de slopes_map.world

**Problème :** La map `slopes_map.world` s'ouvrait blanche dans Gazebo 11 classic.

**Cause identifiée :** Le modèle `pallet_box_mobile` contenait une URI
`https://fuel.ignitionrobotics.org/...` — Gazebo classic tentait de télécharger
le mesh depuis internet, bloquant indéfiniment le chargement. Un plugin
Ignition-spécifique (`ignition-gazebo-pose-publisher-system`) était également
présent et incompatible.

**Corrections apportées (`slopes_map.world`) :**
- URI `https://fuel.ignitionrobotics.org/...` → `model://pallet_box/meshes/boxes.dae`
- URI `model://pallet_box_mobile/meshes/boxes.dae` → `model://pallet_box/meshes/boxes.dae`
- Suppression du bloc `<plugin name='ignition::gazebo::systems::PosePublisher' ...>`

**Résultat attendu :** `gazebo ~/projet_bunker/src/bunker_gazebo/worlds/slopes_map.world`
lance la map correctement sans blocage.

---

## 2. Enrichissement de la map (features LIO-SAM)

**Problème :** Le robot manquait de features (amers 3D) dans les zones de coin,
ce qui faisait diverger LIO-SAM.

**14 nouveaux objets ajoutés** (arbres + barrel), tous à ≥ 3 m des chemins du robot :

| Objet | Position | Zone |   
|---|---|---|
| Pine Tree_11 | (3, 3) | Cluster spawn |
| Oak tree_6 | (6, 6) | Cluster spawn |
| Pine Tree_12 | (4, 21.5) | Cluster coin NW (0,25) |
| Oak tree_7 | (7, 22) | Cluster coin NW (0,25) |
| Pine Tree_13 | (10, 22) | Bordure nord intérieure |
| Oak tree_8 | (15, 22) | Bordure nord intérieure |
| Pine Tree_14 | (17, 22) | Cluster coin NE (25,25) |
| Oak tree_9 | (19, 22) | Cluster coin NE (25,25) |
| Oak tree_10 | (22, 8) | Bordure est intérieure |
| Pine Tree_15 | (22, 11) | Bordure est intérieure |
| Pine Tree_16 | (22, 4) | Cluster coin SE (25,0) |
| Oak tree_11 | (20, 3) | Cluster coin SE (25,0) |
| nist_fiducial_barrel_0 | (19, 5) | Cluster coin SE (25,0) |
| Pine Tree_17 | (17, 4) | Bordure sud intérieure |

**Contraintes respectées :**
- Aucun objet à moins de 3 m des arêtes du carré (x=0, x=25, y=0, y=25)
- Aucun objet à moins de 3 m des coins (0,0), (0,25), (25,25), (25,0)
- Aucun objet à moins de 2 m des objets existants
- Positions des objets existants et des rampes non modifiées

---

## 3. Script square_ramp_traverse.py (v2.0)

**Fichier :** `src/bunker_slam/scripts/square_ramp_traverse.py`

**Objectif :** Faire parcourir au robot un carré 25×25 m en passant
obligatoirement sur chacune des trois rampes.

**Trajectoire :** (0,0) → (0,25) → (25,25) → (25,0) → (0,0)

**Rampes traversées :**
- `rampe_30d` à (0, 12.5) — côté Ouest
- `rampe_20d` à (12.5, 25) — côté Nord
- `rampe_10d` à (25, 12.5) — côté Est

**Mécanique de passage forcé sur rampe :** chaque segment est découpé en
deux sous-segments `début → rampe → fin`. Move_base reçoit un waypoint
explicite sur la rampe, l'obligeant à planifier un chemin passant par là.

**Fonctionnalités v2.0 :**

| Fonctionnalité | Paramètre clé | Comportement |
|---|---|---|
| Lecture IMU | `IMU_TOPIC = "/imu/data"` | Pitch, roll, slope_deg enregistrés dans le CSV |
| Arrêt d'urgence pente | `MAX_SLOPE_DEG = 30.0` (max 35°) | Stop immédiat si pente dépassée, flag dans CSV |
| Courbe vitesse/pente | `SLOPE_SPEED_TABLE` | 0–10° : 0.5 m/s / 10–20° : 0.3 m/s / 20–30° : 0.2 m/s / 30–35° : 0.1 m/s |
| Détection patinage | `SLIP_RATIO_THRESHOLD = 0.4` | Alerte si déplacement réel < 40% du déplacement estimé |

**CSV produit (`~/bunker_square_ramp/`) :**
`timestamp_ros`, `timestamp_real`, `segment`, `waypoint_name`,
`target_x/y`, `robot_x/y/yaw`, `vel_x/y/theta`, `distance_to_goal`,
`speed_setting`, `pitch_deg`, `roll_deg`, `slope_deg`, `slip_detected`, `planner`

**Lancement :**
```bash
source ~/projet_bunker/devel/setup.bash
rosrun bunker_slam square_ramp_traverse.py
```

---

## 4. Launch file slopes_navigation.launch

**Fichier :** `src/bunker_slam/launch/slopes_navigation.launch`

Lance Gazebo avec `slopes_map.world` et spawn le robot bunker à (0, 0, 0.2).
Même structure que `navigation.launch`.

**Séquence de lancement complète :**
```bash
# Terminal 1 — Gazebo + robot
source /home/projet_bunker/devel/setup.bash
roslaunch bunker_slam slopes_navigation.launch

# Terminal 2 — SLAM (LIO-SAM)
source /home/projet_bunker/devel/setup.bash
roslaunch bunker_slam slam.launch

# Terminal 3 — Navigation (move_base)
source /home/projet_bunker/devel/setup.bash
roslaunch bunker_slam move_base.launch

# Terminal 4 — RViz
source /home/projet_bunker/devel/setup.bash
roslaunch bunker_slam rviz.launch

# Terminal 5 — Traversée carré avec rampes
source /home/projet_bunker/devel/setup.bash
rosrun bunker_slam square_ramp_traverse.py
```

---

## 5. Corrections et bugs résolus (square_ramp_traverse.py)

| Bug | Cause | Correction |
|---|---|---|
| `move_base timeout` immédiat | `wait_for_server(rospy.Duration(30))` utilise le temps simulé ; au démarrage, sim_time=0 puis saute à ~1900s → timeout déclenché instantanément | Boucle en wall time (`time.time()`) avec `wait_for_server(rospy.Duration(0.5))` par itération |
| Vitesse à plat ignorée | `_get_speed_for_slope` retournait toujours 0.5 depuis la table, ignorant la constante `SPEED` | Retourne `SPEED` si `slope < premier_seuil` |
| Skip sur rampes raides | `WP_TIMEOUT_S = 30s` = temps de trajet exact à 0.1 m/s → expire avant arrivée | Augmenté à 90s |
| Timeout odométrie fragile | `rospy.Time.now()` avec `use_sim_time=true` peut sauter | Deadline en `time.time()` (wall clock) |
| `RuntimeError` non capturé | Seul `ROSInterruptException` était attrapé dans `__main__` | Ajout `except RuntimeError` avec `sys.exit(1)` |

**Trajectoire corrigée (v2.1) :** `(0,0) → (25,0) → (25,25) → (0,25) → (0,0)`  
La rampe la plus difficile (30°) est désormais passée en dernier.

---

## 6. Analyse des approches pour la navigation en terrain pentu

### Contexte et problème

Lors du test de `square_ramp_traverse.py`, le robot a correctement négocié les
rampes à 10° et 20°, mais a refusé d'aborder la rampe à 30°. Cause : le LiDAR
Velodyne détectait la surface inclinée de la rampe dans la fenêtre de hauteur
d'obstacle `[0.15m, 0.5m]` et la marquait comme obstacle létal dans la costmap.
Le planificateur DWA ne trouvait alors aucun chemin — `DWA planner failed to produce path`.

### Vision à long terme

L'objectif est que le robot puisse naviguer de manière autonome sur n'importe
quelle map avec du dénivelé, en **estimant la complexité de chaque itinéraire**
et en choisissant le chemin de moindre effort — passer par une pente si c'est
plus court, la contourner si le coût est trop élevé.

### Options étudiées

#### Option 1 — `min_obstacle_height` dynamique via dynreconf
**Principe :** Avant chaque waypoint de rampe, appeler `dynamic_reconfigure` pour
monter temporairement `min_obstacle_height` à 0.4 m sur les deux costmaps, puis
le restaurer à 0.15 m après passage.

| ✅ Avantages | ❌ Inconvénients |
|---|---|
| Simple à implémenter (4 appels dynreconf) | Aveugle le robot pendant ~30s sur les rampes |
| Aucun nouveau package | Ne généralise pas à des maps inconnues |
| | Incompatible avec la vision « évaluation de complexité » |

**Verdict : écarté.** C'est un contournement, pas une solution intelligente.

#### Option 2 — `VoxelLayer` + couche de coût de pente
**Principe :** Remplacer `ObstacleLayer` (2D) par `VoxelLayer` (3D). Le paramètre
`mark_threshold` permet de n'enregistrer comme obstacle que les colonnes ayant
plusieurs voxels occupés — une surface inclinée crée 1 voxel par couche, un mur
en crée plusieurs. Ajouter ensuite une couche custom qui encode l'angle de pente
comme coût variable.

| ✅ Avantages | ❌ Inconvénients |
|---|---|
| Disponible nativement dans costmap_2d | Ne donne pas au planificateur une vision de « difficulté » |
| Résout le faux positif sur les rampes | Mark_threshold à tuner empiriquement |
| Détection d'obstacles conservée | Pas généralisable seul à des maps inconnues |

**Verdict : bon intermédiaire,** mais insuffisant pour la vision long terme.

#### Option 3 — Carte d'élévation + couche de traversabilité ✅ RETENU
**Principe :** Un nœud dédié (`bunker_elevation_map_node`) accumule les hauteurs
du Velodyne cellule par cellule pour construire une carte d'élévation
(`grid_map_msgs/GridMap`). Un plugin costmap (`TraversabilityLayer`) calcule la
pente par différences finies et encode chaque cellule avec un **coût proportionnel
à l'angle** : pente faible = coût faible, pente forte = coût élevé, pente > 35° =
obstacle létal. Le planificateur global (navfn/Dijkstra) choisit alors
naturellement le chemin de moindre coût — il peut préférer une pente douce à un
grand détour.

| ✅ Avantages | ❌ Inconvénients |
|---|---|
| Compatible avec la vision « évaluation de complexité » | Complexité élevée (C++ plugin + nœud PCL) |
| Fonctionne sur n'importe quelle map inconnue | Nécessite grid_map (apt) |
| Distinction traversable / obstacle basée sur la physique | Latence : la carte s'enrichit au fur et à mesure |
| Coûts continus → planificateur optimise naturellement | Zones non visitées = pas de données de traversabilité |
| Base pour ajouter rugosité, franchissement de seuil, etc. | |

**Verdict : retenu.** C'est le fondement exact de la navigation en terrain complexe.

#### Option 4 — Nœud de filtrage PCL (normales de surface)
**Principe :** Intercaler un nœud entre le Velodyne et la costmap qui estime la
normale de chaque point, retire les points dont la normale est trop inclinée
(surface de rampe) et les remplace par des points virtuels avec coût adapté.

| ✅ Avantages | ❌ Inconvénients |
|---|---|
| Solution la plus physiquement rigoureuse | Latence d'estimation des normales (PCL NormalEstimation) |
| Géneralise à toute géométrie | Difficile à intégrer proprement avec costmap_2d |

**Verdict : approche complémentaire,** potentiellement utile à terme pour la rugosité.

### Pourquoi l'Option 3 est la bonne fondation

La vision finale est un planificateur qui **optimise un critère mixte** :
distance + coût de traversabilité. Cela nécessite que le costmap encode la
difficulté du terrain comme un coût continu (non binaire). L'Option 3 est la
seule qui fournit ce coût continu de manière générique, sans connaissance a priori
de la carte. Chaque heure investie dans cette architecture sert directement
l'objectif final.

---

## 7. Package `bunker_traversability`

**Chemin :** `src/bunker_traversability/`

### Architecture (après Piste 1)

```
Velodyne (/velodyne_points)
    ↓
adaptive_obstacle_filter_node  ← robot_z via TF (base_footprint→map)
    |  Garde uniquement les points dans [robot_z+0.05m, robot_z+0.55m]
    ↓  /velodyne_points_filtered (repère map)
    ├─→ ObstacleLayer (costmap_2d)
    |      marque les obstacles statiques (arbres, murs)
    |
    └─→ bunker_elevation_map_node (/velodyne_points direct)
           ↓  /bunker/elevation_map (GridMap, layer "elevation")
        TraversabilityLayer (costmap_2d plugin)
           ↓  coût par cellule selon pente
move_base costmap (obstacle → traversability → inflation)
    ↓
DWA Planner → commandes de vitesse
```

### Composants

**`adaptive_obstacle_filter_node.py`** (nœud Python — Piste 1)
- Souscrit à `/velodyne_points`, transforme dans le repère `map` via TF2
- Récupère `robot_z` = hauteur de `base_footprint` dans `map`
- Filtre : conserve uniquement les points où `robot_z + 0.05 ≤ z ≤ robot_z + 0.55`
- Publie sur `/velodyne_points_filtered` (repère `map`)
- **Effet :** quand le robot monte (z > 0), la surface de la rampe en-dessous
  sort de la fenêtre → plus de faux obstacles sous le robot en dénivelé.
  La canopée (z >> robot_z + 0.55) est également exclue.

**`bunker_elevation_map_node`** (nœud C++)
- Souscrit à `/velodyne_points` (nuage brut, non filtré — pour avoir l'élévation réelle)
- Transforme en frame `map` via TF2
- Accumule une moyenne courante de hauteur par cellule (0.15 m/cell)
- Carte fixe 70×70 m centrée à (12.5, 12.5), couvrant le carré + features extérieures
- Publie `/bunker/elevation_map` à 2 Hz
- Paramètre `max_height: 1.2 m` — exclut canopée (>1.5m), capture troncs + rampe 30° (~1.15m)

**`TraversabilityLayer`** (plugin costmap_2d, C++)
- Hérite de `costmap_2d::CostmapLayer` (fournit `updateWithOverwrite`)
- Table de coûts pente → coût costmap (`min_slope_deg: 5.0°`, `lethal_slope_deg: 25.0°`) :

| Pente | Coût | Comportement planificateur |
|---|---|---|
| < 5° | NO_INFORMATION (transparent) | Conserve les marquages d'obstacles plats (arbres) |
| 5°–10° | 10 | Légère pénalité |
| 10°–20° | 50 | Pénalité modérée, détour préférable |
| 20°–25° | 90 | Forte pénalité, juste sous le seuil létal |
| ≥ 25° | 254 (létal) | Interdit |

- Ordre dans la costmap : `obstacle_layer` → **`traversability_layer`** → `inflation_layer`
- Les cellules de rampe (pente ≥ 8°) sont **corrigées** : le faux positif
  de l'obstacle_layer est remplacé par le coût de traversabilité réel.
- Les obstacles plats (arbres) : pente ≈ 0° → transparent → marquage létal conservé.

### Séquence de lancement

```bash
# Terminal 1 — Gazebo + robot
roslaunch bunker_slam slopes_navigation.launch

# Terminal 2 — SLAM (LIO-SAM)
roslaunch bunker_slam slam.launch

# Terminal 3 — Navigation + filtre adaptatif + carte d'élévation (inclus automatiquement)
roslaunch bunker_slam move_base.launch

# Terminal 4 — RViz
roslaunch bunker_slam rviz.launch

# Terminal 5 — Traversée carré avec rampes
rosrun bunker_slam square_ramp_traverse.py
```

> `move_base.launch` inclut `traversability.launch` qui démarre :
> - `adaptive_obstacle_filter_node.py`
> - `bunker_elevation_map_node`

---

## 8. Corrections faux obstacles — historique des pistes

### Problème initial : canopée détectée comme obstacle

**Cause :** `max_height: 3.0m` dans `elevation_map.yaml` capturait les retours
de canopée → gradient de hauteur élevé à la lisière des arbres → cellules létales
sur le chemin du robot.

**Correction :** `max_height: 1.2m` — couvre la rampe 30° (~1.15m) et les troncs
(0–1m), exclut la canopée (démarre à ~1.5m).

### Problème persistant : faux obstacles sur la rampe 30° quand le robot est en hauteur

**Cause :** Quand le robot monte la rampe à ~1m de hauteur, le LiDAR Velodyne
(monté sur le robot) voit la surface de la rampe dans sa fenêtre `[min, max]`.
Les points de la surface sont à ~0m dans le repère world, donc bien dans la
fenêtre d'obstacle → marqués comme obstacles létaux autour du robot.

**Deux pistes retenues :**

#### Piste 1 — Filtre adaptatif de hauteur (implémentée) ✅
**Fichier :** `src/bunker_traversability/scripts/adaptive_obstacle_filter_node.py`

Filtre les points LiDAR en fonction de la hauteur courante du robot :
- Fenêtre : `[robot_z + 0.05m, robot_z + 0.55m]`
- Quand robot_z = 1m, seuls les points entre 1.05m et 1.55m sont gardés
  → la surface de la rampe (z≈0) est exclue

**Impact sur costmap_common_params.yaml :**
```yaml
velodyne:
  sensor_frame: map                   # nuage déjà transformé dans map
  topic: /velodyne_points_filtered    # ← nuage filtré
  min_obstacle_height: -10.0          # fenêtre gérée par le nœud Python
  max_obstacle_height:  10.0
```

#### Piste 2 — Relâchement de min_slope_deg (implémentée) ✅
`min_slope_deg: 5.0°` → `8.0°` dans `costmap_common_params.yaml`.

Évite que des micro-ondulations du terrain (~5–7°) déclenchent inutilement
la TraversabilityLayer et écrasent les marquages d'obstacles plats.

---

## 9. To-do list — Navigation traversable

### ✅ Fait
- [x] Package `bunker_traversability` créé et compilé
- [x] `bunker_elevation_map_node` : accumulation des hauteurs Velodyne → GridMap
- [x] `TraversabilityLayer` : plugin costmap_2d avec table de coûts pente
- [x] Intégration dans `costmap_common_params.yaml`, `global_costmap_params.yaml`, `local_costmap_params.yaml`
- [x] `move_base.launch` inclut automatiquement `traversability.launch`
- [x] Correction canopée : `max_height 3.0 → 1.2m`
- [x] Piste 1 : `adaptive_obstacle_filter_node.py` — fenêtre de hauteur relative au robot
- [x] Piste 2 : `min_slope_deg 5° → 8°`

### 🔲 À valider en simulation
- [ ] **Vérifier la carte d'élévation dans RViz** : ajouter un display `GridMap`
  (plugin `grid_map_rviz_plugin`), topic `/bunker/elevation_map`, layer `elevation`.
  Vérifier que les rampes apparaissent avec une hauteur cohérente après ~30s de navigation.
- [ ] **Vérifier la costmap dans RViz** : activer `Map` → `/move_base/global_costmap/costmap`.
  Les rampes à 30° doivent apparaître en coût élevé (rouge/orange) mais non létal.
- [ ] **Tester le passage de la rampe 30°** avec `square_ramp_traverse.py`.
  Le robot doit maintenant choisir de passer dessus sans détecter de faux obstacles.
- [ ] **Valider la non-régression** : les arbres doivent rester des obstacles létaux
  (le robot ne doit pas essayer de passer à travers).
- [ ] **Vérifier le topic filtré** : `rostopic hz /velodyne_points_filtered` doit
  afficher ~10 Hz ; `rostopic echo /velodyne_points_filtered/header` doit montrer
  `frame_id: map`.

### 🔲 Améliorations futures (vision long terme)
- [ ] **Ajouter une couche de rugosité** (`roughness`) dans la carte d'élévation :
  variance locale de hauteur → détecte les sols irréguliers dangereux même sans pente.
- [ ] **Ajouter un critère de franchissement de seuil** (`step_height`) :
  différence de hauteur max entre cellules adjacentes → détecte les marches et rochers.
- [ ] **Fenêtre glissante vs carte globale** : évaluer si une rolling window
  (suivi du robot) ou une carte globale accumulée donne de meilleurs résultats
  pour la planification globale.
- [ ] **Tune des coûts** selon les retours de simulation : les seuils 8°/10°/20°/30°/35°
  et les coûts 10/30/80/120/254 sont des valeurs initiales à valider empiriquement.
- [ ] **Tests en map inconnue** : vérifier que le comportement se généralise
  au-delà du carré 25×25 m.
- [ ] **Intégration d'un planificateur 3D** (ex. OMPL ou SBPL) pour remplacer
  navfn/NavFN si la navigation en dénivelé complexe dépasse les capacités du 2D.

---

## 10. Commits git

| Hash | Description |
|---|---|
| `5b4397d` | feat: add slopes_map world and square ramp traversal script |
| `7a63361` | feat: add IMU slope safety, speed curve, slip detection + slopes launch |
| `8d3882c` | feat: adaptive obstacle height filter (Piste 1) + relax min_slope_deg (Piste 2) |
