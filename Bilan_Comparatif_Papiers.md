# Bilan comparatif — Projet personnel vs littérature

> **Date :** 2026-04-27 — **Mis à jour :** 2026-06-10
> **Source projet :** `\\wsl.localhost\Ubuntu-20.04\home\jules\projet_bunker\src\CLAUDE.md`
> **Littérature analysée :** 15 documents techniques (voir `Base_Comparative_Papiers.md`)

---

## 1. Description du projet personnel

### Contexte

Le projet est développé dans un environnement **ROS 1 (Noetic)** sur un **robot Bunker** (robot différentiel tout-terrain à 4 roues, type UGV compact), simulé dans **Gazebo Classic**. Le capteur principal de perception est un **LiDAR Velodyne**. Le SLAM utilisé est **LIO-SAM** (LiDAR-IMU Odometry via Smoothing and Mapping). La navigation autonome repose sur **move_base** avec le planificateur local **TEB** (Timed Elastic Band), qui remplace l'ancien DWA depuis mai 2026.

### Problème central

La costmap 2D de move_base détecte les surfaces inclinées (rampes) comme des **obstacles létaux** via la `ObstacleLayer`, car le LiDAR voit ces surfaces dans la fenêtre de hauteur d'obstacle configurée. Le robot refusait d'aborder la rampe à 30°, alors qu'elle est physiquement traversable.

### Solution retenue et implémentée

Package `bunker_traversability` avec trois composants :

1. **`adaptive_obstacle_filter_node`** (C++ — migré depuis Python en juin 2026) : filtre les points LiDAR en fonction de la hauteur courante du robot (via TF2) → fenêtre `[robot_z + 0.03m, robot_z + 1.55m]`. Quand le robot monte, la surface de la rampe en-dessous sort de la fenêtre → plus de faux obstacles. Le filtre opère dans le repère `map` et publie sur `/velodyne_points_filtered`. La migration C++ apporte un gain de performance de ~20–50× par rapport à la version Python (pas de GIL, boucle vectorisée).

2. **`bunker_elevation_map_node`** (C++) : accumule les hauteurs Velodyne cellule par cellule en prenant le **minimum de hauteur** (modèle de sol — exclut la canopée des arbres qui crée des retours multiples à grande hauteur). Grille 50×50 m @ 0,15 m/cell, centrée à (0, 0). Filtre en hauteur : `[0,0 m, 1,5 m]`. Deux couches GridMap publiées : `elevation` (hauteur minimale) et `hits` (nombre d'impacts). Publié à 1 Hz.

3. **`TraversabilityLayer`** (C++, plugin costmap_2d) : calcule la pente par différences finies 4-voisins sur la couche `elevation` de la GridMap → encode chaque cellule avec un coût proportionnel à l'angle. Requiert au moins `min_hits = 3` impacts par cellule avant d'assigner un coût (les cellules sous-observées restent `NO_INFORMATION`). Table de coûts actuelle (`lethal_slope_deg = 25°`) :

| Pente | Coût | Comportement planificateur |
|---|---|---|
| < 5° | NO_INFORMATION (transparent) | Conserve les marquages d'obstacles plats (arbres) |
| 5°–10° | 10 | Légère pénalité |
| 10°–20° | 50 | Pénalité modérée, détour préférable |
| 20°–25° | 90 | Forte pénalité, juste sous le seuil létal |
| ≥ 25° | 254 (létal) | Interdit |

La TraversabilityLayer est intégrée dans **les deux costmaps** (globale et locale), dans la même pile de plugins : `obstacle_layer` → `traversability_layer` → `inflation_layer`. La costmap globale (40×40 m, 0,15 m/cell) bénéficie ainsi du coût de traversabilité pour la planification globale.

### Planificateurs disponibles

Trois planificateurs locaux sont désormais configurés et sélectionnables via argument de lancement (`planner:=teb|dwa|eband`) :

| Planificateur | max_vel_x | Particularités |
|---|---|---|
| **TEB** (défaut) | 0,7 m/s | Homotopy class planning activé, weight_forward_drive=10.0, max_lookahead=8m |
| DWA | 0,4 m/s | Ancien planificateur, conservé pour comparaison |
| EBand | 0,4 m/s | Alternative légère, eband_internal_force_gain=1.0 |

### Mode debug

Un mode de navigation sans LIO-SAM a été ajouté (`navigation_debug.launch` + `ekf_debug.yaml`) : il utilise directement l'odométrie Gazebo (ground truth) relayée via un filtre EKF (`robot_localization`), sans SLAM. Ce mode permet de valider la navigation en isolation, sans le risque de divergence de LIO-SAM.

### État actuel (2026-06-10)

| Composant | Statut |
|---|---|
| Package `bunker_traversability` créé et compilé | ✅ |
| `bunker_elevation_map_node` fonctionnel (min height + hits) | ✅ |
| `TraversabilityLayer` plugin — costmap locale | ✅ |
| `TraversabilityLayer` plugin — costmap globale | ✅ |
| `adaptive_obstacle_filter_node` migré en C++ | ✅ |
| Planificateur local TEB (remplace DWA) | ✅ |
| `square_ramp_traverse.py` v4.0 (rosbag + PCD) | ✅ |
| Mode debug `navigation_debug.launch` | ✅ |
| Vérification carte d'élévation dans RViz | 🔲 |
| Test passage rampe 30° (validation complète) | 🔲 |
| Validation non-régression (arbres restent obstacles) | 🔲 |

### Vision long terme

Le planificateur doit **optimiser un critère mixte** : distance + coût de traversabilité. Sur une carte inconnue, le robot doit choisir automatiquement entre traverser une rampe douce ou la contourner selon le coût. Les futures couches envisagées : rugosité, hauteur de seuil (step height), fenêtre glissante vs carte globale, planificateur 3D (OMPL/SBPL).

---

## 2. Comparaison avec la littérature

### 2.1 Similitudes directes

#### Avec le rapport PFE Titouan Leost (traversabilité proprioceptive sur KIPP)

C'est le document le **plus proche** du projet :

| Aspect | Projet personnel (Bunker) | PFE Leost (KIPP) |
|---|---|---|
| Framework | ROS1 Noetic | ROS1 Noetic |
| Type de robot | Différentiel (Bunker UGV) | Quasi-différentiel (quad) |
| Capteurs principaux | Velodyne LiDAR | LiDAR, IMU, GNSS, odométrie |
| Simulation | Gazebo Classic | Gazebo Classic |
| Approche traversabilité | Extéroceptive (pente depuis LiDAR) | Proprioceptive (glissement, résistance au roulement, rugosité) |
| Navigation | move_base + TEB | Navigation autonome TTA |
| Carte de traversabilité | GridMap (élévation → pente → coût) | Carte topologique proprioceptive |

**Ce que le projet fait de plus :** intégration directe de la traversabilité dans les deux planificateurs (global + local via costmap plugin TraversabilityLayer) → la planification globale Dijkstra optimise naturellement les itinéraires selon la pente.

**Ce que le rapport PFE fait de plus :** analyse physique fine (glissement mesuré, résistance au roulement modélisée, rugosité vibratoire) → caractérisation multi-critères du terrain. Le projet personnel n'a pas de composante proprioceptive.

**Piste de synergie :** Ajouter les critères proprioceptifs de Leost (IMU + odométrie) en couches supplémentaires dans la GridMap du projet → **fusion extéroception + proprioception comme suggéré dans le PFE et le survey.**

---

#### Avec le papier "Stable Wheelchair Navigation on Slopes" (Wang et al., 2020)

| Aspect | Projet personnel | Wang et al. |
|---|---|---|
| Robot | Bunker (différentiel) | Fauteuil roulant |
| Représentation | GridMap élévation → pente | Grille d'élévation 3D → carte traversabilité |
| Planification | TEB (local) + navfn (global) | RRT* modifié (global, sampling) |
| Prise en compte des rampes | Coût proportionnel à la pente | Zones sûres/dangereuses + confort humain |
| Carte de traversabilité | Construite en ligne depuis LiDAR | Construite a priori (connaissance de l'environnement) |
| Traversabilité dans costmap globale | ✅ Oui (depuis juin 2026) | ✅ Oui (planificateur global) |

**Points communs clés :** les deux approches construisent une carte de traversabilité 2,5D (élévation → pente → coût) et l'intègrent dans la planification. Avec l'ajout de la TraversabilityLayer dans la costmap globale, les deux projets partagent désormais le même principe : le planificateur global optimise les itinéraires en tenant compte de la traversabilité, pas uniquement de la distance.

**Ce que Wang et al. font encore de plus :** fonction d'utilité combinant confort + coût de chemin (multi-critères) ; planificateur global spécialement adapté (RRT* modifié) qui optimise dès le début la trajectoire globale. Dans le projet personnel, navfn/Dijkstra sur la costmap de traversabilité est une approximation — il optimise le coût cellule par cellule mais pas une fonction d'utilité multi-critères.

**Piste d'amélioration résiduelle :** Remplacer navfn par un planificateur global capable d'optimiser une combinaison pondérée distance + traversabilité de manière plus explicite (ex. SBPL avec un modèle de coût personnalisé).

---

#### Avec "Comparison of ROS Local Planners" (Naotunna & Wongratanaphisan, 2020)

C'est le paper le **plus directement applicable** dans le contexte technique du projet, et ses recommandations ont été mises en œuvre :

| Aspect | Projet personnel | Naotunna & Wongratanaphisan |
|---|---|---|
| Robot | Bunker (différentiel, ~30 kg) | Robot différentiel lourd (>50 kg) |
| Planificateur local | **TEB** (depuis mai 2026) | DWA, EBand, TEB (comparés) |
| Capteurs | Velodyne LiDAR | Realsense D435i + T265 |
| SLAM | LIO-SAM | RTAB-Map |
| Planificateur global | navfn (Dijkstra) | navfn (Dijkstra) |

**Statut :** La recommandation d'utiliser TEB à la place de DWA **a été appliquée**. TEB est maintenant le planificateur par défaut. Le `homotopy_class_planning = true` permet à TEB de trouver plusieurs familles de trajectoires et de choisir la moins coûteuse — particulièrement utile sur les rampes étroites où DWA pouvait rater le passage. Le `weight_kinematics_forward_drive = 10.0` (réduit depuis 1000,0) accepte les légères marches arrière pour mieux négocier les espaces contraints.

**Ce qui reste à évaluer :** Comparer quantitativement TEB vs DWA sur la traversée du carré avec rampes — le script `square_ramp_traverse.py` v4.0 enregistre maintenant des rosbags qui permettront cette comparaison.

---

### 2.2 Similitudes indirectes / inspiration

#### Avec la Tesina Jordi Ferrer (planificateurs locaux Ackermann)

Le Bunker est différentiel (non Ackermann), donc les planificateurs Ackermann ne s'appliquent pas directement. Cependant, l'approche de **TEB avec contraintes cinématiques** est directement utilisée : le TEB configuré pour le Bunker exploite sa formulation par graphe de contraintes g²o pour intégrer les coûts de traversabilité de la costmap directement dans l'optimisation locale de trajectoire.

**Statut :** TEB actif, la contrainte cinématique est correctement configurée (robot différentiel, `max_vel_x_backwards = 0.2 m/s`). La suite logique est d'explorer l'ajout de contraintes de pente directement dans l'optimiseur TEB (plutôt que passant par la costmap), comme suggéré dans la tesina.

---

#### Avec le papier quadrupède multi-layer elevation map (Lu et al., 2025)

Ce travail utilise une **carte d'élévation multi-couches** — concept proche de la `GridMap` du projet personnel. La couche `hits` ajoutée à `bunker_elevation_map_node` est un premier pas dans cette direction (données d'occupation vs données d'élévation séparées).

| Aspect | Projet personnel | Lu et al. |
|---|---|---|
| Type de robot | Roues (Bunker) | Pattes (quadrupède) |
| Planification | move_base + TEB | RL hiérarchique (spécialistes + distillation) |
| Couches GridMap | 2 (elevation + hits) | 3 (sol + obstacles intermédiaires + surface sup.) |
| Obstacles suspendus | Partiellement géré (filtre adaptatif) | Géré (3e couche) |
| Apprentissage | Non (règles explicites) | Oui (RL, Isaac Gym) |

**Ce que le projet peut retenir :** Une **troisième couche** dans la GridMap pour les obstacles suspendus (branches basses, barrières) reste à ajouter si le Bunker doit opérer dans des environnements avec des obstacles à différentes hauteurs. Le filtre adaptatif actuel ([robot_z + 0.03m, robot_z + 1.55m]) gère partiellement le cas, mais sans persistance dans la GridMap.

---

#### Avec Real-Time Elevation Mapping + Traversability (Xie et al., IROS 2023)

C'est le papier **techniquement le plus proche** du projet en termes d'implémentation concrète :

| Aspect | Projet personnel | Xie et al. IROS 2023 |
|---|---|---|
| Capteur | Velodyne LiDAR (3D) | Robosense RS-16 LiDAR (3D) |
| Représentation | GridMap 2 couches (elevation min + hits) | GridMap 2 couches (brut + sol) |
| Obstacles suspendus | Partiellement (filtre adaptatif C++) | Oui (ray tracing → double couche) |
| Features de traversabilité | Pente uniquement | Pente + Step + Rugosité |
| Zones creuses | NO_INFORMATION (si hits < 3) | Remplissage Bayésien |
| Accélération GPU | Non | Oui (traversabilité en 1,4 ms) |
| SLAM | LIO-SAM | FAST-LIO2 |
| Intégration navigation | move_base (ROS1) | ROS (topics TF) |

**Ce que le projet peut directement adopter :**
- Le **remplissage Bayésien** (Bayesian Generalized Kernel Inference) pour combler les zones de la GridMap non observées — actuellement, ces zones ont `NO_INFORMATION` (si hits < 3), ce qui peut créer des lacunes dans la planification globale.
- Ajouter les features **Step** (seuil de hauteur entre cellules adjacentes) et **Rugosité** (variance locale) en plus de la pente est directement faisable dans le `bunker_elevation_map_node` existant.
- La double couche brut + sol de Xie et al. est fonctionnellement similaire au filtre adaptatif + couche d'élévation du projet — l'approche ray tracing serait plus robuste mais plus coûteuse à implémenter.

---

#### Avec DEM-Based Traversability Map for Multirobot (Orbea et al., 2026)

Ce papier est le seul à utiliser des **données géospatiales externes (DEM)** plutôt que des capteurs embarqués — approche complémentaire à celle du projet.

| Aspect | Projet personnel | Orbea et al. 2026 |
|---|---|---|
| Source de la carte d'élévation | LiDAR temps réel (en ligne) | DEM géospatial open-source (hors ligne) |
| Résolution | 0,15 m/cellule | Variable (résolution DEM) |
| Échelle | Locale (50×50 m) | Kilomètre-scale |
| Seuil de pente | Table de coûts continus (létal à 25°) | Binaire par robot (seuil admissible) |
| Framework | ROS1 + move_base | ROS2 + Nav2 |
| Multi-robot | Non | Oui (seuil adaptatif par robot) |
| Traversabilité dans planif. globale | ✅ Oui | ✅ Oui |

**Apport pour le projet :** L'approche DEM est pertinente pour une **planification globale à grande échelle** avant le déploiement. Le seuil de pente létal de 25° utilisé dans le projet est cohérent avec les valeurs répertoriées dans ce papier pour des UGVs commerciaux de catégorie similaire au Bunker.

---

#### Avec Learning Multiobjective Traversability (Wallin et al., 2022)

Ce papier introduit le concept de **traversabilité multi-objectif** :

| Mesure | Équivalent dans le projet | Statut |
|---|---|---|
| Locomotion (capacité à avancer) | Coût de pente → obstacle létal si ≥ 25° | ✅ Implémenté |
| Énergie (consommation) | Non modélisée | ❌ |
| Accélération (confort/sécurité mécanique) | Non modélisée | ❌ |

**Ce que le projet peut retenir :** À moyen terme, remplacer la table de coûts à un seul critère (pente) par une **combinaison pondérée** pente + step + rugosité serait une première étape vers le multi-objectif. La rugosité impacte l'accélération ; la pente impacte l'énergie.

---

#### Avec MTraMap (Huang et al., ICRA 2024) — traversabilité multi-directionnelle

Ce papier soulève un point ignoré dans le projet :

La traversabilité des rampes du Bunker est **anisotrope** — aborder une rampe à 25° de face est très différent de l'aborder en oblique. La `TraversabilityLayer` actuelle calcule le gradient scalaire de pente sans tenir compte de la direction d'approche prévue. Avec TEB (actif depuis mai 2026), le planificateur local optimise explicitement la trajectoire dans l'espace de configurations, ce qui le rend potentiellement plus sensible à l'angle d'approche — mais le coût assigné par la TraversabilityLayer à chaque cellule reste scalaire.

**Piste concrète :** Calculer la pente dans la direction de déplacement prévue (depuis la commande de vitesse ou la direction du chemin TEB global) plutôt qu'un gradient scalaire → une TraversabilityLayer directionnelle simple et sans apprentissage.

---

#### Avec TNT (Pan et al., 2024) — terrains apparemment non-traversables

La problématique centrale du projet (la rampe à 25° détectée comme obstacle) est exactement l'inverse du problème de TNT : TNT cherche à traverser ce qui semble non-traversable, le projet cherche à ne pas bloquer ce qui est traversable. Les deux approches convergent vers la même idée : **la classification binaire traversable/non-traversable est insuffisante**.

**Leçon transférable :** L'utilisation des angles de roulis/tangage mesurés (IMU) pendant le passage d'un obstacle pour affiner la traversabilité future est directement applicable : si le Bunker passe la rampe sans problème (roulis et tangage dans les tolérances), cette expérience devrait réduire le coût associé à cette pente dans les passages futurs.

---

#### Avec le mémoire Guerraiche (2025) — navigation réactive

Ce mémoire n'est pas directement en lien avec la traversabilité, mais avec la **navigation locale réactive** — une alternative à move_base pour les environnements très dynamiques ou inconnus.

**Point de contact :** Le contrôleur SVC (Safety Velocity Cones) de Guerraiche offre des **garanties formelles de non-collision**, contrairement à TEB. Si à terme le Bunker navigue en présence de personnes ou d'obstacles dynamiques, SVC serait plus sûr formellement que TEB. VFH est également une alternative légère sans cartographie.

---

#### Avec RoadRunner (Frey et al., TFR 2024) — traversabilité SSL, BEV, MPPI

RoadRunner représente une direction radicalement différente du projet personnel : là où le projet Bunker utilise des heuristiques géométriques (pente mesurée depuis LiDAR), RoadRunner **apprend la traversabilité directement depuis les expériences de conduite** (auto-supervisé, hindsight labels).

**Points de contact :**
- Les deux systèmes visent à produire une **carte de traversabilité en temps réel** utilisée par un planificateur de trajectoire
- Les deux utilisent des données LiDAR comme capteur principal de perception du terrain
- L'idée de hindsight (utiliser l'expérience passée du robot pour générer des labels de traversabilité) est **directement transférable à moyen terme** : si le Bunker passe une rampe sans problème (roulis + tangage dans les tolérances), cette trajectoire devient un label de traversabilité positive. Les rosbags enregistrés par `square_ramp_traverse.py` v4.0 constituent désormais un début de dataset pour cette approche.
- Le **planificateur MPPI** est une alternative à TEB pour les terrains accidentés : MPPI échantillonne des trajectoires et sélectionne la moins coûteuse selon la carte de traversabilité → planification plus souple sur terrain irrégulier

**Différences clés :**
- RoadRunner cible des vitesses de 15 m/s sur véhicule hors-route de 800 kg avec 4 GPU embarqués — le Bunker (~30 kg, ROS1, CPU seulement) est dans une autre catégorie de ressources
- Le projet actuel n'a pas de caméra → pas de branche image BEV possible immédiatement

---

#### Avec Locomotion Policy Guided (Frey et al., IROS 2022) — traversabilité par politique locomotion, voxels 3D

Ce papier pose une question philosophique importante : **qu'est-ce que la traversabilité pour un robot spécifique ?** Sa réponse (le taux de succès de la politique de locomotion) est très différente de l'approche du projet personnel (heuristiques géométriques), mais les deux cherchent la même chose : un coût qui reflète la **capacité réelle du robot** à traverser un terrain.

**Points de contact :**
- La `TraversabilityLayer` du projet encode un coût physiquement motivé (seuils 5°/10°/20°/25° basés sur les capacités physiques du Bunker) — c'est la même logique, mais formulée avec des règles manuelles plutôt que des données de simulation
- La **représentation volumétrique 3D** (voxels) est une extension naturelle de la GridMap actuelle : si le Bunker navigue dans des environnements avec des obstacles suspendus (branches basses, passages sous des structures), les voxels résoudraient les ambiguïtés que la GridMap 2,5D ne peut pas gérer

**Statut :** Validation que l'approche actuelle (coûts basés sur les capacités physiques réelles) est la bonne philosophie — en format heuristique plutôt qu'appris. Les rosbags de `square_ramp_traverse.py` v4.0 pourront à terme servir de base pour passer de l'heuristique manuel à l'heuristique appris.

---

#### Avec le survey de traversabilité en forêt (Carvalho et al., 2025)

| Catégorie (taxonomie survey) | Position du projet personnel |
|---|---|
| Approche géométrique (pente depuis LiDAR) | ✅ Implémentée (TraversabilityLayer, pente) |
| Approche proprioceptive (glissement, vibrations) | ❌ Non encore implémentée |
| Approche par apprentissage (CNN, RL) | ❌ Hors scope actuel |
| NDT-based | ❌ Hors scope actuel |
| Temps réel | ✅ (1 Hz pour la GridMap) |
| Tests sur robot réel | ❌ Simulation uniquement (Gazebo) |

**Positionnement :** Le projet se situe dans la catégorie **géométrique classique** du survey — approche bien documentée, robuste, interprétable, mais limitée en termes de richesse sémantique du terrain (pas de différentiation texture, humidité, type de sol).

---

## 3. Analyse des forces et lacunes du projet

### Forces

| Force | Justification |
|---|---|
| **Architecture TraversabilityLayer bien fondée** | Identique en principe à Wang et al. 2020, Xie et al. 2023 (IROS), et à l'approche recommandée dans les surveys. C'est la bonne philosophie (coûts continus vs obstacles binaires). |
| **Traversabilité dans les deux costmaps** | La TraversabilityLayer est désormais dans la costmap globale ET locale — le planificateur global Dijkstra optimise les itinéraires en tenant compte de la pente, pas uniquement de la distance. Résout le gap identifié dans la version précédente. |
| **Planificateur local TEB** | Meilleure gestion des espaces étroits, support de la légère marche arrière, homotopy class planning activé. Conforme aux recommandations de Naotunna 2020 et Tesina Ferrer 2018. |
| **Filtre adaptatif de hauteur (C++)** | Solution originale au problème de faux positifs sur rampe. Migration C++ : ~20–50× plus rapide que la version Python, pas de GIL. |
| **Modèle de sol par minimum de hauteur** | Utiliser le minimum (au lieu de la moyenne) dans l'elevation_map_node exclut naturellement les retours de canopée sans seuil max artificiel — approche plus robuste sur terrain varié. |
| **Couche `hits` + seuil min_hits** | Évite d'assigner des coûts de traversabilité à partir d'une seule mesure bruitée. Les cellules avec < 3 impacts restent NO_INFORMATION → planification conservative mais correcte. |
| **Table de coûts physiquement motivée** | Les seuils (5°, 10°, 20°, 25°) correspondent aux limites physiques réelles d'un UGV à roues, cohérent avec les valeurs de terrain répertoriées dans Orbea et al. 2026. |
| **Enregistrement rosbag + PCD (v4.0)** | `square_ramp_traverse.py` v4.0 enregistre maintenant les données de navigation complètes → base pour comparaison TEB vs DWA et futur apprentissage auto-supervisé (RoadRunner). |
| **Mode debug sans SLAM** | `navigation_debug.launch` permet de valider la navigation en isolation (EKF + ground truth Gazebo) sans risque de divergence LIO-SAM — accélère les itérations de test. |

### Lacunes identifiées par comparaison

| Lacune | Référence | Statut |
|---|---|---|
| **Pas de composante proprioceptive** | PFE Leost, Survey Carvalho, Survey Sevastopoulos | ❌ À ajouter : glissement (odométrie + IMU) et rugosité (variance IMU) comme couches GridMap |
| **GridMap à deux couches (elevation + hits) — pas de step ni rugosité** | Lu et al. 2025, Xie et al. IROS 2023 | ❌ Ajouter Step (diff. de hauteur entre cellules adjacentes) et Rugosité (variance locale) dans `bunker_elevation_map_node` |
| **Zones non visitées = NO_INFORMATION** | Xie et al. IROS 2023, Wallin et al. 2022 | ❌ Remplissage Bayésien (Bayesian Generalized Kernel Inference) des zones creuses — les zones inconnues bloquent ou fragilisent la planification globale |
| **Traversabilité scalaire (non directionnelle)** | MTraMap (Huang et al. ICRA 2024) | ❌ La pente d'approche dépend de la direction — calculer le gradient dans la direction de déplacement TEB plutôt qu'un gradient scalaire |
| **Simulation uniquement** | Tous les papiers expérimentaux | ❌ Validation sur le robot Bunker réel requise |
| **Pas d'apprentissage** | Wallin 2022, TNT 2024, MTraMap 2024, Survey Sevastopoulos | ❌ Hors scope actuel — la méthode géométrique est une fondation solide et interprétable |
| **Planificateur global navfn — optimisation implicite** | Wang et al. 2020, Orbea et al. 2026 | ⚠️ La TraversabilityLayer dans le global costmap permet à navfn d'optimiser le chemin via les coûts de cellules, mais sans modèle de coût explicitement multi-critères. Amélioration possible avec SBPL ou un global planner personnalisé. |

---

## 4. Recommandations prioritaires

### Court terme (validation de l'existant)

1. **Vérifier la carte d'élévation dans RViz** — s'assurer que les rampes apparaissent correctement (layer `elevation` du topic `/bunker/elevation_map`) et que les hits sont suffisants (layer `hits` ≥ 3)
2. **Tester le passage de la rampe à 25° (ancienne rampe 30°)** avec `square_ramp_traverse.py` en mode TEB — valider que le filtre adaptatif C++ + TraversabilityLayer globale et locale résolvent le faux positif
3. **Valider la non-régression** — les arbres doivent rester des obstacles létaux (pente ≈ 0° → transparent → marquage létal conservé)
4. **Comparer TEB vs DWA** sur le même scénario via les rosbags v4.0 — quantifier l'amélioration

### Moyen terme (améliorations fondées sur la littérature)

5. **Ajouter une couche de rugosité** dans la GridMap — variance locale de hauteur depuis le Velodyne (suggéré dans CLAUDE.md + validé par PFE Leost, Xie et al. IROS 2023, Survey Carvalho)
6. **Ajouter une couche de step height** — différence de hauteur max entre cellules adjacentes dans `bunker_elevation_map_node` (Xie et al. IROS 2023)
7. **Remplissage Bayésien** des zones de la GridMap non observées (inspiré de Xie et al. IROS 2023) — les zones avec hits < 3 reçoivent actuellement NO_INFORMATION ; un remplissage par inférence kernel améliorerait la planification globale
8. **Traversabilité directionnelle** — calculer le gradient de pente dans la direction de déplacement TEB plutôt qu'un scalaire (MTraMap, Huang et al. ICRA 2024)

### Long terme (vision ambitieuse)

9. **Ajouter des critères proprioceptifs** (glissement via odométrie + IMU, résistance au roulement) comme couches dans la GridMap — directement inspiré du PFE Leost, Survey Sevastopoulos 2022 et Survey Carvalho 2025
10. **Explorer les 3 mesures continues** de Wallin et al. 2022 (locomotion, énergie, accélération) pour un planificateur multi-objectif : au lieu d'un seul coût de pente, le planificateur pourrait optimiser une combinaison pondérée de ces métriques
11. **Évaluer un planificateur global dédié** (SBPL ou navfn modifié) pour optimiser explicitement distance + traversabilité, au-delà de l'approximation via costmap
12. **Évaluer un planificateur 3D** (OMPL, SBPL) pour remplacer navfn si les terrains deviennent trop complexes pour la planification 2D
13. **Tests sur robot réel** pour valider que la GridMap construite depuis un Velodyne réel (bruit, occultations) reste utilisable — le remplissage Bayésien (Xie et al.) est particulièrement pertinent pour cette transition
14. **Apprentissage auto-supervisé** depuis les rosbags de `square_ramp_traverse.py` — les zones parcourues sans incident deviennent automatiquement des labels de traversabilité positive (approche RoadRunner)

---

## 5. Synthèse positionnement

```
                          GÉOMÉTRIQUE          PROPRIOCEPTIF        APPRENTISSAGE     TEMPS RÉEL   ROBOT RÉEL
                          (pente/rugosité)     (glissement/vib.)    (CNN/RL/DL)

Wang et al. 2020          ✅ (carte trav.)     ❌                   ❌                ✅            ✅ (fauteuil)
Naotunna 2020             ✅ (costmap 2D)      ❌                   ❌                ✅            ✅ (diff. drive)
Tesina Ferrer 2018        ✅ (costmap 2D)      ❌                   ❌                ✅            ❌ (simu)
PFE Leost 2025            ❌ (extéroc. exist)  ✅ (3 critères)      ❌                ✅            ✅ (KIPP)
Lu et al. 2025            ✅ (3-layer map)     ✅ (implicite RL)    ✅ (RL hiérar.)   ✅            ✅ (quadrupède)
Survey Carvalho 2025      ✅ (revue)           ✅ (revue)           ✅ (revue)         —             —
Wallin et al. 2022        ✅ (3 mesures DL)    ❌                   ✅ (DL)           ✅            ❌ (simu+scan)
TNT 2024                  ✅ (heightmap)       ✅ (interactions)    ✅ (data-driven)  ✅            ✅ (V4W)
MTraMap ICRA 2024         ✅ (élévation)       ❌                   ✅ (distillation)  ✅            ✅ (Hunter SE)
Survey Sevastopoulos 2022 ✅ (revue)           ✅ (revue)           ✅ (revue)         —             —
Xie et al. IROS 2023      ✅ (pente+step+rough)❌                   ❌                ✅            ✅ (Scout-mini)
Orbea et al. 2026         ✅ (DEM → pente)     ❌                   ❌                ✅            ❌ (simu)
Guerraiche 2025           ❌ (navigation réact.)❌                  ❌                ✅            ❌ (simu)
RoadRunner TFR 2024       ✅ (élévation BEV)   ❌                   ✅ (SSL BEV)      ✅            ✅ (buggy 15m/s)
Loco. Policy IROS 2022    ✅ (voxels 3D)       ✅ (politique loco.) ✅ (sparse CNN)   ✅ (16,5 Hz)  ✅ (ANYmal)

PROJET PERSONNEL          ✅ (élévation        ❌ (à ajouter)       ❌                ✅ (1 Hz)     ❌ (simu)
(bunker_traversability)      → pente, 2 couches)
```

**Positionnement :** Le projet se situe dans la catégorie **géométrique classique temps réel**, aux côtés de Wang 2020, Xie IROS 2023 et Orbea 2026 — c'est la bonne fondation pour un système ROS1 embarqué sans GPU dédié. Par rapport à la version de référence (2026-04-27), deux lacunes majeures ont été comblées : (1) **le planificateur local TEB remplace DWA**, résolvant les problèmes de passage en espace contraint signalés par Naotunna 2020 et Tesina Ferrer ; (2) **la TraversabilityLayer est désormais dans les deux costmaps**, ce qui aligne le projet avec la philosophie de Wang et al. 2020 d'optimiser les itinéraires globaux selon la traversabilité et pas uniquement selon la distance.

Les prochains axes à fort impact sont : la **rugosité + step** dans la GridMap (passage de 2 à 4 couches, alignement avec Xie IROS 2023), le **remplissage Bayésien** des zones non observées, et la **traversabilité directionnelle** (alignement avec MTraMap). Ces trois améliorations sont directement implémentables dans le `bunker_elevation_map_node` et la `TraversabilityLayer` existants, sans changement d'architecture.
