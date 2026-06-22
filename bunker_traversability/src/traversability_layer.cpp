#include "bunker_traversability/traversability_layer.h"
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_2d.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <cmath>
#include <cstring>
#include <limits>

// Register this class as a costmap_2d plugin.
PLUGINLIB_EXPORT_CLASS(bunker_traversability::TraversabilityLayer, costmap_2d::Layer)

namespace bunker_traversability {

// ── Constructor ────────────────────────────────────────────────────────────
// Default slope_cost_table matches costmap_common_params.yaml — used as
// fallback if the param server has no slope_cost_table entry.
TraversabilityLayer::TraversabilityLayer()
    : map_received_(false),
      lethal_slope_deg_(25.0),
      min_slope_deg_(5.0),
      update_radius_(8.0),
      min_hits_(3.0f),
      preserve_lethal_(false),
      delta_obstacle_threshold_(0.15),
      slope_cost_table_({{10.0, 10}, {20.0, 30}, {30.0, 50}, {35.0, 120}}) {}

// ── onInitialize ──────────────────────────────────────────────────────────
void TraversabilityLayer::onInitialize() {
    ros::NodeHandle nh("~/" + name_);

    nh.param("elevation_topic",  elevation_topic_,
             std::string("/bunker/elevation_map"));
    nh.param("lethal_slope_deg", lethal_slope_deg_, 25.0);
    nh.param("min_slope_deg",    min_slope_deg_,     5.0);

    double min_hits_d = 3.0;
    nh.param("min_hits",        min_hits_d,         3.0);
    min_hits_ = static_cast<float>(min_hits_d);

    nh.param("preserve_lethal",          preserve_lethal_,          false);
    nh.param("update_radius",            update_radius_,            8.0);
    nh.param("delta_obstacle_threshold", delta_obstacle_threshold_, 0.15);

    // Load slope_cost_table from param server (set in costmap_common_params.yaml).
    // Format: list of [max_slope_deg, cost] pairs.
    XmlRpc::XmlRpcValue table_param;
    if (nh.getParam("slope_cost_table", table_param) &&
        table_param.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        table_param.size() > 0)
    {
        slope_cost_table_.clear();
        for (int i = 0; i < table_param.size(); ++i) {
            XmlRpc::XmlRpcValue& entry = table_param[i];
            if (entry.getType() == XmlRpc::XmlRpcValue::TypeArray && entry.size() == 2) {
                const double  slope = static_cast<double>(entry[0]);
                const uint8_t cost  = static_cast<uint8_t>(static_cast<int>(entry[1]));
                slope_cost_table_.emplace_back(slope, cost);
            }
        }
        ROS_INFO("[TraversabilityLayer] Loaded %zu slope_cost_table entries from params.",
                 slope_cost_table_.size());
    }
    else
    {
        ROS_WARN("[TraversabilityLayer] slope_cost_table not found — using built-in defaults.");
    }

    matchSize();   // resize internal costmap to match parent

    ros::NodeHandle root_nh;
    map_sub_ = root_nh.subscribe(elevation_topic_, 1,
                                  &TraversabilityLayer::mapCallback, this);

    ROS_INFO("[TraversabilityLayer] Subscribing to '%s'  "
             "lethal=%.0f°  min_slope=%.0f°  min_hits=%.0f  "
             "delta_obstacle=%.2fm  update_radius=%.1fm",
             elevation_topic_.c_str(), lethal_slope_deg_, min_slope_deg_,
             static_cast<double>(min_hits_), delta_obstacle_threshold_,
             update_radius_);

    enabled_ = true;
}

// ── matchSize ─────────────────────────────────────────────────────────────
// Called whenever the parent costmap is resized.
void TraversabilityLayer::matchSize() {
    CostmapLayer::matchSize();
    resetMaps();  // fill internal costmap with NO_INFORMATION (255)
}

// ── reset ─────────────────────────────────────────────────────────────────
void TraversabilityLayer::reset() {
    resetMaps();
    map_received_ = false;
    // Ne pas passer current_ à false : move_base appelle reset() lors des
    // recovery behaviors et vérifie isCurrent() immédiatement après.
    // La prochaine callback d'élévation (2 Hz) va re-remplir le layer ;
    // bloquer le robot en attendant 500 ms n'apporte rien de sûr.
    current_ = true;
}

// ── Gaussian 3×3 smoothing ────────────────────────────────────────────────
// Lisse la couche d'élévation dans `out` AVANT le calcul de pente.
// Utilisation : réduire les spikes de transition rampe→plat qui créent des
// faux gradients élevés selon l'angle d'approche.
// `out` est initialisé depuis `in` — les cellules NaN ou de bord sont
// conservées telles quelles ; seules les cellules avec wsum > 0.5 sont lissées.
void TraversabilityLayer::smoothElevation3x3(const Eigen::MatrixXf& in,
                                              Eigen::MatrixXf& out) {
    // Noyau Gaussien 3×3 (σ≈1) : centre=0.25, arêtes=0.125, coins=0.0625
    static const float K[3][3] = {
        {0.0625f, 0.125f, 0.0625f},
        {0.125f,  0.25f,  0.125f},
        {0.0625f, 0.125f, 0.0625f}
    };
    out = in;  // copie (bords + NaN conservés)
    const int R = static_cast<int>(in.rows());
    const int C = static_cast<int>(in.cols());
    for (int r = 1; r < R - 1; ++r) {
        for (int c = 1; c < C - 1; ++c) {
            float sum = 0.f, wsum = 0.f;
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    const float v = in(r + dr, c + dc);
                    if (std::isfinite(v)) {
                        const float w = K[dr + 1][dc + 1];
                        sum  += w * v;
                        wsum += w;
                    }
                }
            }
            if (wsum > 0.5f)
                out(r, c) = sum / wsum;
        }
    }
}

// ── Elevation map callback ─────────────────────────────────────────────────
void TraversabilityLayer::mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    grid_map::GridMap raw;
    if (!grid_map::GridMapRosConverter::fromMessage(*msg, raw)) {
        ROS_WARN_THROTTLE(5.0, "[TraversabilityLayer] Failed to parse GridMap.");
        return;
    }
    if (!raw.exists("elevation")) {
        ROS_WARN_THROTTLE(5.0, "[TraversabilityLayer] 'elevation' layer missing.");
        return;
    }

    // Pre-compute slope layer from elevation gradient.
    // A cell is only eligible if all four neighbours have been observed at
    // least min_hits_ times — single-return cells on noisy terrain would
    // otherwise produce wildly exaggerated gradients.
    raw.add("slope", std::numeric_limits<float>::quiet_NaN());
    const double res       = raw.getResolution();
    const auto&  sz        = raw.getSize();
    const bool   has_hits  = raw.exists("hits");

    // Lissage Gaussien 3×3 sur une copie de la couche d'élévation.
    // Réduit les spikes de transition rampe→plat qui produisent de faux
    // gradients élevés selon l'angle d'approche.
    // NB : raw["elevation"] (données brutes) reste intact pour le check delta.
    Eigen::MatrixXf smooth_elev;
    smoothElevation3x3(raw["elevation"], smooth_elev);

    for (int row = 1; row < sz(0) - 1; ++row) {
        for (int col = 1; col < sz(1) - 1; ++col) {

            // ── Reliability guard (Problem 3) ──────────────────────────
            if (has_hits) {
                const float h_rp = raw["hits"](row + 1, col);
                const float h_rm = raw["hits"](row - 1, col);
                const float h_cp = raw["hits"](row, col + 1);
                const float h_cm = raw["hits"](row, col - 1);
                if (h_rp < min_hits_ || h_rm < min_hits_ ||
                    h_cp < min_hits_ || h_cm < min_hits_)
                    continue;  // not enough data — leave slope as NaN (transparent)
            }

            const float z_rp = smooth_elev(row + 1, col);
            const float z_rm = smooth_elev(row - 1, col);
            const float z_cp = smooth_elev(row, col + 1);
            const float z_cm = smooth_elev(row, col - 1);

            if (std::isnan(z_rp) || std::isnan(z_rm) ||
                std::isnan(z_cp) || std::isnan(z_cm))
                continue;

            const double dr = (z_rp - z_rm) / (2.0 * res);
            const double dc = (z_cp - z_cm) / (2.0 * res);
            raw["slope"](row, col) =
                static_cast<float>(std::atan(std::sqrt(dr*dr + dc*dc))
                                   * 180.0 / M_PI);
        }
    }

    // Swap into shared state under lock
    std::lock_guard<std::mutex> lock(map_mutex_);
    elevation_map_ = std::move(raw);
    map_received_  = true;
    current_       = true;
    // Note: addExtraBounds() intentionally NOT called here.
    // updateBounds() limits updates to update_radius_ around the robot,
    // avoiding a full-map re-evaluation (70×70 m) on every 2 Hz delivery.
}

// ── slopeToCost ───────────────────────────────────────────────────────────
uint8_t TraversabilityLayer::slopeToCost(double slope_deg) const {
    if (slope_deg >= lethal_slope_deg_)
        return costmap_2d::LETHAL_OBSTACLE;
    for (const auto& entry : slope_cost_table_) {
        if (slope_deg <= entry.first)
            return entry.second;
    }
    // Fallback: slope is above last table entry but below lethal — use last cost.
    return slope_cost_table_.empty() ? costmap_2d::LETHAL_OBSTACLE
                                     : slope_cost_table_.back().second;
}

// ── updateBounds ──────────────────────────────────────────────────────────
//
// Previously, this function expanded the update region to the full elevation
// map, forcing updateCosts() to iterate the entire map at every costmap cycle.
// Now it limits the update region to update_radius_ around
// the robot, typically reducing the evaluated area by 4–10×.
//
void TraversabilityLayer::updateBounds(double rx, double ry,
                                        double /*ryaw*/,
                                        double* min_x, double* min_y,
                                        double* max_x, double* max_y) {
    // Consume any pending extra-bounds (should be none after removing the
    // addExtraBounds call in mapCallback, but kept for safety).
    useExtraBounds(min_x, min_y, max_x, max_y);

    if (!map_received_) return;

    // Expand to a fixed-radius window around the robot — not the full map.
    // update_radius_ (default 12 m) covers the local costmap (10×10 m) plus
    // a planning lookahead margin. Distant cells keepentry their previously written
    // cost and are re-evaluated the next time the robot passes through.
    *min_x = std::min(*min_x, rx - update_radius_);
    *min_y = std::min(*min_y, ry - update_radius_);
    *max_x = std::max(*max_x, rx + update_radius_);
    *max_y = std::max(*max_y, ry + update_radius_);
}

// ── updateCosts ───────────────────────────────────────────────────────────
//
// Plugin order in costmap: ObstacleLayer → TraversabilityLayer → InflationLayer
//
// For each cell in the update region:
//   - No slope data        → write NO_INFORMATION → updateWithOverwrite skips it
//                            → master grid keeps ObstacleLayer's marking  ✓
//   - slope < min_slope    → same as no data (flat ground, real obstacles preserved) ✓
//   - min_slope ≤ slope < lethal → write graded cost, overrides ramp false-positives ✓
//   - slope ≥ lethal       → write LETHAL (cliff / wall) ✓
//
void TraversabilityLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                       int min_i, int min_j,
                                       int max_i, int max_j) {
    if (!enabled_ || !map_received_) return;

    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!elevation_map_.exists("slope")) return;

    // Présence du layer de surface max (disponible après mise à jour elevation_map_node).
    const bool has_elev_max = elevation_map_.exists("elevation_max");
    const float delta_thr   = static_cast<float>(delta_obstacle_threshold_);

    // Réinitialise uniquement la région active à NO_INFORMATION.
    {
        const unsigned int col_start = static_cast<unsigned int>(min_i);
        const unsigned int col_count = static_cast<unsigned int>(max_i - min_i);
        for (int j = min_j; j < max_j; ++j) {
            std::memset(costmap_ + static_cast<unsigned int>(j) * size_x_ + col_start,
                        costmap_2d::NO_INFORMATION,
                        col_count);
        }
    }

    double wx, wy;
    for (int j = min_j; j < max_j; ++j) {
        for (int i = min_i; i < max_i; ++i) {
            master_grid.mapToWorld(i, j, wx, wy);

            const grid_map::Position pos(wx, wy);
            if (!elevation_map_.isInside(pos)) continue;

            grid_map::Index gm_idx;
            if (!elevation_map_.getIndex(pos, gm_idx)) continue;

            // ── 1. Pente du sol (priorité) ────────────────────────────────────
            // La pente est calculée depuis elevation (min) → terrain réel.
            // Si la pente est connue ET supérieure à min_slope_deg, on est sur
            // du terrain incliné : on utilise le coût de pente et ON IGNORE le
            // check delta. Sur une pente, elevation_max - elevation reflète la
            // variation intra-cellule de la surface inclinée (≠ obstacle).
            const float slope_f = elevation_map_.isValid(gm_idx, "slope")
                                 ? elevation_map_["slope"](gm_idx(0), gm_idx(1))
                                 : std::numeric_limits<float>::quiet_NaN();
            const bool slope_valid = std::isfinite(slope_f);

            if (slope_valid && slope_f >= static_cast<float>(min_slope_deg_)) {
                // Terrain incliné connu → coût de pente, delta ignoré.
                setCost(i, j, slopeToCost(static_cast<double>(slope_f)));
                continue;
            }

            // ── 2. Détection d'obstacles via delta (terrain plat ou inconnu) ──
            // On cherche un delta significatif uniquement sur terrain plat/inconnu
            // (slope < min_slope_deg ou NaN) — là où un objet peut se distinguer
            // du sol par sa hauteur.
            if (has_elev_max && elevation_map_.isValid(gm_idx, "elevation_max")) {
                float elev_g = NAN;
                if (elevation_map_.isValid(gm_idx, "elevation"))
                    elev_g = elevation_map_["elevation"](gm_idx(0), gm_idx(1));
                const float elev_m = elevation_map_["elevation_max"](gm_idx(0), gm_idx(1));
                if (std::isfinite(elev_g) && std::isfinite(elev_m) &&
                    (elev_m - elev_g) >= delta_thr) {
                    setCost(i, j, costmap_2d::LETHAL_OBSTACLE);
                    continue;
                }
            }

            // ── 3. Terrain plat sans obstacle ────────────────────────────────
            // slope valide mais < min_slope_deg → transparent (laisser NO_INFORMATION)
            // slope NaN → pas de données → transparent
        }
    }

    // Copie les cellules non-NO_INFORMATION dans le master costmap.
    updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace bunker_traversability
