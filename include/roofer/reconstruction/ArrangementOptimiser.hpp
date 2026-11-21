// Copyright (c) 2018-2026 TU Delft 3D geoinformation group, Ravi Peters (3DGI),
// and Balazs Dukai (3DGI)

// This file is part of roofer (https://github.com/3DBAG/roofer)

// geoflow-roofer was created as part of the 3DBAG project by the TU Delft 3D
// geoinformation group (3d.bk.tudelf.nl) and 3DGI (3dgi.nl)

// geoflow-roofer is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option) any
// later version. geoflow-roofer is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
// Public License for more details. You should have received a copy of the GNU
// General Public License along with geoflow-roofer. If not, see
// <https://www.gnu.org/licenses/>.

// Author(s):
// Ravi Peters

#pragma once
#include <map>
#include <memory>
#include <roofer/common/Raster.hpp>
#include <roofer/common/datastructures.hpp>
#include <roofer/reconstruction/cgal_shared_definitions.hpp>
#include <roofer/common/ConfigField.hpp>

namespace roofer::reconstruction {

#define ROOFER_ARRANGEMENT_OPTIMISER_FIELDS(X)                            \
  X(float, complexity_factor, 0.888F,                                     \
    "Balance between data fidelity (1) and smoothness (0).",              \
    config::in_range(0.0F, 1.0F), public_)                                \
  X(bool, preset_labels, false, "Preset graph-cut labels.",               \
    config::no_validation<bool>(), internal)                              \
  X(bool, normalise, false, "Normalise graph-cut costs.",                 \
    config::no_validation<bool>(), internal)                              \
  X(int, graph_cut_impl, 0, "Graph-cut implementation selector.",         \
    config::at_least(0), internal)                                        \
  X(bool, label_ground_outside_footprint, true,                           \
    "Label ground outside the footprint.", config::no_validation<bool>(), \
    internal)                                                             \
  X(bool, use_ground, true, "Use ground labels (pipeline-derived).",      \
    config::no_validation<bool>(), internal)                              \
  X(float, label_cost, 0.0F,                                              \
    "Cost of using a roof plane label at all (Delong et al. 2012), in "   \
    "cubic metres of height-field volume; scaled like the data term. "    \
    "0 disables label costs.",                                            \
    config::at_least(0.0F), public_)                                      \
  X(float, intersection_discount, 0.0F,                                   \
    "Fraction of the edge cost removed when an edge lies on the "         \
    "intersection line of the two planes it separates. 0 disables.",      \
    config::in_range(0.0F, 1.0F), public_)                                \
  X(float, intersection_distance, 0.5F,                                   \
    "Distance from an edge to the plane intersection line at which the "  \
    "intersection discount has faded to zero, in metres.",                \
    config::greater_than(0.0F), public_)                                  \
  X(int, intersection_min_adjacency, 5,                                   \
    "Minimum number of neighbouring point pairs between two planes for "  \
    "their intersection discount (when plane adjacencies are given).",    \
    config::at_least(0), public_)                                         \
  X(bool, init_best_data_label, false,                                    \
    "Start Roofer's CGAL graph-cut from each face's best-fitting plane "  \
    "instead of the first plane.",                                        \
    config::no_validation<bool>(), public_)                               \
  X(bool, label_cost_solver, false,                                       \
    "Use the label-cost alpha-expansion solver even when label_cost and " \
    "intersection_discount are 0 (for ablations).",                       \
    config::no_validation<bool>(), public_)                               \
  X(int, max_expansion_cycles, 20,                                        \
    "Maximum number of alpha-expansion cycles of the label-cost solver.", \
    config::at_least(1), internal)
  struct ArrangementOptimiserConfig {
    using Self = ArrangementOptimiserConfig;
    ROOFER_CONFIG_MEMBERS(ROOFER_ARRANGEMENT_OPTIMISER_FIELDS)

    [[nodiscard]] constexpr float data_weight() const {
      return complexity_factor;
    }
    [[nodiscard]] constexpr float smoothness_weight() const {
      return 1.0F - complexity_factor;
    }
  };
#undef ROOFER_ARRANGEMENT_OPTIMISER_FIELDS

  struct ArrangementOptimiserInterface {
    // add_input("arrangement", typeid(Arrangement_2));
    // add_input("heightfield", typeid(RasterTools::Raster));
    // add_input("pts_per_roofplane", typeid(IndexedPlanesWithPoints ));
    // add_input("ground_pts_per_roofplane", typeid(IndexedPlanesWithPoints ));
    // add_output("arrangement", typeid(Arrangement_2));
    // add_vector_output("groundparts", typeid(LinearRing));

    // Optional: point-evidence adjacency between roof planes (plane id ->
    // neighbour plane id -> number of neighbouring point pairs), as computed
    // by the PlaneDetector. When set, the intersection discount only applies
    // to plane pairs that are adjacent in the point cloud.
    const std::map<size_t, std::map<size_t, size_t>>* plane_adjacencies =
        nullptr;

    virtual ~ArrangementOptimiserInterface() = default;
    virtual void compute(
        Arrangement_2& arrangement, const RasterTools::Raster& heightfield,
        const IndexedPlanesWithPoints& roof_planes,
        const IndexedPlanesWithPoints& ground_planes,
        ArrangementOptimiserConfig config = ArrangementOptimiserConfig()) = 0;
  };

  std::vector<LinearRing> arr2polygons(Arrangement_2& arr);

  std::unique_ptr<ArrangementOptimiserInterface> createArrangementOptimiser();
}  // namespace roofer::reconstruction
