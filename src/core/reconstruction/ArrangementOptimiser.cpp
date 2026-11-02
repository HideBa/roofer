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

#include <CGAL/Arr_face_index_map.h>
#include <CGAL/boost/graph/Alpha_expansion_MaxFlow_tag.h>
#include <CGAL/boost/graph/alpha_expansion_graphcut.h>
#include <CGAL/graph_traits_dual_arrangement_2.h>
#include <CGAL/property_map.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <roofer/reconstruction/ArrangementBase.hpp>
#include <roofer/reconstruction/LabelCostExpansion.hpp>
#include <roofer/reconstruction/ArrangementOptimiser.hpp>
#include <vector>

namespace roofer::reconstruction {

  class FootprintGraph {
   public:
    typedef typename Arrangement_2::Face_handle vertex_descriptor;
    typedef typename Arrangement_2::Halfedge_handle edge_descriptor;
    typedef std::vector<vertex_descriptor> vertex_container;
    typedef std::vector<edge_descriptor> edge_container;
    typedef vertex_container::const_iterator vertex_iterator;
    typedef edge_container::const_iterator edge_iterator;

   private:
    vertex_container vertices_;
    edge_container edges_;

   public:
    FootprintGraph(vertex_container& vc, edge_container& ec)
        : vertices_(vc), edges_(ec) {}

    friend vertex_descriptor source(edge_descriptor& e,
                                    const FootprintGraph& g) {
      return e->face();
    };
    friend vertex_descriptor target(edge_descriptor& e,
                                    const FootprintGraph& g) {
      return e->twin()->face();
    };
    friend vertex_container::size_type num_vertices(const FootprintGraph& g) {
      return g.vertices_.size();
    };
    friend edge_container::size_type num_edges(const FootprintGraph& g) {
      return g.edges_.size();
    };
    friend std::pair<vertex_iterator, vertex_iterator> vertices(
        const FootprintGraph& g) {
      return std::make_pair(g.vertices_.begin(), g.vertices_.end());
    };
    friend std::pair<edge_iterator, edge_iterator> edges(
        const FootprintGraph& g) {
      return std::make_pair(g.edges_.begin(), g.edges_.end());
    };
  };

  // A property map that reads/writes the information to/from the extended
  // face.
  class Vertex_label_cost_property_map {
   public:
    typedef typename Arrangement_2::Face_handle Face_handle;
    // Boost property type definitions.
    typedef boost::read_write_property_map_tag category;
    typedef std::vector<double> value_type;
    typedef value_type& reference;
    typedef Face_handle key_type;
    // The get function is required by the property map concept.
    friend reference get(const Vertex_label_cost_property_map&, key_type key) {
      return key->data().vertex_label_cost;
    }
    // The put function is required by the property map concept.
    friend void put(const Vertex_label_cost_property_map&, key_type key,
                    value_type val) {
      key->data().vertex_label_cost = val;
    }
  };
  // A property map that reads/writes the information to/from the extended
  // face.
  class Vertex_label_property_map {
   public:
    typedef typename Arrangement_2::Face_handle Face_handle;
    // Boost property type definitions.
    typedef boost::read_write_property_map_tag category;
    typedef size_t value_type;
    typedef value_type& reference;
    typedef Face_handle key_type;
    // The get function is required by the property map concept.
    friend reference get(const Vertex_label_property_map&, key_type key) {
      return key->data().label;
    }
    // The put function is required by the property map concept.
    friend void put(const Vertex_label_property_map&, key_type key,
                    value_type val) {
      key->data().label = val;
    }
  };
  // A property map that reads/writes the information to/from the extended
  // face.
  class Vertex_index_map {
   public:
    typedef typename Arrangement_2::Face_handle Face_handle;
    // Boost property type definitions.
    typedef boost::readable_property_map_tag category;
    typedef size_t value_type;
    typedef value_type& reference;
    typedef Face_handle key_type;
    // The get function is required by the property map concept.
    friend reference get(const Vertex_index_map&, key_type key) {
      return key->data().v_index;
    }
  };
  // A property map that reads/writes the information to/from the extended
  // edge.
  class Edge_weight_property_map {
   public:
    typedef typename Arrangement_2::Halfedge_handle Halfedge_handle;
    // Boost property type definitions.
    typedef boost::read_write_property_map_tag category;
    typedef double value_type;
    typedef value_type& reference;
    typedef Halfedge_handle key_type;
    // The get function is required by the property map concept.
    friend reference get(const Edge_weight_property_map&, key_type key) {
      return key->data().edge_weight;
    }
    // The put function is required by the property map concept.
    friend void put(const Edge_weight_property_map&, key_type key,
                    value_type val) {
      key->data().edge_weight = val;
    }
  };

}  // namespace roofer::reconstruction

namespace boost {
  template <>
  struct graph_traits<roofer::reconstruction::FootprintGraph> {
    typedef typename roofer::Arrangement_2::Face_handle vertex_descriptor;
    typedef typename roofer::Arrangement_2::Halfedge_handle edge_descriptor;
    typedef boost::disallow_parallel_edge_tag edge_parallel_category;
    typedef boost::edge_list_graph_tag traversal_category;
    typedef boost::directed_tag directed_category;
    typedef roofer::reconstruction::FootprintGraph::vertex_container::size_type
        vertices_size_type;
    typedef roofer::reconstruction::FootprintGraph::edge_container::size_type
        edges_size_type;
  };
}  // namespace boost

namespace roofer::reconstruction {

  inline double edge_length(const Arrangement_2::Halfedge_handle& e) {
    const double x1 = CGAL::to_double(e->source()->point().x());
    const double y1 = CGAL::to_double(e->source()->point().y());
    const double x2 = CGAL::to_double(e->target()->point().x());
    const double y2 = CGAL::to_double(e->target()->point().y());
    const double diff_x = x2 - x1;
    const double diff_y = y2 - y1;
    return std::sqrt(diff_x * diff_x + diff_y * diff_y);
  }

  inline double rmse_plane_points(Plane& plane, std::vector<Point>& points) {
    double dist_sum = 0;
    for (auto& p : points) {
      dist_sum += CGAL::squared_distance(plane, p);
    }
    return CGAL::sqrt(dist_sum / points.size());
  }
  inline double volume_to_plane(Plane& plane, vec3f& points) {
    double volume = 0;
    for (auto& p : points) {
      volume += std::abs(p[2] - (-plane.a() / plane.c() * p[0] -
                                 plane.b() / plane.c() * p[1] -
                                 plane.d() / plane.c()));
    }
    return volume;
  }

  struct InFootprintFaceFilter {
    InFootprintFaceFilter() {}

    template <typename Face_handle>
    bool operator()(const Face_handle& f) const {
      return f->data().in_footprint;
    }
  };
  struct InFootprintEdgeFilter {
    InFootprintEdgeFilter() {}

    template <typename Edge_handle>
    bool operator()(const Edge_handle& e) const {
      bool infp_l = e->twin()->face()->data().in_footprint;
      bool infp_r = e->face()->data().in_footprint;
      return infp_l && infp_r;
    }
  };

  class ArrangementOptimiser : public ArrangementOptimiserInterface {
    void compute(Arrangement_2& arr, const RasterTools::Raster& heightfield,
                 const IndexedPlanesWithPoints& roof_planes,
                 const IndexedPlanesWithPoints& ground_planes,
                 ArrangementOptimiserConfig cfg) override {
      std::vector<std::tuple<Plane, size_t>>
          points_per_plane;  // plane, points, seg_id
      for (const auto& [plane_id, plane_pts] : roof_planes) {
        if (plane_id < 1) continue;  // ignore unclassified points
        points_per_plane.push_back(std::make_tuple(plane_pts.first, plane_id));
      }
      size_t roofplane_cnt = points_per_plane.size();
      if (cfg.use_ground) {
        for (const auto& [plane_id, plane_pts] : ground_planes) {
          if (plane_id < 1) continue;  // ignore unclassified points
          size_t plane_id_ = (roofplane_cnt - 1) + plane_id;
          points_per_plane.push_back(
              std::make_tuple(plane_pts.first, plane_id_));
        }
      }

      // compute vertex_label_cost (data term)
      // 1 compute for each face the error to each plane
      double max_cost = 0;
      size_t face_i = 0;
      size_t label = 0;
      double cell_area = heightfield.cellSize_ * heightfield.cellSize_;
      std::vector<Face_handle> faces;
      for (auto face : arr.face_handles()) {
        if (face->data().in_footprint) {
          vec2f polygon;
          arrangementface_to_polygon(face, polygon);
          auto height_points = heightfield.rasterise_polygon(polygon, false);

          for (auto& [plane, plane_id] : points_per_plane) {
            double volume = cfg.data_weight() * cell_area *
                            volume_to_plane(plane, height_points);
            face->data().vertex_label_cost.push_back(volume);
            max_cost = std::max(max_cost, volume);
          }
          face->data().v_index = face_i++;
          faces.push_back(face);
          if (cfg.preset_labels)
            face->data().label = label;  // assign an initial label
          // also compute average elevation for all inliers (needed for LoD1.3
          // later)
        }
        ++label;
      }
      // normalise
      if (cfg.normalise) {
        for (auto face : faces) {
          for (auto& c : face->data().vertex_label_cost) {
            c = (c / max_cost);
          }
        }
      }

      // std::cerr << "  Graph-cut with " << faces.size() << " faces and " <<
      // points_per_plane.size() << " plane labels\n"; don't bother if there are
      // no planes or faces
      if (roofplane_cnt == 0 || faces.size() == 0) {
        arr_dissolve_fp(arr, true, true);
        // output("arrangement").set(arr);
        return;
      }

      // compute edge_weights (smoothness term)
      double max_weight = 0;
      std::vector<Halfedge_handle> edges;
      for (auto edge : arr.edge_handles()) {
        bool fp_u = edge->twin()->face()->data().in_footprint;
        bool fp_l = edge->face()->data().in_footprint;
        if (fp_u && fp_l) {  // only edges with both neighbour faces inside the
                             // footprint
          double l = cfg.smoothness_weight() * edge_length(edge);
          edge->data().edge_weight = l;
          edge->twin()->data().edge_weight = l;
          max_weight = std::max(max_weight, l);
          edges.push_back(edge);
          edges.push_back(edge->twin());
        }
      }
      // normalise
      if (cfg.normalise) {
        for (auto edge : edges) {
          bool fp_l = edge->face()->data().in_footprint;
          double n_w = edge->data().edge_weight / max_weight;
          edge->data().edge_weight = n_w;
        }
      }

      const bool use_label_cost_solver = cfg.label_cost_solver ||
                                         cfg.label_cost > 0 ||
                                         cfg.intersection_discount > 0;
      // ROOFER_LABELCOST_DEBUG=1 prints the energy reached by the solver
      // that ran (both evaluated with the label-cost energy) to stderr.
      static const bool debug_energy =
          std::getenv("ROOFER_LABELCOST_DEBUG") != nullptr;
      std::unique_ptr<labelcost::Solver> debug_solver;
      if (use_label_cost_solver || debug_energy) {
        std::vector<std::vector<double>> D(faces.size());
        for (size_t p = 0; p < faces.size(); ++p)
          D[p] = faces[p]->data().vertex_label_cost;
        std::vector<labelcost::Edge> lc_edges;
        for (auto edge : arr.edge_handles()) {
          auto f1 = edge->face(), f2 = edge->twin()->face();
          if (!f1->data().in_footprint || !f2->data().in_footprint) continue;
          if (f1 == f2) continue;
          labelcost::Edge e;
          e.p = f1->data().v_index;
          e.q = f2->data().v_index;
          e.w = edge->data().edge_weight;
          e.x0 = CGAL::to_double(edge->source()->point().x());
          e.y0 = CGAL::to_double(edge->source()->point().y());
          e.x1 = CGAL::to_double(edge->target()->point().x());
          e.y1 = CGAL::to_double(edge->target()->point().y());
          lc_edges.push_back(e);
        }
        std::vector<std::array<double, 3>> planes;
        std::vector<double> h;
        for (size_t l = 0; l < points_per_plane.size(); ++l) {
          const auto& pl = std::get<0>(points_per_plane[l]);
          const double c = pl.c();
          if (std::fabs(c) < 1e-9)
            planes.push_back({0, 0, 0});
          else
            planes.push_back({-pl.a() / c, -pl.b() / c, -pl.d() / c});
          h.push_back(l < roofplane_cnt ? cfg.data_weight() * cfg.label_cost
                                        : 0.0);
        }
        labelcost::Solver solver(D, lc_edges, h, roofplane_cnt,
                                 cfg.intersection_discount,
                                 cfg.intersection_distance, planes);
        if (plane_adjacencies && cfg.intersection_discount > 0) {
          solver.pair_ok.assign(roofplane_cnt * roofplane_cnt, 0);
          for (size_t a = 0; a < roofplane_cnt; ++a) {
            for (size_t b = a + 1; b < roofplane_cnt; ++b) {
              const size_t ida = std::get<1>(points_per_plane[a]);
              const size_t idb = std::get<1>(points_per_plane[b]);
              size_t cnt = 0;
              for (auto [x, y] : {std::pair{ida, idb}, std::pair{idb, ida}}) {
                auto it = plane_adjacencies->find(x);
                if (it == plane_adjacencies->end()) continue;
                auto jt = it->second.find(y);
                if (jt != it->second.end()) cnt = std::max(cnt, jt->second);
              }
              if (cnt >= size_t(cfg.intersection_min_adjacency))
                solver.pair_ok[a * roofplane_cnt + b] = 1;
            }
          }
        }
        size_t n_moves = 0;
        int n_cycles = 0;
        if (use_label_cost_solver) {
          auto lab = solver.solve(cfg.max_expansion_cycles, n_moves, n_cycles);
          for (size_t p = 0; p < faces.size(); ++p)
            faces[p]->data().label = lab[p];
          if (debug_energy) {
            std::vector<char> used(planes.size(), 0);
            for (auto l : lab) used[l] = 1;
            std::cerr << "[labelcost] solver=expansion faces=" << faces.size()
                      << " labels=" << planes.size()
                      << " used=" << std::count(used.begin(), used.end(), 1)
                      << " cycles=" << n_cycles << " moves=" << n_moves
                      << " energy=" << solver.energy(lab) << "\n";
          }
        } else {
          // debug only: run Roofer's CGAL graph-cut below, then report its
          // energy under the same energy function
          debug_solver = std::make_unique<labelcost::Solver>(std::move(solver));
        }
      }
      if (!use_label_cost_solver) {
        FootprintGraph graph(faces, edges);

        // assign initial labels?
        // auto graph = Dual_arrangement(arr);
        // InFootprintFaceFilter face_filter;
        // InFootprintEdgeFilter edge_filter;
        // boost::filtered_graph<Dual_arrangement, InFootprintEdgeFilter,
        // InFootprintFaceFilter >
        //   filtered_graph(graph, edge_filter, face_filter);

        // boost::adjacency_list<> g(N);

        double result;

        if (cfg.graph_cut_impl == 0) {
          result = CGAL::alpha_expansion_graphcut(
              graph, Edge_weight_property_map(),
              Vertex_label_cost_property_map(), Vertex_label_property_map(),
              CGAL::parameters::vertex_index_map(Vertex_index_map())
                  .implementation_tag(
                      CGAL::Alpha_expansion_boost_adjacency_list_tag()));
        } else if (cfg.graph_cut_impl == 1) {
          result = CGAL::alpha_expansion_graphcut(
              graph, Edge_weight_property_map(),
              Vertex_label_cost_property_map(), Vertex_label_property_map(),
              CGAL::parameters::vertex_index_map(Vertex_index_map())
                  .implementation_tag(
                      CGAL::Alpha_expansion_boost_compressed_sparse_row_tag()));
        } else if (cfg.graph_cut_impl == 2) {
          result = CGAL::alpha_expansion_graphcut(
              graph, Edge_weight_property_map(),
              Vertex_label_cost_property_map(), Vertex_label_property_map(),
              CGAL::parameters::vertex_index_map(Vertex_index_map())
                  .implementation_tag(CGAL::Alpha_expansion_MaxFlow_tag()));
        }

        if (debug_solver) {
          std::vector<size_t> lab(faces.size());
          for (size_t p = 0; p < faces.size(); ++p)
            lab[p] = faces[p]->data().label;
          std::vector<char> used(points_per_plane.size(), 0);
          for (auto l : lab) used[l] = 1;
          std::cerr << "[labelcost] solver=cgal faces=" << faces.size()
                    << " labels=" << points_per_plane.size()
                    << " used=" << std::count(used.begin(), used.end(), 1)
                    << " energy=" << debug_solver->energy(lab) << "\n";
        }
      }  // end CGAL alpha-expansion

      // store ground parts
      // std::vector<LinearRing> groundparts;

      //  assign planes from label map to arrangement faces
      for (auto& face : faces) {
        size_t i = face->data().label;
        face->data().plane = std::get<0>(points_per_plane[i]);
        face->data().segid = std::get<1>(points_per_plane[i]);
        face->data().rms_error_to_avg = face->data().vertex_label_cost[i];
        if (i >= roofplane_cnt && cfg.label_ground_outside_footprint) {
          face->data().in_footprint = false;
          face->data().is_ground = true;
          LinearRing polygon;
          arrangementface_to_polygon(face, polygon);
          // groundparts.push_back(polygon);
        }
      }
    }
  };

  std::vector<LinearRing> arr2polygons(Arrangement_2& arr) {
    std::vector<LinearRing> linear_rings;

    for (const auto face : arr.face_handles()) {
      if (face->is_fictitious() || face->is_unbounded()) continue;

      LinearRing polygon;
      arrangementface_to_polygon(face, polygon);
      linear_rings.push_back(polygon);
    }

    return linear_rings;
  }

  std::unique_ptr<ArrangementOptimiserInterface> createArrangementOptimiser() {
    return std::make_unique<ArrangementOptimiser>();
  }

}  // namespace roofer::reconstruction
