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

#include <roofer/reconstruction/LineRegulariser.hpp>
#include <roofer/reconstruction/LineRegulariserBase.hpp>

#include <map>
#include <limits>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <numeric>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#ifdef RF_USE_OSQP
#include <osqp/osqp.h>

#include <CGAL/Shape_regularization/regularize_segments.h>
#include <tuple>
#endif

namespace roofer::reconstruction {

  namespace globalreg {
    using GK = CGAL::Exact_predicates_inexact_constructions_kernel;
    using GPoint = GK::Point_2;
    using GSegment = GK::Segment_2;
    using GSegments = std::vector<GSegment>;

#ifdef RF_USE_OSQP
    namespace SR = CGAL::Shape_regularization::Segments;
    using NeighborQuery = SR::Delaunay_neighbor_query_2<GK, GSegments>;

    // Model of CGAL's QuadraticProgramTraits for the OSQP 0.6 C API
    // (ConanCenter ships OSQP 0.6; CGAL 6's own wrapper targets OSQP 1.x).
    // Solves  min 1/2 x'Px + q'x  s.t.  l <= Ax <= u.
    class QP {
      using Triplet = std::tuple<std::size_t, std::size_t, double>;
      std::size_t n = 0, m = 0;
      std::vector<Triplet> P_vec, A_vec;
      std::vector<double> q_vec, l_vec, u_vec;

      void to_csc(const std::vector<Triplet>& tr, std::vector<c_float>& x,
                  std::vector<c_int>& i, std::vector<c_int>& p) const {
        const std::size_t nnz = tr.size();
        x.assign(nnz, 0);
        i.assign(nnz, 0);
        p.assign(n + 1, 0);
        for (const auto& t : tr) p[std::get<1>(t) + 1]++;
        for (std::size_t j = 0; j < n; ++j) p[j + 1] += p[j];
        std::vector<c_int> next(p.begin(), p.end() - 1);
        // stable fill keeps row indices sorted within a column if the
        // triplets were added in row order; sort to be safe
        std::vector<Triplet> sorted = tr;
        std::sort(sorted.begin(), sorted.end(),
                  [](const auto& a, const auto& b) {
                    return std::tie(std::get<1>(a), std::get<0>(a)) <
                           std::tie(std::get<1>(b), std::get<0>(b));
                  });
        for (const auto& t : sorted) {
          const c_int dest = next[std::get<1>(t)]++;
          i[dest] = static_cast<c_int>(std::get<0>(t));
          x[dest] = static_cast<c_float>(std::get<2>(t));
        }
      }

     public:
      void resize(std::size_t new_n, std::size_t new_m = 0) {
        n = new_n;
        m = new_m;
        P_vec.clear();
        A_vec.clear();
        q_vec.assign(n, 0);
        l_vec.assign(m, 0);
        u_vec.assign(m, 0);
      }
      void set_P(std::size_t i, std::size_t j, double v) {
        if (j < i) return;  // upper triangle only
        if (j >= n) n = j + 1;
        P_vec.emplace_back(i, j, v);
      }
      void set_q(std::size_t i, double v) {
        if (i >= q_vec.size()) q_vec.resize(i + 1, 0);
        q_vec[i] = v;
      }
      void set_r(double) {}
      void set_A(std::size_t i, std::size_t j, double v) {
        if (i >= m) m = i + 1;
        if (j >= n) n = j + 1;
        A_vec.emplace_back(i, j, v);
      }
      void set_l(std::size_t i, double v) {
        if (i >= l_vec.size()) l_vec.resize(i + 1, 0);
        l_vec[i] = v;
      }
      void set_u(std::size_t i, double v) {
        if (i >= u_vec.size()) u_vec.resize(i + 1, 0);
        u_vec[i] = v;
      }
      template <typename OutIterator>
      bool solve(OutIterator solution) {
        q_vec.resize(n, 0);
        l_vec.resize(m, 0);
        u_vec.resize(m, 0);
        std::vector<c_float> Px, Ax;
        std::vector<c_int> Pi, Pp, Ai, Ap;
        to_csc(P_vec, Px, Pi, Pp);
        to_csc(A_vec, Ax, Ai, Ap);
        std::vector<c_float> q(q_vec.begin(), q_vec.end());
        std::vector<c_float> l(l_vec.begin(), l_vec.end());
        std::vector<c_float> u(u_vec.begin(), u_vec.end());
        for (auto& v : l) v = std::max<c_float>(v, -OSQP_INFTY);
        for (auto& v : u) v = std::min<c_float>(v, OSQP_INFTY);

        OSQPData data;
        data.n = static_cast<c_int>(n);
        data.m = static_cast<c_int>(m);
        data.P = csc_matrix(data.n, data.n, static_cast<c_int>(Px.size()),
                            Px.data(), Pi.data(), Pp.data());
        data.A = csc_matrix(data.m, data.n, static_cast<c_int>(Ax.size()),
                            Ax.data(), Ai.data(), Ap.data());
        data.q = q.data();
        data.l = l.data();
        data.u = u.data();
        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose = 0;
        settings.eps_abs = 1e-8;
        settings.eps_rel = 1e-8;
        settings.max_iter = 20000;
        settings.polish = 1;
        OSQPWorkspace* work = nullptr;
        c_int flag = osqp_setup(&work, &data, &settings);
        bool ok = false;
        if (!flag && work) {
          osqp_solve(work);
          for (std::size_t k = 0; k < n; ++k)
            *solution++ = static_cast<double>(work->solution->x[k]);
          ok = work->info->status_val == OSQP_SOLVED ||
               work->info->status_val == OSQP_SOLVED_INACCURATE;
          osqp_cleanup(work);
        } else {
          for (std::size_t k = 0; k < n; ++k) *solution++ = 0.0;
        }
        c_free(data.P);
        c_free(data.A);
        return ok;
      }
    };

    // A CGAL regularisation type with a bound per segment. In the CGAL QP the
    // bound scales the cost of moving a segment by 1/bound^2, so a small bound
    // keeps that segment (a plane intersection line) where it is while
    // boundary segments are pulled onto it.
    template <typename Base>
    struct PerSegmentBound : public Base {
      std::vector<GK::FT> bounds;
      template <typename NP>
      PerSegmentBound(GSegments& segments, std::vector<GK::FT> b, const NP& np)
          : Base(segments, np), bounds(std::move(b)) {}
      GK::FT bound(const std::size_t i) const { return bounds[i]; }
    };
    using AngleReg = PerSegmentBound<SR::Angle_regularization_2<GK, GSegments>>;
    using OffsetReg =
        PerSegmentBound<SR::Offset_regularization_2<GK, GSegments>>;
#endif

    struct UnionFind {
      std::vector<size_t> parent;
      explicit UnionFind(size_t n) : parent(n) {
        std::iota(parent.begin(), parent.end(), 0);
      }
      size_t find(size_t i) {
        while (parent[i] != i) i = parent[i] = parent[parent[i]];
        return i;
      }
      void join(size_t a, size_t b) { parent[find(a)] = find(b); }
    };

    inline double orientation(const GSegment& s) {
      double a = std::atan2(s.target().y() - s.source().y(),
                            s.target().x() - s.source().x());
      if (a < 0) a += std::numbers::pi;
      if (a >= std::numbers::pi) a -= std::numbers::pi;
      return a;
    }
    inline double angle_diff(double a, double b) {
      double d = std::fabs(a - b);
      return std::min(d, std::numbers::pi - d);
    }
  }  // namespace globalreg

  class LineRegulariser : public LineRegulariserInterface {
    // Global variant: angles and offsets of all segments are regularised
    // jointly with the quadratic program of Bauchet & Lafarge (CGAL
    // Shape_regularization). Afterwards only segments that ended up on the
    // same line are merged; every other segment keeps its own line, so the
    // arrangement receives an over-complete set of candidate edges and the
    // graph-cut decides which ones carry a roof boundary.
    void compute_global(const SegmentCollection& edge_segments,
                        const SegmentCollection& ints_segments,
                        const LineRegulariserConfig& cfg) {
      using namespace globalreg;
      GSegments segs;
      std::vector<bool> is_ints;
      auto add = [&](const SegmentCollection& sc, bool ints) {
        for (const auto& s : sc) {
          GPoint a(s[0][0], s[0][1]), b(s[1][0], s[1][1]);
          if (CGAL::squared_distance(a, b) < 1e-4) continue;
          segs.emplace_back(a, b);
          is_ints.push_back(ints);
        }
      };
      add(edge_segments, false);
      add(ints_segments, true);
      const size_t n = segs.size();
      if (n == 0) return;

#ifdef RF_USE_OSQP
      if (n >= 3) {
        try {
          std::vector<GK::FT> abounds(n), obounds(n);
          for (size_t i = 0; i < n; ++i) {
            const double f = is_ints[i] ? cfg.global_anchor_factor : 1.0;
            abounds[i] = cfg.global_max_angle * f;
            obounds[i] = cfg.global_max_offset * f;
          }
          GSegments original = segs;
          NeighborQuery neighbor_query(segs);
          AngleReg angle_reg(
              segs, abounds,
              CGAL::parameters::maximum_angle(GK::FT(cfg.global_max_angle)));
          QP qp_angles;
          SR::regularize_segments(segs, neighbor_query, angle_reg, qp_angles);

          std::vector<std::vector<std::size_t>> parallel_groups;
          angle_reg.parallel_groups(std::back_inserter(parallel_groups));
          OffsetReg offset_reg(
              segs, obounds,
              CGAL::parameters::maximum_offset(GK::FT(cfg.global_max_offset)));
          neighbor_query.clear();
          size_t n_groups = 0;
          for (const auto& group : parallel_groups) {
            if (group.size() < 2) continue;
            neighbor_query.add_group(group);
            offset_reg.add_group(group);
            ++n_groups;
          }
          if (n_groups > 0) {
            QP qp_offsets;
            SR::regularize_segments(segs, neighbor_query, offset_reg,
                                    qp_offsets);
          }
          // guard against solver failures that leave degenerate segments
          for (size_t i = 0; i < n; ++i) {
            if (!std::isfinite(CGAL::to_double(segs[i].source().x())) ||
                !std::isfinite(CGAL::to_double(segs[i].target().y())) ||
                CGAL::squared_distance(segs[i].source(), segs[i].target()) <
                    1e-6) {
              segs[i] = original[i];
            }
          }
        } catch (const std::exception&) {
          // keep the unregularised segments
        }
      }
#endif

      // Group segments that now lie on one line (within the tolerance).
      const double angle_tol = 0.2 * std::numbers::pi / 180.0;
      const double offset_tol = cfg.global_collinear_tolerance;
      std::vector<double> theta(n), offset(n), length(n);
      for (size_t i = 0; i < n; ++i) {
        theta[i] = orientation(segs[i]);
        length[i] = std::sqrt(CGAL::to_double(segs[i].squared_length()));
      }
      UnionFind uf(n);
      for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
          if (angle_diff(theta[i], theta[j]) > angle_tol) continue;
          // offset of j's endpoints from i's supporting line
          const auto li = segs[i].supporting_line();
          const double d =
              std::max(std::sqrt(CGAL::to_double(
                           CGAL::squared_distance(li, segs[j].source()))),
                       std::sqrt(CGAL::to_double(
                           CGAL::squared_distance(li, segs[j].target()))));
          if (d > offset_tol) continue;
          // never merge two plane intersection lines (as in the legacy
          // regulariser with merge_intersection_lines = false)
          if (is_ints[i] && is_ints[j] && !cfg.merge_intersection_lines)
            continue;
          uf.join(i, j);
        }
      }
      std::map<size_t, std::vector<size_t>> groups;
      for (size_t i = 0; i < n; ++i) groups[uf.find(i)].push_back(i);

      for (const auto& [root, members] : groups) {
        // reference line: the longest intersection line if there is one,
        // else the longest segment
        size_t ref = members.front();
        for (auto i : members) {
          const bool better =
              (is_ints[i] && !is_ints[ref]) ||
              (is_ints[i] == is_ints[ref] && length[i] > length[ref]);
          if (better) ref = i;
        }
        const auto& rs = segs[ref];
        const double dx =
            CGAL::to_double(rs.target().x() - rs.source().x()) / length[ref];
        const double dy =
            CGAL::to_double(rs.target().y() - rs.source().y()) / length[ref];
        const double ox = CGAL::to_double(rs.source().x());
        const double oy = CGAL::to_double(rs.source().y());
        double tmin = std::numeric_limits<double>::max();
        double tmax = std::numeric_limits<double>::lowest();
        for (auto i : members) {
          for (const auto& p : {segs[i].source(), segs[i].target()}) {
            const double t = (CGAL::to_double(p.x()) - ox) * dx +
                             (CGAL::to_double(p.y()) - oy) * dy;
            tmin = std::min(tmin, t);
            tmax = std::max(tmax, t);
          }
        }
        tmin -= cfg.extension;
        tmax += cfg.extension;
        const double x0 = ox + tmin * dx, y0 = oy + tmin * dy;
        const double x1 = ox + tmax * dx, y1 = oy + tmax * dy;
        auto new_seg = Segment();
        new_seg[0] = {float(x0), float(y0), 0};
        new_seg[1] = {float(x1), float(y1), 0};
        regularised_edges.push_back(new_seg);
        exact_regularised_edges.push_back(
            EPECK::Segment_2(EPECK::Point_2(x0, y0), EPECK::Point_2(x1, y1)));
      }
    }

    void compute(const SegmentCollection& edge_segments,
                 const SegmentCollection& ints_segments,
                 LineRegulariserConfig cfg) override {
      if (cfg.global_regularisation) {
        compute_global(edge_segments, ints_segments, cfg);
        return;
      }
      // get clusters from line regularisation
      auto LR = linereg::LineRegulariser();
      LR.add_segments(0, edge_segments);
      LR.add_segments(2, ints_segments);
      LR.dist_threshold = cfg.distance_threshold * cfg.distance_threshold;
      LR.angle_threshold =
          cfg.angle_threshold * std::numbers::pi_v<double> / 180.0;

      LR.perform_angle_clustering();
      LR.perform_distance_clustering();

      SegmentCollection edges_out_;
      vec1i priorities, angle_cluster_ids, dist_cluster_ids;
      // we should iterate of the distance clusters and output one segment per
      // cluster
      for (auto& line : LR.lines) {
        linereg::Segment_2 segment;
        segment = line.segment;
        auto new_seg = Segment();
        new_seg[0] = {float(CGAL::to_double(segment.source().x())),
                      float(CGAL::to_double(segment.source().y())), 0};
        new_seg[1] = {float(CGAL::to_double(segment.target().x())),
                      float(CGAL::to_double(segment.target().y())), 0};
        edges_out_.push_back(new_seg);
        priorities.push_back(line.priority);
        priorities.push_back(line.priority);
        angle_cluster_ids.push_back(line.angle_cluster_id);
        angle_cluster_ids.push_back(line.angle_cluster_id);
        dist_cluster_ids.push_back(line.dist_cluster_id);
        dist_cluster_ids.push_back(line.dist_cluster_id);
      }
      for (auto& dclust : LR.dist_clusters) {
        size_t dclust_size = dclust->lines.size();
        linereg::Segment_2 segment;
        // get lines with highest priority
        size_t max_priority = 0;
        for (auto line : dclust->lines) {
          if (line->priority > max_priority) max_priority = line->priority;
        }
        std::vector<linereg::linetype*> prio_lines;
        for (auto line : dclust->lines) {
          if (line->priority == max_priority) {
            prio_lines.push_back(line);
          }
        }
        // TODO: split clusters using common planes if intersection segment(s)
        // are present?
        // TODO: compute distance clusters with 3D dists?
        // TODO: output exact segments? quick solve of spikes?
        // TODO: performance optimise clustering algo
        // //skip if cluster only contains fp segments
        // if(max_priority==1 && (dclust_size == prio_lines.size())) continue;
        if (!cfg.merge_intersection_lines &&
            (max_priority == 2 && (prio_lines.size() > 1))) {
          std::vector<linereg::linetype*> other_lines;
          for (auto line : dclust->lines) {
            if (line->priority != 2) {
              other_lines.push_back(line);
            }
          }
          for (auto line : prio_lines) {
            double mean_angle = line->angle;
            auto centroid = line->midpoint;
            segment = linereg::calc_segment(centroid, mean_angle, other_lines,
                                            cfg.extension);
            auto new_seg = Segment();
            new_seg[0] = {float(CGAL::to_double(segment.source().x())),
                          float(CGAL::to_double(segment.source().y())), 0};
            new_seg[1] = {float(CGAL::to_double(segment.target().x())),
                          float(CGAL::to_double(segment.target().y())), 0};
            regularised_edges.push_back(new_seg);
            exact_regularised_edges.push_back(segment);
          }
        } else {
          // compute mean line with small extensions on both ends
          double mean_angle = calc_mean_angle(prio_lines);
          auto centroid = calc_centroid(prio_lines);
          segment = linereg::calc_segment(centroid, mean_angle, dclust->lines,
                                          cfg.extension);
          auto new_seg = Segment();
          new_seg[0] = {float(CGAL::to_double(segment.source().x())),
                        float(CGAL::to_double(segment.source().y())), 0};
          new_seg[1] = {float(CGAL::to_double(segment.target().x())),
                        float(CGAL::to_double(segment.target().y())), 0};
          regularised_edges.push_back(new_seg);
          exact_regularised_edges.push_back(segment);
        }
      }
      // output("angle_cluster_id").set(angle_cluster_ids);
      // output("dist_cluster_id").set(dist_cluster_ids);
      // output("priorities").set(priorities);
      // output("edges_out_").set(edges_out_);
      // output("n_angle_clusters").set(int(LR.angle_clusters.size()));
      // output("edges_out").set(new_segments);
      // output("rings_out").set(new_rings);
      // output("footprint_out").set(new_fp);
    }
  };

  std::unique_ptr<LineRegulariserInterface> createLineRegulariser() {
    return std::make_unique<LineRegulariser>();
  };

}  // namespace roofer::reconstruction
