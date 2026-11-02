// Copyright (c) 2018-2026 TU Delft 3D geoinformation group, Ravi Peters (3DGI),
// and Balazs Dukai (3DGI)

// This file is part of roofer (https://github.com/3DBAG/roofer)

// roofer is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version. See <https://www.gnu.org/licenses/>.

#pragma once
#include <algorithm>
#include <array>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <cmath>
#include <cstddef>
#include <vector>

namespace roofer::reconstruction {
  // ---------------------------------------------------------------------
  // Alpha-expansion with label costs (Delong, Osokin, Isack & Boykov, IJCV
  // 2012) and label-pair dependent edge costs.
  //
  // E(l) = sum_p D_p(l_p) + sum_(p,q) w_pq c_pq(l_p, l_q) + sum_L h_L [L used]
  //
  // c_pq(a, b) = 1 - discount * max(0, 1 - d_pq(a, b) / d0) for two roof
  // planes a != b, where d_pq is the largest distance of the edge's end
  // points to the 2D projection of the intersection line of planes a and b.
  // Edges that lie on the ridge/valley the two planes imply are cheaper.
  // c is 1 for pairs with a ground label and 0 for a == b.
  //
  // c_pq is not a metric, so an expansion move can be non-submodular; such
  // pairwise terms are truncated (Rother et al. 2005) and every move is
  // accepted only if the exact energy decreases, so the energy never goes
  // up. The label cost of alpha itself is checked the same way.
  // ---------------------------------------------------------------------
  namespace labelcost {
    using Traits = boost::adjacency_list_traits<boost::vecS, boost::vecS,
                                                boost::directedS>;
    using Graph = boost::adjacency_list<
        boost::vecS, boost::vecS, boost::directedS,
        boost::property<
            boost::vertex_color_t, boost::default_color_type,
            boost::property<boost::vertex_distance_t, double,
                            boost::property<boost::vertex_predecessor_t,
                                            Traits::edge_descriptor>>>,
        boost::property<
            boost::edge_capacity_t, double,
            boost::property<boost::edge_residual_capacity_t, double,
                            boost::property<boost::edge_reverse_t,
                                            Traits::edge_descriptor>>>>;

    constexpr double kInf = 1e15;

    struct FlowGraph {
      Graph g;
      size_t s, t;
      explicit FlowGraph(size_t n) : g(n + 2), s(n), t(n + 1) {}
      size_t add_node() { return boost::add_vertex(g); }
      void add_edge(size_t u, size_t v, double cuv, double cvu) {
        auto e1 = boost::add_edge(u, v, g).first;
        auto e2 = boost::add_edge(v, u, g).first;
        boost::put(boost::edge_capacity, g, e1, cuv);
        boost::put(boost::edge_capacity, g, e2, cvu);
        boost::put(boost::edge_reverse, g, e1, e2);
        boost::put(boost::edge_reverse, g, e2, e1);
      }
      // cost_x1: paid if the node ends on the sink side (x = 1, takes alpha)
      // cost_x0: paid if it ends on the source side (x = 0, keeps its label)
      void add_unary(size_t v, double cost_x1, double cost_x0) {
        if (cost_x1 > 0) add_edge(s, v, cost_x1, 0);
        if (cost_x0 > 0) add_edge(v, t, cost_x0, 0);
      }
      double max_flow() { return boost::boykov_kolmogorov_max_flow(g, s, t); }
      bool takes_alpha(size_t v) const {
        return boost::get(boost::vertex_color, g, v) !=
               boost::color_traits<boost::default_color_type>::black();
      }
    };

    struct Edge {
      size_t p, q;
      double w;               // (1 - lambda) * length
      double x0, y0, x1, y1;  // end points
    };

    struct Solver {
      std::vector<std::vector<double>> D;  // D[p][l]
      std::vector<Edge> edges;
      std::vector<double> h;  // label costs
      size_t n_roof;          // labels < n_roof are roof
      double discount, d0;
      // per roof label: z = gx * x + gy * y + g0
      std::vector<std::array<double, 3>> grad;
      size_t L;
      // cache of c_e(a, b) per edge is too big; cache intersection lines
      std::vector<std::array<double, 4>> lines;  // L x L, [A, B, C, norm]
      std::vector<char> line_set;
      // optional n_roof x n_roof mask: discount only these plane pairs
      std::vector<char> pair_ok;

      Solver(const std::vector<std::vector<double>>& D_,
             const std::vector<Edge>& edges_, std::vector<double> h_,
             size_t n_roof_, double discount_, double d0_,
             std::vector<std::array<double, 3>> planes)
          : D(D_),
            edges(edges_),
            h(std::move(h_)),
            n_roof(n_roof_),
            discount(discount_),
            d0(d0_),
            grad(std::move(planes)),
            L(grad.size()) {
        if (discount > 0) {
          lines.resize(n_roof * n_roof);
          line_set.assign(n_roof * n_roof, 0);
        }
      }

      double cfactor(const Edge& e, size_t a, size_t b) {
        if (a == b) return 0.0;
        if (discount <= 0 || a >= n_roof || b >= n_roof) return 1.0;
        const size_t ia = std::min(a, b), ib = std::max(a, b);
        if (!pair_ok.empty() && !pair_ok[ia * n_roof + ib]) return 1.0;
        auto& ln = lines[ia * n_roof + ib];
        if (!line_set[ia * n_roof + ib]) {
          const double A = grad[ia][0] - grad[ib][0];
          const double B = grad[ia][1] - grad[ib][1];
          const double C = grad[ia][2] - grad[ib][2];
          ln = {A, B, C, std::sqrt(A * A + B * B)};
          line_set[ia * n_roof + ib] = 1;
        }
        // planes (nearly) parallel: no intersection line to agree with
        if (ln[3] < 1e-3) return 1.0;
        const double d =
            std::max(std::fabs(ln[0] * e.x0 + ln[1] * e.y0 + ln[2]) / ln[3],
                     std::fabs(ln[0] * e.x1 + ln[1] * e.y1 + ln[2]) / ln[3]);
        return 1.0 - discount * std::max(0.0, 1.0 - d / d0);
      }
      double V(const Edge& e, size_t a, size_t b) {
        return e.w * cfactor(e, a, b);
      }

      double energy(const std::vector<size_t>& lab) {
        double E = 0;
        for (size_t p = 0; p < lab.size(); ++p) E += D[p][lab[p]];
        for (const auto& e : edges) E += V(e, lab[e.p], lab[e.q]);
        std::vector<char> used(L, 0);
        for (auto l : lab) used[l] = 1;
        for (size_t l = 0; l < L; ++l)
          if (used[l]) E += h[l];
        return E;
      }

      std::vector<size_t> expand(const std::vector<size_t>& lab, size_t alpha) {
        const size_t F = lab.size();
        FlowGraph fg(F);
        std::vector<double> u0(F, 0.0), u1(F, 0.0);
        for (size_t p = 0; p < F; ++p) {
          u1[p] += D[p][alpha];
          if (lab[p] == alpha)
            u0[p] = kInf;
          else
            u0[p] += D[p][lab[p]];
        }
        for (const auto& e : edges) {
          const size_t a = lab[e.p], b = lab[e.q];
          const double A = V(e, a, b);
          const double B = V(e, a, alpha);
          const double C = V(e, alpha, b);
          // E(x_p, x_q) = A + (C - A) x_p - C x_q + w (1 - x_p) x_q
          if (u0[e.p] < kInf) u1[e.p] += C - A;
          if (u0[e.q] < kInf) u1[e.q] -= C;
          double w = B + C - A;
          if (w > 1e-12) fg.add_edge(e.p, e.q, w, 0);
          // w < 0: non-submodular, truncated to 0 (move is verified later)
        }
        for (size_t p = 0; p < F; ++p) {
          if (u0[p] >= kInf) {
            fg.add_unary(p, 0, kInf);
          } else {
            const double m = std::min(u0[p], u1[p]);
            fg.add_unary(p, u1[p] - m, u0[p] - m);
          }
        }
        // label costs of the labels that could disappear in this move
        std::vector<std::vector<size_t>> members(L);
        for (size_t p = 0; p < F; ++p)
          if (lab[p] != alpha) members[lab[p]].push_back(p);
        for (size_t l = 0; l < L; ++l) {
          if (members[l].empty() || h[l] <= 0) continue;
          const size_t y = fg.add_node();
          fg.add_edge(y, fg.t, h[l], 0);
          for (auto p : members[l]) fg.add_edge(p, y, h[l], 0);
        }
        fg.max_flow();
        std::vector<size_t> out = lab;
        for (size_t p = 0; p < F; ++p)
          if (fg.takes_alpha(p)) out[p] = alpha;
        return out;
      }

      std::vector<size_t> solve(int max_cycles, size_t& n_moves,
                                int& n_cycles) {
        const size_t F = D.size();
        std::vector<size_t> lab(F, 0);
        for (size_t p = 0; p < F; ++p) {
          lab[p] = std::min_element(D[p].begin(), D[p].end()) - D[p].begin();
        }
        double E = energy(lab);
        n_moves = 0;
        for (n_cycles = 0; n_cycles < max_cycles; ++n_cycles) {
          bool improved = false;
          for (size_t alpha = 0; alpha < L; ++alpha) {
            auto cand = expand(lab, alpha);
            if (cand == lab) continue;
            const double Ec = energy(cand);
            if (Ec < E - 1e-9 * std::max(1.0, std::fabs(E))) {
              lab.swap(cand);
              E = Ec;
              improved = true;
              ++n_moves;
            }
          }
          if (!improved) break;
        }
        return lab;
      }
    };
  }  // namespace labelcost
}  // namespace roofer::reconstruction
