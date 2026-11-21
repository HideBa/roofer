// Tests for the label-cost alpha-expansion used by the ArrangementOptimiser
// when label_cost or intersection_discount is set.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <random>
#include <roofer/reconstruction/LabelCostExpansion.hpp>

using roofer::reconstruction::labelcost::Edge;
using roofer::reconstruction::labelcost::Solver;

namespace {
  // horizontal planes z = h (no intersection lines between them)
  std::vector<std::array<double, 3>> flat_planes(size_t L) {
    std::vector<std::array<double, 3>> p;
    for (size_t l = 0; l < L; ++l) p.push_back({0, 0, double(l)});
    return p;
  }
  double brute_force_min(Solver& s, size_t F, size_t L) {
    std::vector<size_t> lab(F, 0);
    double best = 1e300;
    while (true) {
      best = std::min(best, s.energy(lab));
      size_t k = 0;
      while (k < F && ++lab[k] == L) lab[k++] = 0;
      if (k == F) break;
    }
    return best;
  }
}  // namespace

TEST_CASE("expansion recovers the data-preferred labelling on a chain") {
  // 3 faces in a row, weak edges: each face keeps its preferred label
  std::vector<std::vector<double>> D = {{0, 5}, {5, 0}, {0, 5}};
  std::vector<Edge> E = {{0, 1, 0.1, 0, 0, 0, 1}, {1, 2, 0.1, 1, 0, 1, 1}};
  Solver s(D, E, {0, 0}, 2, 0.0, 0.5, flat_planes(2));
  size_t moves;
  int cycles;
  auto lab = s.solve(10, moves, cycles);
  CHECK(lab == std::vector<size_t>{0, 1, 0});
}

TEST_CASE("label cost removes a plane that explains too little") {
  // face 3 fits label 2 slightly better (by 0.5) than label 0
  std::vector<std::vector<double>> D = {
      {0, 9, 9}, {0, 9, 9}, {9, 0, 9}, {0.5, 9, 0}};
  std::vector<Edge> E = {{0, 1, 0.01, 0, 0, 0, 1},
                         {1, 2, 0.01, 1, 0, 1, 1},
                         {2, 3, 0.01, 2, 0, 2, 1}};
  size_t moves;
  int cycles;
  {
    Solver s(D, E, {1.0, 1.0, 1.0}, 3, 0.0, 0.5, flat_planes(3));
    auto lab = s.solve(10, moves, cycles);
    CHECK(lab[3] == 0);  // label 2 costs 1.0 > 0.5 gain
    CHECK(lab[2] == 1);  // label 1 gains 9 > 1.0
  }
  {
    Solver s(D, E, {0.1, 0.1, 0.1}, 3, 0.0, 0.5, flat_planes(3));
    auto lab = s.solve(10, moves, cycles);
    CHECK(lab[3] == 2);  // cheap label: keep it
  }
}

TEST_CASE("edges on the plane intersection line are discounted") {
  // planes z = x and z = -x intersect along x = 0
  std::vector<std::array<double, 3>> planes = {{1, 0, 0}, {-1, 0, 0}};
  std::vector<std::vector<double>> D = {{0, 0}, {0, 0}};
  Edge on{0, 1, 2.0, 0.0, -1.0, 0.0, 1.0};
  Edge off{0, 1, 2.0, 2.0, -1.0, 2.0, 1.0};
  Edge half{0, 1, 2.0, 0.25, -1.0, 0.25, 1.0};
  Solver s(D, {on}, {0, 0}, 2, 0.8, 0.5, planes);
  CHECK(s.V(on, 0, 1) == Catch::Approx(2.0 * 0.2));
  CHECK(s.V(off, 0, 1) == Catch::Approx(2.0));
  CHECK(s.V(half, 0, 1) == Catch::Approx(2.0 * (1 - 0.8 * 0.5)));
  CHECK(s.V(on, 1, 1) == 0.0);
  // ground labels (index >= n_roof) never get the discount
  Solver g(D, {on}, {0, 0}, 1, 0.8, 0.5, planes);
  CHECK(g.V(on, 0, 1) == Catch::Approx(2.0));
}

TEST_CASE("expansion never ends above the initial energy and is near optimal") {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  int optimal = 0, n = 40;
  for (int trial = 0; trial < n; ++trial) {
    const size_t F = 6, L = 3;
    std::vector<std::vector<double>> D(F, std::vector<double>(L));
    for (auto& d : D)
      for (auto& v : d) v = 3 * U(rng);
    std::vector<Edge> E;
    for (size_t p = 0; p < F; ++p)
      for (size_t q = p + 1; q < F; ++q)
        if (U(rng) < 0.5)
          E.push_back(
              {p, q, U(rng), 4 * U(rng), 4 * U(rng), 4 * U(rng), 4 * U(rng)});
    std::vector<std::array<double, 3>> planes;
    for (size_t l = 0; l < L; ++l)
      planes.push_back({U(rng) - 0.5, U(rng) - 0.5, U(rng)});
    Solver s(D, E, {0.5 * U(rng), 0.5 * U(rng), 0.5 * U(rng)}, L, 0.7, 1.0,
             planes);
    std::vector<size_t> init(F);
    for (size_t p = 0; p < F; ++p)
      init[p] = std::min_element(D[p].begin(), D[p].end()) - D[p].begin();
    size_t moves;
    int cycles;
    auto lab = s.solve(20, moves, cycles);
    const double Es = s.energy(lab), Emin = brute_force_min(s, F, L);
    CHECK(Es <= s.energy(init) + 1e-9);
    CHECK(Es >= Emin - 1e-9);
    if (Es <= Emin + 1e-9) ++optimal;
  }
  // alpha-expansion is approximate, but on tiny problems it should usually
  // find the optimum
  CHECK(optimal >= n * 3 / 4);
}
