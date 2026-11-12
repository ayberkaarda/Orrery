#include "orrery/geom/mesh_repair.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "orrery/geom/winding_number.h"

namespace orrery::geom {

namespace {

constexpr double degenerate_ratio = 1e-12;

std::uint32_t to_index(std::size_t i) {
    if (i > std::numeric_limits<std::uint32_t>::max()) {
        throw std::length_error("mesh index exceeds 32 bits");
    }
    return static_cast<std::uint32_t>(i);
}

// Maps every vertex to the first vertex (lowest index) at its position.
struct Weld {
    std::vector<std::uint32_t> map;
    std::size_t merged = 0;
};

Weld weld(const std::vector<Vector3>& vertices, double tolerance) {
    if (tolerance > 0.0) {
        // Every quantized coordinate must stay finite: an infinite cell
        // index would put unrelated vertices into one cell.
        double largest = 0.0;
        for (const Vector3& v : vertices) {
            largest = std::max({largest, std::abs(v.x), std::abs(v.y), std::abs(v.z)});
        }
        if (!std::isfinite(largest / tolerance)) {
            throw std::invalid_argument("weld tolerance too small for the coordinate range");
        }
    }
    auto key = [tolerance](const Vector3& v) {
        if (tolerance > 0.0) {
            return std::make_tuple(std::floor(v.x / tolerance), std::floor(v.y / tolerance),
                                   std::floor(v.z / tolerance));
        }
        return std::make_tuple(v.x, v.y, v.z);
    };
    std::vector<std::uint32_t> order(vertices.size());
    std::iota(order.begin(), order.end(), 0U);
    std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) {
        const auto ka = key(vertices[a]);
        const auto kb = key(vertices[b]);
        return ka != kb ? ka < kb : a < b;
    });
    Weld w;
    w.map.assign(vertices.size(), 0U);
    for (std::size_t i = 0; i < order.size();) {
        std::size_t j = i + 1;
        while (j < order.size() && key(vertices[order[j]]) == key(vertices[order[i]])) {
            ++j;
        }
        // order[i] is the lowest index of the group (ties sorted by index).
        for (std::size_t k = i; k < j; ++k) {
            w.map[order[k]] = order[i];
        }
        w.merged += j - i - 1;
        i = j;
    }
    return w;
}

bool degenerate(const std::vector<Vector3>& v, const Triangle& t) {
    if (t[0] == t[1] || t[1] == t[2] || t[0] == t[2]) {
        return true;
    }
    const Vector3 e0 = v[t[1]] - v[t[0]];
    const Vector3 e1 = v[t[2]] - v[t[1]];
    const Vector3 e2 = v[t[0]] - v[t[2]];
    const double longest = std::max({squared_norm(e0), squared_norm(e1), squared_norm(e2)});
    return norm(cross(e0, -e2)) <= degenerate_ratio * longest;
}

std::array<std::uint32_t, 3> sorted(const Triangle& t) {
    std::array<std::uint32_t, 3> s = t;
    std::sort(s.begin(), s.end());
    return s;
}

// Welded triangles without degenerate and duplicate ones (first kept).
struct Cleaned {
    std::vector<Triangle> triangles;
    std::size_t degenerate = 0;
    std::size_t duplicate = 0;
};

Cleaned clean(const std::vector<Vector3>& vertices, const std::vector<Triangle>& triangles) {
    Cleaned c;
    std::vector<std::pair<std::array<std::uint32_t, 3>, std::size_t>> keys;
    std::vector<bool> keep(triangles.size(), false);
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        if (degenerate(vertices, triangles[i])) {
            ++c.degenerate;
        } else {
            keys.emplace_back(sorted(triangles[i]), i);
        }
    }
    std::sort(keys.begin(), keys.end());
    for (std::size_t i = 0; i < keys.size(); ++i) {
        if (i > 0 && keys[i].first == keys[i - 1].first) {
            ++c.duplicate;
        } else {
            keep[keys[i].second] = true;
        }
    }
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        if (keep[i]) {
            c.triangles.push_back(triangles[i]);
        }
    }
    return c;
}

// One use of an undirected edge (lo, hi) by a triangle; `forward` when the
// triangle runs lo -> hi.
struct EdgeUse {
    std::uint32_t lo = 0;
    std::uint32_t hi = 0;
    std::uint32_t triangle = 0;
    bool forward = false;
};

std::vector<EdgeUse> edge_uses(const std::vector<Triangle>& triangles) {
    std::vector<EdgeUse> uses;
    uses.reserve(3 * triangles.size());
    for (std::size_t t = 0; t < triangles.size(); ++t) {
        for (std::size_t k = 0; k < 3; ++k) {
            const std::uint32_t a = triangles[t][k];
            const std::uint32_t b = triangles[t][(k + 1) % 3];
            uses.push_back({std::min(a, b), std::max(a, b), to_index(t), a < b});
        }
    }
    std::sort(uses.begin(), uses.end(), [](const EdgeUse& x, const EdgeUse& y) {
        return std::tie(x.lo, x.hi, x.triangle) < std::tie(y.lo, y.hi, y.triangle);
    });
    return uses;
}

// Calls f(first, last) for each group of uses of one undirected edge.
template <typename F> void for_each_edge(const std::vector<EdgeUse>& uses, F f) {
    for (std::size_t i = 0; i < uses.size();) {
        std::size_t j = i + 1;
        while (j < uses.size() && uses[j].lo == uses[i].lo && uses[j].hi == uses[i].hi) {
            ++j;
        }
        f(i, j);
        i = j;
    }
}

struct UnionFind {
    std::vector<std::uint32_t> parent;
    explicit UnionFind(std::size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0U); }
    std::uint32_t find(std::uint32_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void unite(std::uint32_t a, std::uint32_t b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent[std::max(a, b)] = std::min(a, b);
        }
    }
};

struct BoundaryLoops {
    std::vector<std::vector<std::uint32_t>> closed;
    std::size_t open = 0;
};

// Walks the directed boundary edges (in the direction of their triangle):
// chains are started at vertices without an incoming boundary edge first, so
// an open chain is counted once.
BoundaryLoops boundary_loops(const std::vector<EdgeUse>& uses) {
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges; // from -> to
    for_each_edge(uses, [&](std::size_t i, std::size_t j) {
        if (j - i == 1) {
            const EdgeUse& u = uses[i];
            edges.emplace_back(u.forward ? u.lo : u.hi, u.forward ? u.hi : u.lo);
        }
    });
    std::sort(edges.begin(), edges.end());
    std::vector<std::uint32_t> heads;
    heads.reserve(edges.size());
    for (const auto& e : edges) {
        heads.push_back(e.second);
    }
    std::sort(heads.begin(), heads.end());
    std::vector<bool> used(edges.size(), false);
    auto next_from = [&](std::uint32_t v) -> std::size_t {
        auto it = std::lower_bound(edges.begin(), edges.end(), std::make_pair(v, 0U));
        for (; it != edges.end() && it->first == v; ++it) {
            const auto index = static_cast<std::size_t>(it - edges.begin());
            if (!used[index]) {
                return index;
            }
        }
        return edges.size();
    };
    BoundaryLoops result;
    for (int pass = 0; pass < 2; ++pass) {
        for (std::size_t i = 0; i < edges.size(); ++i) {
            if (used[i]) {
                continue;
            }
            const std::uint32_t start = edges[i].first;
            if (pass == 0 && std::binary_search(heads.begin(), heads.end(), start)) {
                continue;
            }
            std::vector<std::uint32_t> path{start};
            used[i] = true;
            std::uint32_t current = edges[i].second;
            bool closed = false;
            while (true) {
                if (current == start) {
                    closed = true;
                    break;
                }
                path.push_back(current);
                const std::size_t next = next_from(current);
                if (next == edges.size()) {
                    break;
                }
                used[next] = true;
                current = edges[next].second;
            }
            if (closed) {
                result.closed.push_back(std::move(path));
            } else {
                ++result.open;
            }
        }
    }
    return result;
}

// Six times the signed volume of each component (root index), tetrahedra
// taken about `origin`.
std::vector<double> component_volumes6(const std::vector<Vector3>& vertices,
                                       const std::vector<Triangle>& triangles,
                                       const std::vector<std::uint32_t>& root, Vector3 origin) {
    std::vector<double> v6(triangles.size(), 0.0);
    for (std::size_t t = 0; t < triangles.size(); ++t) {
        const Vector3 a = vertices[triangles[t][0]] - origin;
        const Vector3 b = vertices[triangles[t][1]] - origin;
        const Vector3 c = vertices[triangles[t][2]] - origin;
        v6[root[t]] += dot(a, cross(b, c));
    }
    return v6;
}

double volume_of(const std::vector<Vector3>& vertices, const std::vector<Triangle>& triangles,
                 Vector3 origin) {
    const std::vector<std::uint32_t> same(triangles.size(), 0U);
    if (triangles.empty()) {
        return 0.0;
    }
    return component_volumes6(vertices, triangles, same, origin)[0] / 6.0;
}

struct Topology {
    std::size_t edges = 0;
    std::size_t boundary = 0;
    std::size_t non_manifold = 0;
    std::size_t inconsistent = 0;
};

Topology topology(const std::vector<EdgeUse>& uses) {
    Topology t;
    for_each_edge(uses, [&](std::size_t i, std::size_t j) {
        ++t.edges;
        if (j - i == 1) {
            ++t.boundary;
        } else if (j - i > 2) {
            ++t.non_manifold;
        } else if (uses[i].forward == uses[i + 1].forward) {
            ++t.inconsistent;
        }
    });
    return t;
}

std::vector<std::uint32_t> components_of(std::size_t triangles, const std::vector<EdgeUse>& uses) {
    UnionFind uf(triangles);
    for_each_edge(uses, [&](std::size_t i, std::size_t j) {
        for (std::size_t k = i + 1; k < j; ++k) {
            uf.unite(uses[i].triangle, uses[k].triangle);
        }
    });
    std::vector<std::uint32_t> root(triangles);
    for (std::size_t t = 0; t < triangles; ++t) {
        root[t] = uf.find(to_index(t));
    }
    return root;
}

std::size_t count_roots(const std::vector<std::uint32_t>& root) {
    std::size_t n = 0;
    for (std::size_t t = 0; t < root.size(); ++t) {
        n += root[t] == t ? 1U : 0U;
    }
    return n;
}

std::vector<Triangle> remap(const std::vector<Triangle>& triangles,
                            const std::vector<std::uint32_t>& map) {
    std::vector<Triangle> out;
    out.reserve(triangles.size());
    for (const Triangle& t : triangles) {
        out.push_back({map[t[0]], map[t[1]], map[t[2]]});
    }
    return out;
}

void flip(Triangle& t) {
    std::swap(t[1], t[2]);
}

// Winding number of a closed component (its triangles listed in `members`)
// at p: +-1 inside, 0 outside, whatever its orientation.
double component_winding(const std::vector<Vector3>& vertices,
                         const std::vector<Triangle>& triangles,
                         const std::vector<std::uint32_t>& members, const Vector3& p) {
    double sum = 0.0;
    for (std::uint32_t t : members) {
        const Triangle& tri = triangles[t];
        sum += solid_angle(p, vertices[tri[0]], vertices[tri[1]], vertices[tri[2]]);
    }
    return sum / (4.0 * std::numbers::pi);
}

// Turns each closed component so that solid lies on the inside of the
// surface: a component nested inside an even number of others (an outer
// wall or an island in a cavity) gets a positive volume, one nested inside an
// odd number (a cavity wall) a negative one. Returns the triangles flipped.
std::size_t orient_closed_components(const std::vector<Vector3>& vertices,
                                     std::vector<Triangle>& triangles) {
    const std::vector<EdgeUse> uses = edge_uses(triangles);
    const std::vector<std::uint32_t> root = components_of(triangles.size(), uses);
    std::vector<bool> open(triangles.size(), false);
    for_each_edge(uses, [&](std::size_t i, std::size_t j) {
        if (j - i == 1) {
            open[root[uses[i].triangle]] = true;
        }
    });
    TriangleMesh view;
    view.vertices = vertices;
    const std::vector<double> v6 =
        component_volumes6(vertices, triangles, root, bounds(view).centre());

    struct Shell {
        std::uint32_t root = 0;
        std::vector<std::uint32_t> members;
        Bounds box;
    };
    // A component's root is its lowest triangle index, so it is met first.
    std::vector<Shell> shells;
    std::vector<std::size_t> shell_of(triangles.size(), 0);
    for (std::size_t t = 0; t < triangles.size(); ++t) {
        if (open[root[t]]) {
            continue;
        }
        if (root[t] == t) {
            shell_of[t] = shells.size();
            const Vector3& first = vertices[triangles[t][0]];
            shells.push_back({root[t], {}, {first, first}});
        }
        Shell& s = shells[shell_of[root[t]]];
        for (std::uint32_t v : triangles[t]) {
            const Vector3& p = vertices[v];
            s.box.min = {std::min(s.box.min.x, p.x), std::min(s.box.min.y, p.y),
                         std::min(s.box.min.z, p.z)};
            s.box.max = {std::max(s.box.max.x, p.x), std::max(s.box.max.y, p.y),
                         std::max(s.box.max.z, p.z)};
        }
        s.members.push_back(to_index(t));
    }
    auto box_inside = [](const Bounds& inner, const Bounds& outer) {
        return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y &&
               outer.min.z <= inner.min.z && inner.max.x <= outer.max.x &&
               inner.max.y <= outer.max.y && inner.max.z <= outer.max.z;
    };
    std::size_t flipped = 0;
    for (std::size_t i = 0; i < shells.size(); ++i) {
        const Shell& s = shells[i];
        // Shells do not intersect (a closed manifold mesh): one corner of s
        // decides whether s lies inside another shell.
        const Vector3& probe = vertices[triangles[s.members.front()][0]];
        std::size_t depth = 0;
        for (std::size_t j = 0; j < shells.size(); ++j) {
            if (j != i && box_inside(s.box, shells[j].box) &&
                std::abs(component_winding(vertices, triangles, shells[j].members, probe)) > 0.5) {
                ++depth;
            }
        }
        const double volume6 = v6[s.root];
        const bool want_positive = depth % 2 == 0;
        if ((want_positive && volume6 < 0.0) || (!want_positive && volume6 > 0.0)) {
            for (std::uint32_t t : s.members) {
                flip(triangles[t]);
            }
            flipped += s.members.size();
        }
    }
    return flipped;
}

} // namespace

WatertightReport analyze_mesh(const TriangleMesh& mesh, double weld_tolerance) {
    if (!std::isfinite(weld_tolerance) || weld_tolerance < 0.0) {
        throw std::invalid_argument("analyze_mesh: weld tolerance must be finite and >= 0");
    }
    WatertightReport r;
    r.vertices = mesh.vertices.size();
    r.triangles = mesh.triangles.size();
    std::vector<bool> referenced(mesh.vertices.size(), false);
    for (const Triangle& t : mesh.triangles) {
        for (std::uint32_t v : t) {
            referenced[v] = true;
        }
    }
    r.unreferenced_vertices =
        static_cast<std::size_t>(std::count(referenced.begin(), referenced.end(), false));

    const Weld w = weld(mesh.vertices, weld_tolerance);
    r.duplicate_vertices = w.merged;
    const Cleaned c = clean(mesh.vertices, remap(mesh.triangles, w.map));
    r.degenerate_triangles = c.degenerate;
    r.duplicate_triangles = c.duplicate;

    const std::vector<EdgeUse> uses = edge_uses(c.triangles);
    const Topology t = topology(uses);
    r.edges = t.edges;
    r.boundary_edges = t.boundary;
    r.non_manifold_edges = t.non_manifold;
    r.inconsistent_edges = t.inconsistent;
    const BoundaryLoops loops = boundary_loops(uses);
    r.boundary_loops = loops.closed.size();
    r.open_boundary_chains = loops.open;
    r.components = count_roots(components_of(c.triangles.size(), uses));
    r.signed_volume = volume_of(mesh.vertices, c.triangles, bounds(mesh).centre());
    return r;
}

RepairResult repair_mesh(const TriangleMesh& mesh, const RepairOptions& options) {
    RepairResult result;
    result.before = analyze_mesh(mesh, options.weld_tolerance);

    std::vector<Vector3> vertices = mesh.vertices;
    const Weld w = weld(vertices, options.weld_tolerance);
    result.welded_vertices = w.merged;
    Cleaned c = clean(vertices, remap(mesh.triangles, w.map));
    result.removed_degenerate = c.degenerate;
    result.removed_duplicates = c.duplicate;
    std::vector<Triangle> triangles = std::move(c.triangles);

    // Fins: two or more boundary edges and a non-manifold edge. Removing one
    // can make its non-manifold edge manifold, so repeat until stable.
    while (options.remove_fins) {
        const std::vector<EdgeUse> uses = edge_uses(triangles);
        std::vector<int> boundary(triangles.size(), 0);
        std::vector<bool> on_non_manifold(triangles.size(), false);
        for_each_edge(uses, [&](std::size_t i, std::size_t j) {
            if (j - i == 1) {
                ++boundary[uses[i].triangle];
            } else if (j - i > 2) {
                for (std::size_t k = i; k < j; ++k) {
                    on_non_manifold[uses[k].triangle] = true;
                }
            }
        });
        std::vector<Triangle> kept;
        for (std::size_t t = 0; t < triangles.size(); ++t) {
            if (boundary[t] >= 2 && on_non_manifold[t]) {
                ++result.removed_fins;
            } else {
                kept.push_back(triangles[t]);
            }
        }
        if (kept.size() == triangles.size()) {
            break;
        }
        triangles = std::move(kept);
    }

    if (options.fix_orientation) {
        // Breadth-first propagation over manifold edges: two neighbours are
        // consistent when they use their shared edge in opposite directions.
        const std::vector<EdgeUse> uses = edge_uses(triangles);
        std::vector<std::tuple<std::uint32_t, std::uint32_t, bool>> links;
        for_each_edge(uses, [&](std::size_t i, std::size_t j) {
            if (j - i == 2) {
                const bool same = uses[i].forward == uses[i + 1].forward;
                links.emplace_back(uses[i].triangle, uses[i + 1].triangle, same);
                links.emplace_back(uses[i + 1].triangle, uses[i].triangle, same);
            }
        });
        std::sort(links.begin(), links.end());
        std::vector<std::size_t> first(triangles.size() + 1, 0);
        for (const auto& l : links) {
            ++first[std::get<0>(l) + 1];
        }
        std::partial_sum(first.begin(), first.end(), first.begin());
        std::vector<int> state(triangles.size(), -1); // -1 unvisited, 0 keep, 1 flip
        std::vector<std::uint32_t> queue;
        for (std::size_t seed = 0; seed < triangles.size(); ++seed) {
            if (state[seed] >= 0) {
                continue;
            }
            state[seed] = 0;
            queue.assign(1, to_index(seed));
            for (std::size_t q = 0; q < queue.size(); ++q) {
                const std::uint32_t t = queue[q];
                for (std::size_t k = first[t]; k < first[t + 1]; ++k) {
                    const std::uint32_t u = std::get<1>(links[k]);
                    // Same direction: exactly one of the two must flip.
                    const int wanted = std::get<2>(links[k]) ? 1 - state[t] : state[t];
                    if (state[u] < 0) {
                        state[u] = wanted;
                        queue.push_back(u);
                    } else if (state[u] != wanted) {
                        result.orientation_conflict = true;
                    }
                }
            }
        }
        for (std::size_t t = 0; t < triangles.size(); ++t) {
            if (state[t] == 1) {
                flip(triangles[t]);
                ++result.flipped_triangles;
            }
        }
    }

    if (options.fill_holes) {
        const BoundaryLoops loops = boundary_loops(edge_uses(triangles));
        for (const std::vector<std::uint32_t>& loop : loops.closed) {
            if (loop.size() > options.max_hole_edges) {
                continue;
            }
            // Boundary edges run a -> b in their triangle; the patch uses b -> a.
            if (loop.size() == 3) {
                triangles.push_back({loop[0], loop[2], loop[1]});
                ++result.added_triangles;
            } else {
                Vector3 centre;
                for (std::uint32_t v : loop) {
                    centre += vertices[v];
                }
                const std::uint32_t c_index = to_index(vertices.size());
                vertices.push_back(centre / static_cast<double>(loop.size()));
                for (std::size_t i = 0; i < loop.size(); ++i) {
                    triangles.push_back({loop[(i + 1) % loop.size()], loop[i], c_index});
                }
                result.added_triangles += loop.size();
            }
            ++result.filled_holes;
        }
    }

    if (options.fix_orientation) {
        // Closed components face away from the solid, cavities included.
        result.flipped_triangles += orient_closed_components(vertices, triangles);
    }

    // Drop unreferenced vertices, keeping the order of the others.
    std::vector<std::uint32_t> new_index(vertices.size(),
                                         std::numeric_limits<std::uint32_t>::max());
    for (const Triangle& t : triangles) {
        for (std::uint32_t v : t) {
            new_index[v] = 0;
        }
    }
    std::uint32_t next = 0;
    for (std::size_t v = 0; v < vertices.size(); ++v) {
        if (new_index[v] == 0) {
            new_index[v] = next++;
            result.mesh.vertices.push_back(vertices[v]);
        }
    }
    result.mesh.triangles = remap(triangles, new_index);
    result.after = analyze_mesh(result.mesh, options.weld_tolerance);
    return result;
}

} // namespace orrery::geom
