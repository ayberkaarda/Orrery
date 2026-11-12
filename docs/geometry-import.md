# Geometry Pipeline and Model Import

Reference for `orrery_geom` (Phase 6a): parametric primitives, model import,
repair and the watertight report, voxelization with a narrow-band signed
distance field, panel meshes and reference quantities. Physics-facing
approximations are listed in `physics.md` ("Geometry pipeline"); the design
decisions are in ADR-018 (kernels without libigl, validation gate around
assimp) and ADR-010 (untrusted models). URL download, cache and archives are
Phase 6b (`src/geom/net`).

Everything is double precision. A `TriangleMesh` holds vertices (metres once
the unit scale is applied) and index triples, counter-clockwise seen from
outside. The project frame is right-handed with +Z up; aerodynamic helpers
default to the free stream along +X and lift along +Z, so the span runs
along +Y.

## 1. Primitives

Headers `orrery/geom/primitives.h` and `orrery/geom/naca.h`. Each primitive
has two exact references, so a generated mesh is checked against an exact
number at any resolution:

| Primitive | Placement | True shape (closed form) | Tessellation (exact polyhedral value) |
|---|---|---|---|
| Sphere r | centred | 4/3 pi r^3, 4 pi r^2 | regular N-gon frustums (below) |
| Cylinder r, h | base at origin, axis +Z | pi r^2 h, 2 pi r (r + h) | same |
| Cone r, h | base at origin, apex at +h | pi r^2 h / 3, pi r (r + sqrt(r^2 + h^2)) | same |
| Capsule (Apollo / Orion type) | nose at origin, axis +Z | arc and line integrals (below) | same |
| Ellipsoid a, b, c | centred | 4/3 pi a b c; 4 pi a b c R_G(a^-2, b^-2, c^-2) | a b c times the unit-sphere tessellation volume |
| Box, cube, flat plate | centred | l_x l_y l_z, 2 (l_x l_y + l_y l_z + l_z l_x) | identical (12 triangles) |
| Wedge L, half angle t, span b | leading edge on the Y axis | L^2 tan(t) b; 2 L h + 2 b L / cos(t) + 2 b h, h = L tan(t) | identical (8 triangles) |
| NACA 4-digit wing | section in X-Z, extruded along Y | section area by quadrature (below) | prism: polygon area x span |

**Solids of revolution.** A `RevolutionProfile` is a chain of lines and
circular arcs in the (r, z) half plane from the axis back to the axis. The
true volume is the integral of pi r^2 dz and the area the integral of
2 pi r ds; both have elementary antiderivatives on an arc
r = r_c + R cos(phi), z = z_c + R sin(phi):

    V = pi R [ r_c^2 sin(phi) + r_c R (phi + sin(phi) cos(phi)) + R^2 (sin(phi) - sin^3(phi) / 3) ]
    A = 2 pi R | r_c phi + R sin(phi) |

and on a line from (r0, z0) to (r1, z1): V = pi (z1 - z0)(r0^2 + r0 r1 + r1^2) / 3,
A = pi (r0 + r1) L. The tessellation revolves the profile polyline (arcs cut
into pieces of at most `arc_step` radians) with N vertices per ring at the
same angles on every ring. Each band is then a frustum of a regular N-gon
pyramid with planar trapezoid faces, so

    V_poly = sum (z1 - z0) k (r0^2 + r0 r1 + r1^2) / 3,   k = (N / 2) sin(2 pi / N)
    A_poly = sum N sin(pi / N) (r0 + r1) sqrt((z1 - z0)^2 + (r1 - r0)^2 cos^2(pi / N))

The poles are single vertices, so every mesh is closed and 2-manifold.

**Mesh size.** Each resolution parameter has its own range (`around` up to
100000, arcs up to 10^6 pieces, NACA `n` up to 10^6 and `span_divisions` up
to 10^5), but together they can describe meshes far too large to build. The
revolution mesh and the NACA wing therefore compute the vertex and triangle
counts their resolution implies (with saturating arithmetic) and throw
`std::invalid_argument` above the default `MeshLimits` (10^7 each, which also
keeps every index below 2^32 - 1) before the profile polyline, the section or
the mesh is allocated.

**Capsule.** Heat-shield sphere of radius R_n (nose at the origin), a
toroidal shoulder of radius R_s tangent to it whose outermost point is the
maximum radius D/2, a conical afterbody leaving the shoulder tangentially at
the half angle theta_c, and a flat top of diameter d_top. The shoulder centre
is at (D/2 - R_s, R_n - sqrt((R_n - R_s)^2 - (D/2 - R_s)^2)). Presets:

| Preset | D | R_n | R_s | theta_c | d_top | height |
|---|---|---|---|---|---|---|
| `apollo()` | 3.912 m | 4.694 m | 0.196 m | 33 deg | 0.7 m | 3.09 m |
| `orion()` | 5.03 m | 6.04 m | 0.25 m | 32.5 deg | 1.8 m | 3.32 m |

The presets are geometric stand-ins (rounded apex replaced by a flat top),
see G1 in `physics.md`.

**Ellipsoid area.** Carlson's symmetric integrals R_F and R_D by the
duplication theorem (stopping tolerance 8e-4, truncation error below 1e-17)
give R_G = (z R_F - (x - z)(y - z) R_D / 3 + sqrt(x y / z)) / 2 with the
middle argument as z, so every term is non-negative.

**NACA 4-digit.** `Naca4::from_designation("2412")` gives camber m = 0.02 at
p = 0.4 and thickness t = 0.12. Half thickness
y_t = 5 t (0.2969 sqrt(x) - 0.1260 x - 0.3516 x^2 + 0.2843 x^3 - 0.1036 x^4)
(closed trailing edge, G2), applied normal to the camber line. The section
area is the integral of 2 y_t sqrt(1 + y_c'^2) dx (the Jacobian of the
normal offset integrates to this exactly), evaluated by composite 24-point
Gauss-Legendre quadrature after x = u^2 (which turns y_t dx into a
polynomial in u), split at the camber kink x = p. For a symmetric section it
equals the closed form 10 t (0.2969 * 2/3 - 0.1260/2 - 0.3516/3 + 0.2843/4 -
0.1036/5). Sections use cosine spacing x_s = (1 - cos(pi s / n)) / 2; the
wing is extruded in equal span strips and closed by flat tip caps
triangulated between the upper and lower point of each station. The
trailing-edge vertices are returned for the wake.

## 2. Import

Header `orrery/geom/mesh_import.h`. Formats: OBJ, STL (ASCII and binary),
glTF 2.0 (`.gltf`, `.glb`), PLY, FBX, read through assimp. Every file is
untrusted (ADR-010, ADR-018) and goes through four stages:

1. **Pre-scan** of the counts the file declares (PLY element counts against
   the limits and the bytes after the header, the header ending at the first
   line that is exactly `end_header` as the parser reads it, so a comment
   containing the word cannot end it early; FBX ASCII `*N {` arrays and
   binary typed arrays against a decoded-size budget and, for deflated
   arrays, deflate's maximum ratio 1032:1). Parsers that allocate the
   declared storage up front never see a hostile count.
2. **Parse** with no post-processing.
3. **Validation of the parsed scene**: node hierarchy (at most 100000 nodes,
   depth 128), mesh references, vertex count and the triangle count that
   triangulation will produce, finite coordinates within 1e12, every face
   index below its mesh's vertex count.
4. **Post-processing**, only `Triangulate | JoinIdenticalVertices |
   GenNormals`, then conversion to double with all node transforms applied
   (in double) and the same checks again (`validate_mesh`). A node whose
   world transform mirrors (negative determinant of its linear part) has
   its triangles reversed, so a mirrored closed mesh stays outward.

**Precision (G7).** The vcpkg assimp port is built without
`ASSIMP_DOUBLE_PRECISION` and offers no feature to enable it, so assimp
parses coordinates and node transforms into `float` (`ai_real`), and
triangulation and vertex joining run in `float`. The conversion to double is
exact but cannot restore what the parser rounded: text coordinates closer
than float resolution (relative 6e-8, for example `1.0` and `1.000000001`)
collapse to one value. Everything after the import (repair, voxels, panels,
reference quantities) is double. Models whose detail is below 1e-7 of their
extent need a double-precision assimp build (an overlay port with the
define set for both the library and its users, since `ai_real` changes the
ABI); the box fixtures use float-exact coordinates for this reason.

Point and line primitives are dropped and counted (`skipped_primitives`).
Files a model refers to (glTF buffers, OBJ materials) are opened only from
the model's own directory, read-only; `import_mesh_memory` refuses every
file reference, so only self-contained data (glb, embedded buffers) loads
from memory. glTF, glb and OBJ files are parsed from their path for this
reason; STL, PLY and FBX are read into memory (after the size check) and
parsed from there. For the path-based formats the file system handed to
assimp serves the model file from the bytes already read and pre-scanned
(it is never opened a second time, so a file replaced after the check is
not what gets parsed) and reads every referenced file whole into memory
against what is left of `max_file_bytes`: the model and everything it
references together stay within the limit, and exceeding it is reported as
`file_too_large`.

Limits (`ImportLimits`, all configurable):

| Limit | Default |
|---|---|
| File size (model and referenced files together) | 200 MB (ADR-010) |
| Vertices | 10 000 000 |
| Triangles (after triangulation) | 10 000 000 |
| Coordinate magnitude | 1e12 file units |
| Nodes, node depth | 100 000, 128 |
| One FBX array, decoded | 24 x max(vertices, triangles) bytes (240 MB) |
| All FBX arrays, decoded | 4 x the one-array budget |

Errors are values (`core::Result<ImportedMesh, ImportError>`): codes
`unsupported_format`, `file_not_found`, `file_too_large`, `parse_failed`,
`too_many_nodes` and `invalid_mesh`, the last with a `MeshValidationError`
(`empty`, `too_many_vertices`, `too_many_triangles`, `malformed_array`,
`non_finite_coordinate`, `coordinate_out_of_range`, `index_out_of_range`)
naming the offending element. The same validator serves flat arrays from
other sources (`mesh_from_arrays`).

**Units.** The imported mesh keeps the file's units. `ImportedMesh::unit`
suggests a scale to metres; the caller confirms it (or chooses another) and
applies it with `apply_unit_scale`:

| Source | Suggestion | `from_format` |
|---|---|---|
| glTF / glb | metres (glTF 2.0 specification) | yes |
| FBX with `UnitScaleFactor` | factor / 100 (the factor is centimetres per unit) | yes |
| Anything else | largest bounding-box extent e: e > 500 mm, 50 < e <= 500 cm, otherwise m (G3) | no |

**Up axis.** `convert_to_z_up` rotates a Y-up mesh by +90 degrees about X,
(x, y, z) -> (x, -z, y), which keeps handedness and orientation. glTF is +Y up
by specification; FBX reports `UpAxis` (1 = Y, 2 = Z) in its global
settings; OBJ is assumed +Y up and STL and PLY +Z up (convention only,
`from_format` false).

## 3. Repair and the watertight report

Header `orrery/geom/mesh_repair.h`. `analyze_mesh` welds vertices at the same
position (or within a tolerance cell), then counts degenerate triangles (two
equal welded corners, or twice the area below 1e-12 of the squared longest
edge) and duplicates (the same three welded corners as an earlier triangle,
either orientation). Topology is evaluated on the remaining triangles, by
sorting edge uses (deterministic, no hash iteration):

| Field | Meaning |
|---|---|
| `boundary_edges` | edges used by one triangle |
| `non_manifold_edges` | edges used by three or more |
| `inconsistent_edges` | edges used twice in the same direction (a flipped neighbour) |
| `boundary_loops` | closed loops of directed boundary edges (holes) |
| `open_boundary_chains` | boundary chains that do not close (free edges of a fin) |
| `components` | edge-connected components |
| `signed_volume` | divergence-theorem volume, positive when closed and outward |
| `watertight()` | no boundary, non-manifold or inconsistent edge |

`repair_mesh` runs, in this order: weld; drop degenerate and duplicate
triangles; drop fins (two or more boundary edges and a non-manifold edge,
repeated until stable); make orientation consistent per component by
breadth-first propagation over manifold edges (a conflict, as on a Moebius
strip, is reported); fill each boundary loop up to `max_hole_edges` (three
edges: one triangle; more: a fan from the loop centroid, G4); orient every
closed component so the solid lies on its inner side; drop unreferenced
vertices. It reports what it did and the report before and after.

Orientation of closed components counts nesting: a component enclosed by an
even number of other closed components (an outer wall, or an island inside a
cavity) gets a positive volume, one enclosed by an odd number (the wall of a
cavity) a negative volume, so a hollow solid keeps its cavity (volume
outer - inner, winding number 0 in the void). Enclosure is decided by the
winding number of the candidate enclosing component at one corner of the
other, after a bounding-box test; components are assumed not to intersect
(true for a closed 2-manifold mesh). Cost: one direct winding sum per pair
of components whose boxes nest.

A positive weld tolerance quantizes coordinates by division; a tolerance so
small that the largest coordinate divided by it is not finite (for example
1e-320 for coordinates of order 1) would put every vertex in the same
infinite cell, so `analyze_mesh` and `repair_mesh` reject it with
`std::invalid_argument` before changing anything.

The fixture `tests/geom/fixtures/broken_box.stl` (unit cube with a missing
triangle, a flipped triangle, a duplicate, a degenerate triangle and a fin on
an edge) is reported as 14 triangles, 1 degenerate, 1 duplicate, 20 edges,
5 boundary edges, 1 non-manifold edge, 3 inconsistent edges, 1 hole, 1 open
chain, 1 component; repaired it is the watertight, outward unit cube (volume
1, area 6, 8 vertices).

## 4. Voxelization and signed distance

Header `orrery/geom/voxel_grid.h`, `orrery/geom/winding_number.h`. A
`GridSpec` places cell centres at origin + h (i + 1/2, j + 1/2, k + 1/2);
`grid_around` covers a bounding box with padding (at most 2^27 cells). Every
grid is validated before any arithmetic on it: at least one cell per axis, a
cell count at most 2^27 computed without wrapping (a product like
2^63 x 2 x 1 is rejected, not taken as zero cells), a finite origin, a
positive finite spacing, finite cell centres; `grid_around` also needs finite,
ordered bounds, `voxelize` a finite threshold and `signed_distance_band` a band
that stays finite once scaled by the spacing. A cell
is solid when the generalized winding number at its centre is at least 1/2
(Jacobson et al. 2013): w(p) = sum of the signed solid angles of all
triangles / 4 pi, each from the Van Oosterom-Strackee formula
tan(Omega / 2) = a . (b x c) / (|a||b||c| + (a.b)|c| + (b.c)|a| + (c.a)|b|).
w is 1 inside and 0 outside a closed surface and degrades smoothly for holes
and overlaps. The formula is evaluated on the unit vectors a/|a|, b/|b|,
c/|c| (dividing both arguments of the arctangent by |a||b||c|), so it does
not underflow or overflow for very small or very large triangles; a point on
a corner sees no solid angle. `MeshTree` evaluates it hierarchically (Barill
et al. 2018): a median-split tree whose node is replaced by its dipole
(sum a_t n_t) . (c - p) / (4 pi |c - p|^3) when the distance to its
area-weighted centre c exceeds beta = 2 times its radius (G5). The tree
stores area vectors divided by s^2, s a power of two near the mesh extent,
and evaluates the dipole as ((A / s^2) . u) (s / |c - p|)^2 with u the unit
direction, so the same mesh scaled by 1e-110 or 1e100 classifies the same
points.

`signed_distance_band` stores, within `band_cells` cells of the surface, the
exact distance to the closest triangle (Ericson's closest point on a
triangle; a triangle without area, from repeated or collinear corners, is
taken as its three edges, so the distance is never NaN; the same tree with
box-distance pruning) with the sign of the voxel
classification (negative inside); other cells hold +-band and `in_band = 0`.
This is the wall description the LBM solver uses (A4).

## 5. Panel mesh and wake

Header `orrery/geom/panel_mesh.h`. One panel per triangle: collocation point
at the centroid, outward unit normal, area (a zero-area triangle is an
error). `detect_trailing_edge` finds edges with exactly two faces whose
normals differ by more than 2 rad (115 degrees) and both face downstream,
and chains them into one polyline ordered along the span. `add_wake` sheds
flat quadrilateral panels from each trailing-edge segment along the free
stream (`rows` panels over `length`), each carrying the upper and lower
surface panels that meet at its segment for the Kutta condition (A5: fixed
wake). The direction and the up vector must be finite and the direction
non-zero; a wake panel whose area is below 1e-9 of the rectangle spanned by
its segment and the wake step (a direction along the trailing edge) is
rejected. All panels are built first and appended only when every one is
usable, so a rejected request leaves the panel mesh unchanged.

## 6. Reference quantities

Header `orrery/geom/reference_quantities.h`. For a closed, outward mesh:

| Quantity | Definition |
|---|---|
| `wetted_area` | total surface area |
| `frontal_area`, `planform_area` | silhouette along the flow and the lift direction: union of the projected triangles on a raster of 1024 cells along the larger side (G6) |
| `frontal_area_bound`, `planform_area_bound` | half the sum of abs(n . d) A: exact for convex meshes and for surfaces single valued over the projection plane, an upper bound otherwise |
| `reference_length` | extent along the flow |
| `span` | extent along lift x flow |
| `volume` | divergence-theorem volume |
| `centre_of_mass` | volume centroid, uniform density (A1) |

The user chooses the coefficient normalisation with `ReferenceAreaKind`
(frontal, planform or wetted).

## 7. Tests

`tests/geom` (Catch2, tag `[geom]`): primitives against closed forms and an
independent quadrature, tessellations against their polyhedral closed forms
and second-order convergence, Carlson's published test values; NACA areas;
every format importing the same box; FBX unit and axis; hostile arrays and
files (NaN and infinite coordinates, out-of-range indices, oversized counts
in STL, PLY and FBX headers, a PLY comment naming `end_header` before the
counts, a deflate bomb array, size limits, referenced files over the byte
budget, escaping references, a memory import referring to an absolute path
that resolves on disk); mirroring node transforms (single, nested, matrix);
the broken STL fixture; welding (and a subnormal tolerance), inside-out
meshes, large holes, hollow and nested solids and a Moebius strip; winding
number against the direct sum and across scales from 1e-150 to 1e100;
closest points on degenerate triangles; grid specs with wrapped, zero or
oversized counts, reversed bounds, non-finite origins, spacings and
thresholds; sphere voxelization and SDF; panels and wakes (including
non-finite and along-the-edge directions); resolutions whose meshes are too
large; appending a mesh to itself; reference quantities. Measured values are
in `physics.md`.
