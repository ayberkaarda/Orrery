# ADR-018: Geometry kernels in orrery_geom, assimp behind a validation gate

## Context
Phase 6a needs, besides model import, a small set of mesh algorithms:
manifold and orientation analysis, simple repair, the generalized winding
number (Jacobson et al. 2013) for solid/fluid voxelization, closest-point
distances for a narrow-band SDF, and panel and reference-quantity
extraction. The plan named libigl (with Eigen) for these and assimp for
import. Two further facts shaped the decision:

- libigl is a large header-only library on top of Eigen. Including it in
  `orrery_geom` would compile thousands of template lines into every geometry
  translation unit, and the project builds with warnings as errors on MSVC,
  GCC (-O3 scan) and clang-tidy; libigl's fast winding number also brings its
  own spatial tree and parallel loops whose evaluation order is not fixed.
- assimp parses untrusted files. Its post-processing steps do not check face
  indices (with assimp 6.0.6 a PLY face index past the vertex array came out
  of the post-processing steps as an in-range index), and several parsers
  allocate the storage a header declares before reading the data (a PLY
  header of a few hundred bytes made the parser commit about 20 GB before it
  was stopped).

## Decision
- The mesh algorithms are implemented in `orrery_geom` in plain C++20 and
  double precision, about 1200 lines with no third-party dependency:
  sort-based edge analysis (no hash-map iteration, so results are
  reproducible), breadth-first orientation propagation, centroid-fan hole
  filling, the Van Oosterom-Strackee solid angle, and a median-split bounding
  volume hierarchy that serves both the hierarchical winding number (Barill
  et al. 2018, first-order far field) and closest-point queries. libigl and
  Eigen are not added.
- assimp is the only new dependency (private to `orrery_geom`, recorded in
  `tools/check_deps.py`). Every import runs in four stages: a pre-scan of
  declared counts against the file size and the limits (PLY headers, FBX
  arrays), a parse with no post-processing, a full validation of the parsed
  scene (nodes, mesh references, counts including the triangles
  triangulation will create, finite coordinates, every face index), and only
  then the three permitted steps Triangulate, JoinIdenticalVertices and
  GenNormals, followed by a second validation during conversion to double.
- Files referenced by a model are opened only from the model's own directory;
  memory imports may not open any file. The model file is parsed from the
  bytes that were size-checked and pre-scanned (never reopened), and every
  referenced file is read whole into memory against the same
  `max_file_bytes` budget, so the model and its references together stay
  within the limit.

## Consequences
The geometry library builds in seconds and stays warning-free under every
compiler of the project; its results are deterministic and tested against
direct sums (winding number), closed forms (primitives) and fixtures
(watertight report). The project maintains the algorithms itself and gets
fewer of them than libigl offers: no remeshing, no constrained hole filling,
no exact (rational) predicates. The pre-scan covers the formats whose parsers
allocate from declared counts (PLY, FBX); STL, OBJ and glTF bound their
storage by the data present or check it themselves. Shared-library builds
(vcpkg x64-windows) ship assimp's DLL next to every executable that links
`orrery_geom`. The vcpkg assimp port is single precision (`ai_real` is
`float`): imported coordinates carry float resolution until they are widened
to double (G7 in `physics.md`); building assimp with
`ASSIMP_DOUBLE_PRECISION` would need an overlay port and the matching define
in every user of its headers.

Alternatives rejected: libigl with Eigen (compile cost and warning surface
for a few hundred lines of algorithms), the CGAL polygon-mesh-processing
package (licence: GPL), running assimp's ValidateDataStructure step (outside
the permitted post-processing set, and it runs after the parser has already
allocated), and trusting assimp's parsers with untrusted input as-is.

## Status
Accepted
