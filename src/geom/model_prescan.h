#pragma once

#include <cstddef>
#include <optional>
#include <span>
#include <string>

#include "orrery/geom/mesh_import.h"

namespace orrery::geom::detail {

// Checks the element and array counts a model file declares against its
// size and the import limits before the parser reads it: a PLY header or an
// FBX array can declare billions of elements in a few bytes, and the parser
// allocates the declared storage up front. Empty when the file may be parsed.
//   PLY: every element count is within the limits and the declared elements
//        fit in the bytes after the header (ASCII: two bytes per value at
//        least; binary: the exact record size).
//   FBX: every array (ASCII "*N {", binary typed arrays) holds at most
//        24 * max(max_vertices, max_triangles) bytes when decoded, all arrays
//        together at most four times that, a raw array matches its stored
//        length and a deflated one does not claim more than deflate's
//        maximum ratio (1032:1).
// Other formats are not affected (STL, OBJ and glTF parsers bound their
// storage by the data actually present).
[[nodiscard]] std::optional<std::string>
prescan_model(std::span<const std::byte> data, ModelFormat format, const ImportLimits& limits);

} // namespace orrery::geom::detail
