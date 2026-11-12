#pragma once

#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <span>

// A file that is created, never opened (private to orrery_geom_net).
//
// create() fails if anything already exists under the name: a regular file,
// a hard link, a symbolic link (dangling or not) or a junction; it never
// follows a link (O_EXCL on POSIX, CREATE_NEW with
// FILE_FLAG_OPEN_REPARSE_POINT on Windows). A write can
// therefore never go through a name that someone planted beforehand, which
// truncating an existing name with std::ofstream would follow.
namespace orrery::geom::net::detail {

class ExclusiveFile {
public:
    // Creates `file` for binary writing; false if the name exists or the
    // file cannot be created.
    [[nodiscard]] bool create(const std::filesystem::path& file);

    // Appends `data`; false (and stays failed) on an error.
    [[nodiscard]] bool write(std::span<const std::byte> data);

    // Flushes and closes the file; false if any write or the close failed.
    [[nodiscard]] bool close();

private:
    struct Closer {
        void operator()(std::FILE* file) const noexcept;
    };
    std::unique_ptr<std::FILE, Closer> file_;
    bool failed_ = false;
};

} // namespace orrery::geom::net::detail
