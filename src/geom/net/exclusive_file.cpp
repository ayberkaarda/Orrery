#include "exclusive_file.h"

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#include <windows.h>

#include <cstdint>
#endif

namespace orrery::geom::net::detail {

void ExclusiveFile::Closer::operator()(std::FILE* file) const noexcept {
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): the handle owned by file_
    static_cast<void>(std::fclose(file));
}

bool ExclusiveFile::create(const std::filesystem::path& file) {
    file_.reset();
    failed_ = false;
#ifdef _WIN32
    // CREATE_NEW alone follows a dangling symbolic link and creates its
    // target; FILE_FLAG_OPEN_REPARSE_POINT makes the link itself the name
    // that exists, so creation fails for every planted link or junction.
    HANDLE handle = CreateFileW(file.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_NEW,
                                FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
    if (handle == INVALID_HANDLE_VALUE) {
        return false;
    }
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast): CRT handle API
    const int fd = _open_osfhandle(reinterpret_cast<std::intptr_t>(handle), _O_WRONLY | _O_BINARY);
    if (fd == -1) {
        CloseHandle(handle);
        return false;
    }
    std::FILE* raw = _fdopen(fd, "wb");
    if (raw == nullptr) {
        _close(fd); // closes the handle too
        return false;
    }
#else
    // "x": O_EXCL, which fails for any existing name, a dangling symbolic
    // link included, without following it.
    std::FILE* raw = std::fopen(file.c_str(), "wbx"); // NOLINT(cppcoreguidelines-owning-memory)
#endif
    file_.reset(raw);
    return file_ != nullptr;
}

bool ExclusiveFile::write(std::span<const std::byte> data) {
    if (!file_ || failed_) {
        failed_ = true;
        return false;
    }
    if (!data.empty() && std::fwrite(data.data(), 1, data.size(), file_.get()) != data.size()) {
        failed_ = true;
    }
    return !failed_;
}

bool ExclusiveFile::close() {
    if (!file_) {
        return false;
    }
    std::FILE* raw = file_.release();
    // NOLINTNEXTLINE(cppcoreguidelines-owning-memory): released from file_ above
    const bool closed = std::fclose(raw) == 0;
    return closed && !failed_;
}

} // namespace orrery::geom::net::detail
