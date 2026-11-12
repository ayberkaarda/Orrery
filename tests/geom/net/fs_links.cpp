#include "net_test_support.h"

#ifdef _WIN32
#include <windows.h>
#include <winioctl.h>

#include <cstring>
#include <string>
#endif

namespace orrery::geom::net::test {

#ifdef _WIN32
namespace {

class Handle {
public:
    explicit Handle(HANDLE handle) noexcept : handle_(handle) {}
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;
    Handle(Handle&&) = delete;
    Handle& operator=(Handle&&) = delete;
    ~Handle() {
        if (handle_ != INVALID_HANDLE_VALUE) {
            CloseHandle(handle_);
        }
    }
    [[nodiscard]] HANDLE get() const noexcept { return handle_; }

private:
    HANDLE handle_;
};

template <typename T> void put(std::vector<std::byte>& buffer, std::size_t offset, T value) {
    std::memcpy(buffer.data() + offset, &value, sizeof value);
}

} // namespace

// A junction is a mount-point reparse point; creating one needs no privilege.
bool make_directory_link(const std::filesystem::path& target, const std::filesystem::path& link) {
    std::error_code ec;
    if (!std::filesystem::create_directory(link, ec)) {
        return false;
    }
    const Handle handle{CreateFileW(link.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                                    FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT,
                                    nullptr)};
    if (handle.get() == INVALID_HANDLE_VALUE) {
        return false;
    }
    const std::wstring print = std::filesystem::absolute(target).wstring();
    const std::wstring substitute = L"\\??\\" + print;
    const std::size_t substitute_bytes = substitute.size() * sizeof(wchar_t);
    const std::size_t print_bytes = print.size() * sizeof(wchar_t);
    const std::size_t path_bytes =
        substitute_bytes + sizeof(wchar_t) + print_bytes + sizeof(wchar_t);
    // REPARSE_DATA_BUFFER, mount point variant: tag, data length, reserved,
    // then four 16-bit offsets/lengths and the two NUL-terminated names.
    std::vector<std::byte> buffer(16 + path_bytes);
    put<std::uint32_t>(buffer, 0, IO_REPARSE_TAG_MOUNT_POINT);
    put<std::uint16_t>(buffer, 4, static_cast<std::uint16_t>(8 + path_bytes));
    put<std::uint16_t>(buffer, 8, 0);
    put<std::uint16_t>(buffer, 10, static_cast<std::uint16_t>(substitute_bytes));
    put<std::uint16_t>(buffer, 12, static_cast<std::uint16_t>(substitute_bytes + sizeof(wchar_t)));
    put<std::uint16_t>(buffer, 14, static_cast<std::uint16_t>(print_bytes));
    std::memcpy(buffer.data() + 16, substitute.data(), substitute_bytes);
    std::memcpy(buffer.data() + 16 + substitute_bytes + sizeof(wchar_t), print.data(), print_bytes);
    DWORD returned = 0;
    return DeviceIoControl(handle.get(), FSCTL_SET_REPARSE_POINT, buffer.data(),
                           static_cast<DWORD>(buffer.size()), nullptr, 0, &returned,
                           nullptr) != FALSE;
}
#else
bool make_directory_link(const std::filesystem::path& target, const std::filesystem::path& link) {
    std::error_code ec;
    std::filesystem::create_directory_symlink(target, link, ec);
    return !ec;
}
#endif

bool make_file_symlink(const std::filesystem::path& target, const std::filesystem::path& link) {
    std::error_code ec;
    std::filesystem::create_symlink(target, link, ec);
    return !ec;
}

} // namespace orrery::geom::net::test
