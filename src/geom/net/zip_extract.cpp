#include "orrery/geom/net/zip_extract.h"

#include "exclusive_file.h"

#include <miniz/miniz.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <fstream>
#include <span>
#include <system_error>
#include <utility>

namespace orrery::geom::net {

namespace {

using core::failure;

constexpr std::size_t kMaxMemberNameLength = 1024;
constexpr unsigned kHostUnix = 3;
constexpr unsigned kHostMacOs = 19;
constexpr std::uint32_t kFileTypeMask = 0xF000U;
constexpr std::uint32_t kFileTypeSymlink = 0xA000U;
constexpr std::uint64_t kNoOverflow = ~std::uint64_t{0};

FetchError error(FetchErrorKind kind, std::string message) {
    return FetchError{kind, std::move(message)};
}

// Random-access reads of the archive through a stream, so that paths with
// non-ASCII characters work on every platform.
struct ArchiveSource {
    std::ifstream in;
};

std::size_t read_archive(void* opaque, mz_uint64 offset, void* buffer, std::size_t size) {
    auto* source = static_cast<ArchiveSource*>(opaque);
    source->in.clear();
    source->in.seekg(static_cast<std::streamoff>(offset));
    source->in.read(static_cast<char*>(buffer), static_cast<std::streamsize>(size));
    return static_cast<std::size_t>(source->in.gcount());
}

class ZipReader {
public:
    ZipReader() = default;
    ZipReader(const ZipReader&) = delete;
    ZipReader& operator=(const ZipReader&) = delete;
    ZipReader(ZipReader&&) = delete;
    ZipReader& operator=(ZipReader&&) = delete;
    ~ZipReader() {
        if (open_) {
            mz_zip_reader_end(&zip_);
        }
    }

    bool open(const std::filesystem::path& file) {
        std::error_code ec;
        const std::uintmax_t size = std::filesystem::file_size(file, ec);
        if (ec) {
            return false;
        }
        source_.in.open(file, std::ios::binary);
        if (!source_.in) {
            return false;
        }
        zip_.m_pRead = read_archive;
        zip_.m_pIO_opaque = &source_;
        open_ = mz_zip_reader_init(&zip_, size, 0) != MZ_FALSE;
        return open_;
    }

    [[nodiscard]] mz_zip_archive* get() noexcept { return &zip_; }
    [[nodiscard]] const char* last_error() noexcept {
        return mz_zip_get_error_string(mz_zip_get_last_error(&zip_));
    }

private:
    ArchiveSource source_;
    mz_zip_archive zip_{};
    bool open_ = false;
};

struct Entry {
    mz_uint index = 0;
    std::string name;
    std::uint64_t declared_size = 0;
};

// Streams one entry. The first bytes are kept in memory until the format can
// be decided; only a supported model is written to disk.
struct EntrySink {
    std::filesystem::path path;
    std::uint64_t declared_size = 0;
    std::uint64_t written = 0;
    std::vector<std::byte> head;
    detail::ExclusiveFile out;
    ModelFormat format = ModelFormat::unknown;
    bool decided = false;
    bool oversized = false;
    bool skipped = false;
    bool io_error = false;

    bool decide() {
        decided = true;
        format = detect_model_format(head, declared_size);
        if (!is_model_format(format)) {
            skipped = true;
            return false;
        }
        // Created, never opened: a name planted in out_dir (a link to a file
        // elsewhere) makes the extraction fail instead of being written through.
        io_error = !out.create(path) || !out.write(head);
        return !io_error;
    }

    std::size_t accept(std::span<const std::byte> data) {
        if (data.size() > declared_size - written) {
            oversized = true;
            return 0;
        }
        written += data.size();
        if (decided) {
            io_error = !out.write(data);
            return io_error ? 0 : data.size();
        }
        head.insert(head.end(), data.begin(), data.end());
        const auto sniff = static_cast<std::size_t>(
            std::min<std::uint64_t>(declared_size, std::uint64_t{kSniffBytes}));
        if (head.size() >= sniff && !decide()) {
            return 0;
        }
        return data.size();
    }
};

std::size_t write_entry(void* opaque, mz_uint64 /*offset*/, const void* buffer, std::size_t size) {
    auto* sink = static_cast<EntrySink*>(opaque);
    try {
        return sink->accept(std::span{static_cast<const std::byte*>(buffer), size});
    } catch (...) {
        sink->io_error = true;
        return 0;
    }
}

bool is_symlink(const mz_zip_archive_file_stat& stat) noexcept {
    const unsigned host = static_cast<unsigned>(stat.m_version_made_by) >> 8U;
    const std::uint32_t mode = stat.m_external_attr >> 16U;
    return (host == kHostUnix || host == kHostMacOs) && (mode & kFileTypeMask) == kFileTypeSymlink;
}

// File name stem made of [A-Za-z0-9_-], for readable cache file names.
std::string safe_stem(std::string_view member_name) {
    const auto slash = member_name.find_last_of("/\\");
    std::string_view base =
        slash == std::string_view::npos ? member_name : member_name.substr(slash + 1);
    base = base.substr(0, base.find('.'));
    std::string stem;
    for (const char c : base) {
        const bool keep = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (keep && stem.size() < 64) {
            stem.push_back(c);
        }
    }
    return stem.empty() ? std::string{"model"} : stem;
}

std::string numbered(mz_uint index) {
    std::string digits = std::to_string(index);
    if (digits.size() < 4) {
        digits.insert(0, 4 - digits.size(), '0');
    }
    return digits;
}

core::Result<std::vector<Entry>, FetchError> validate(ZipReader& reader,
                                                      const ArchiveLimits& limits) {
    mz_zip_archive* zip = reader.get();
    const mz_uint count = mz_zip_reader_get_num_files(zip);
    if (count == 0) {
        return failure(error(FetchErrorKind::archive_without_model, "the archive is empty"));
    }
    if (count > limits.max_entries) {
        return failure(error(FetchErrorKind::archive_too_many_entries,
                             "the archive has " + std::to_string(count) + " entries, more than " +
                                 std::to_string(limits.max_entries)));
    }
    std::vector<Entry> entries;
    std::uint64_t total = 0;
    std::array<char, kMaxMemberNameLength + 1> name_buffer{};
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat stat{};
        if (mz_zip_reader_file_stat(zip, i, &stat) == MZ_FALSE) {
            return failure(error(FetchErrorKind::archive_invalid,
                                 std::string{"unreadable archive entry: "} + reader.last_error()));
        }
        const mz_uint needed = mz_zip_reader_get_filename(zip, i, nullptr, 0);
        if (needed == 0 || needed > name_buffer.size()) {
            return failure(error(FetchErrorKind::archive_unsafe_entry,
                                 "archive entry name is empty or too long"));
        }
        const mz_uint length = mz_zip_reader_get_filename(zip, i, name_buffer.data(),
                                                          static_cast<mz_uint>(name_buffer.size()));
        std::string name{name_buffer.data(), length > 0 ? length - 1 : 0};
        if (!is_safe_member_name(name)) {
            return failure(error(FetchErrorKind::archive_unsafe_entry,
                                 "unsafe archive entry name '" + name + "'"));
        }
        if (is_symlink(stat)) {
            return failure(error(FetchErrorKind::archive_unsafe_entry,
                                 "archive entry '" + name + "' is a symbolic link"));
        }
        if (stat.m_is_encrypted != MZ_FALSE) {
            return failure(error(FetchErrorKind::archive_unsafe_entry,
                                 "archive entry '" + name + "' is encrypted"));
        }
        if (stat.m_is_directory != MZ_FALSE) {
            continue;
        }
        if (stat.m_is_supported == MZ_FALSE) {
            return failure(error(FetchErrorKind::archive_invalid,
                                 "archive entry '" + name + "' uses an unsupported method"));
        }
        const std::uint64_t compressed = stat.m_comp_size;
        const std::uint64_t uncompressed = stat.m_uncomp_size;
        // uncompressed / compressed > ratio, without dividing and without
        // overflowing (compressed is bounded by the archive size).
        const bool ratio_fits = compressed <= kNoOverflow / limits.max_compression_ratio;
        if (uncompressed > 0 &&
            (compressed == 0 ||
             (ratio_fits && uncompressed > compressed * limits.max_compression_ratio))) {
            return failure(error(FetchErrorKind::archive_bomb,
                                 "archive entry '" + name + "' expands " +
                                     std::to_string(uncompressed) + " bytes from " +
                                     std::to_string(compressed) + ", a ratio above " +
                                     std::to_string(limits.max_compression_ratio)));
        }
        total += uncompressed;
        if (total > limits.max_total_uncompressed_bytes) {
            return failure(error(FetchErrorKind::archive_bomb,
                                 "the archive expands to more than " +
                                     std::to_string(limits.max_total_uncompressed_bytes) +
                                     " bytes"));
        }
        entries.push_back(Entry{i, std::move(name), uncompressed});
    }
    return entries;
}

} // namespace

bool is_safe_member_name(std::string_view name) noexcept {
    if (name.empty() || name.size() > kMaxMemberNameLength) {
        return false;
    }
    if (name.front() == '/' || name.front() == '\\') {
        return false;
    }
    const bool clean = std::all_of(name.begin(), name.end(), [](char c) {
        const auto u = static_cast<unsigned char>(c);
        return u >= 0x20 && u != 0x7F && c != ':';
    });
    if (!clean) {
        return false;
    }
    std::string_view rest = name;
    while (true) {
        const auto separator = rest.find_first_of("/\\");
        const std::string_view component =
            separator == std::string_view::npos ? rest : rest.substr(0, separator);
        if (component == "..") {
            return false;
        }
        if (separator == std::string_view::npos) {
            return true;
        }
        rest = rest.substr(separator + 1);
    }
}

core::Result<std::vector<ExtractedModel>, FetchError>
extract_archive_models(const std::filesystem::path& archive, const std::filesystem::path& out_dir,
                       const ArchiveLimits& limits) {
    if (limits.max_compression_ratio == 0) {
        return failure(error(FetchErrorKind::archive_invalid, "compression ratio limit is zero"));
    }
    ZipReader reader;
    if (!reader.open(archive)) {
        return failure(error(FetchErrorKind::archive_invalid,
                             std::string{"cannot read the zip archive: "} + reader.last_error()));
    }
    auto entries = validate(reader, limits);
    if (!entries) {
        return failure(entries.error());
    }

    std::vector<ExtractedModel> models;
    for (const Entry& entry : entries.value()) {
        EntrySink sink;
        sink.path = out_dir / (numbered(entry.index) + ".part");
        sink.declared_size = entry.declared_size;
        const bool ok = mz_zip_reader_extract_to_callback(reader.get(), entry.index, write_entry,
                                                          &sink, 0) != MZ_FALSE;
        if (sink.oversized) {
            return failure(
                error(FetchErrorKind::archive_bomb,
                      "archive entry '" + entry.name + "' expands beyond its declared size"));
        }
        if (sink.io_error) {
            return failure(error(FetchErrorKind::io_failure,
                                 "cannot write the extracted entry '" + entry.name + "'"));
        }
        if (sink.skipped) {
            continue; // not a model: never written to disk
        }
        if (!ok) {
            return failure(
                error(FetchErrorKind::archive_invalid,
                      "archive entry '" + entry.name + "' is corrupt: " + reader.last_error()));
        }
        if (!sink.decided && !sink.decide()) {
            if (sink.io_error) {
                return failure(error(FetchErrorKind::io_failure,
                                     "cannot write the extracted entry '" + entry.name + "'"));
            }
            continue;
        }
        if (!sink.out.close()) {
            return failure(error(FetchErrorKind::io_failure,
                                 "cannot write the extracted entry '" + entry.name + "'"));
        }
        if (sink.written != entry.declared_size) {
            return failure(error(FetchErrorKind::archive_invalid,
                                 "archive entry '" + entry.name + "' has the wrong size"));
        }
        const std::filesystem::path final_path =
            out_dir / (numbered(entry.index) + "-" + safe_stem(entry.name) +
                       std::string{model_format_extension(sink.format)});
        std::error_code ec;
        std::filesystem::rename(sink.path, final_path, ec);
        if (ec) {
            return failure(error(FetchErrorKind::io_failure,
                                 "cannot name the extracted entry: " + ec.message()));
        }
        models.push_back(ExtractedModel{final_path, sink.format, entry.name});
    }
    if (models.empty()) {
        return failure(error(FetchErrorKind::archive_without_model,
                             "the archive contains no supported model file"));
    }
    return models;
}

} // namespace orrery::geom::net
