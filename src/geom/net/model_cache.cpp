#include "model_cache.h"

#include "orrery/geom/net/sha256.h"

#include <algorithm>
#include <array>
#include <fstream>
#include <random>
#include <span>
#include <sstream>
#include <system_error>
#include <vector>

namespace orrery::geom::net::detail {

namespace {

using core::failure;

constexpr std::string_view kEntryFile = "entry.txt";
constexpr std::string_view kUrlDir = "urls";
constexpr std::string_view kStagingDir = ".staging";

FetchError cache_error(std::string message) {
    return FetchError{FetchErrorKind::cache_failure, std::move(message)};
}

std::span<const std::byte> as_bytes(std::string_view text) noexcept {
    return std::as_bytes(std::span{text.data(), text.size()});
}

bool is_sha256_hex(std::string_view text) noexcept {
    return text.size() == 64 && std::all_of(text.begin(), text.end(), [](char c) {
               return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
           });
}

// A single file name inside an entry directory: no separators, no "..".
bool is_plain_file_name(std::string_view name) noexcept {
    return !name.empty() && name != "." && name != ".." &&
           name.find_first_of("/\\:") == std::string_view::npos &&
           std::all_of(name.begin(), name.end(),
                       [](char c) { return static_cast<unsigned char>(c) >= 0x20; });
}

std::optional<ModelFormat> parse_format(std::string_view name) noexcept {
    for (const ModelFormat format :
         {ModelFormat::stl_ascii, ModelFormat::stl_binary, ModelFormat::obj, ModelFormat::gltf,
          ModelFormat::glb, ModelFormat::ply, ModelFormat::fbx}) {
        if (model_format_name(format) == name) {
            return format;
        }
    }
    return std::nullopt;
}

std::string random_name() {
    std::random_device device;
    const std::uint64_t value = (std::uint64_t{device()} << 32U) ^ std::uint64_t{device()};
    std::ostringstream out;
    out << std::hex << value;
    return out.str();
}

// Writes `text` to `file` through a temporary sibling and a rename.
bool write_atomically(const std::filesystem::path& file, std::string_view text) {
    std::filesystem::path temp = file;
    temp += ".tmp-" + random_name();
    {
        std::ofstream out(temp, std::ios::binary | std::ios::trunc);
        out.write(text.data(), static_cast<std::streamsize>(text.size()));
        if (!out) {
            return false;
        }
    }
    std::error_code ec;
    std::filesystem::rename(temp, file, ec);
    if (ec) {
        std::filesystem::remove(temp, ec);
        return false;
    }
    return true;
}

// A directory itself, not a symbolic link or a junction to one.
bool is_real_directory(const std::filesystem::path& dir) {
    std::error_code ec;
    return std::filesystem::symlink_status(dir, ec).type() == std::filesystem::file_type::directory;
}

// A regular file itself, not a link to one.
bool is_real_file(const std::filesystem::path& file) {
    std::error_code ec;
    return std::filesystem::symlink_status(file, ec).type() == std::filesystem::file_type::regular;
}

std::optional<std::string> read_small_file(const std::filesystem::path& file) {
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(file, ec);
    if (ec || size > 64 * 1024) {
        return std::nullopt;
    }
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return std::nullopt;
    }
    std::string text(static_cast<std::size_t>(size), '\0');
    in.read(text.data(), static_cast<std::streamsize>(text.size()));
    if (static_cast<std::size_t>(in.gcount()) != text.size()) {
        return std::nullopt;
    }
    return text;
}

} // namespace

std::string sha256_of_file(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) {
        return {};
    }
    Sha256 hash;
    std::vector<char> buffer(std::size_t{1} << 16U);
    while (in) {
        in.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
        const auto got = static_cast<std::size_t>(in.gcount());
        hash.update(std::as_bytes(std::span{buffer.data(), got}));
    }
    if (in.bad()) {
        return {};
    }
    return to_hex(hash.finish());
}

std::optional<FetchError> ModelCache::prepare() const {
    std::error_code ec;
    for (const std::filesystem::path& dir : {root_, root_ / kUrlDir, root_ / kStagingDir}) {
        std::filesystem::create_directories(dir, ec);
        if (ec) {
            return cache_error("cannot create the cache directory: " + ec.message());
        }
    }
    // Downloads are staged and indexed only inside the cache itself: a link
    // here would move them to wherever it points.
    for (const std::filesystem::path& dir : {root_ / kUrlDir, root_ / kStagingDir}) {
        if (!is_real_directory(dir)) {
            return cache_error("the cache directory '" + dir.filename().string() +
                               "' is a link or not a directory; refusing to use it");
        }
    }
    return std::nullopt;
}

std::optional<CacheEntry> ModelCache::read_entry(const std::string& sha256) const {
    if (!is_sha256_hex(sha256)) {
        return std::nullopt;
    }
    const std::filesystem::path dir = root_ / sha256;
    if (!is_real_directory(dir)) {
        return std::nullopt;
    }
    const auto text = read_small_file(dir / kEntryFile);
    if (!text) {
        return std::nullopt;
    }
    std::string format_name;
    std::string source;
    std::string model;
    CacheEntry entry;
    std::istringstream lines(*text);
    for (std::string line; std::getline(lines, line);) {
        const auto equals = line.find('=');
        if (equals == std::string::npos) {
            return std::nullopt;
        }
        const std::string key = line.substr(0, equals);
        std::string value = line.substr(equals + 1);
        if (key == "format") {
            format_name = std::move(value);
        } else if (key == "source") {
            source = std::move(value);
        } else if (key == "model") {
            model = std::move(value);
        } else if (key == "member") {
            entry.archive_member = std::move(value);
        }
    }
    const auto format = parse_format(format_name);
    if (!format || !is_plain_file_name(source) || !is_plain_file_name(model)) {
        return std::nullopt;
    }
    entry.source_file = dir / source;
    entry.model_file = dir / model;
    entry.sha256 = sha256;
    entry.format = *format;
    if (!is_real_file(entry.source_file) || !is_real_file(entry.model_file)) {
        return std::nullopt;
    }
    // The content address is the only trusted fact: the source must hash to
    // the name of its directory. A digest stored in entry.txt would prove
    // nothing, as whoever can alter a file can alter entry.txt too.
    if (sha256_of_file(entry.source_file) != sha256) {
        return std::nullopt;
    }
    return entry;
}

std::optional<CacheEntry> ModelCache::lookup_url(std::string_view url) const {
    const auto recorded = read_small_file(root_ / kUrlDir / sha256_hex(as_bytes(url)));
    if (!recorded) {
        return std::nullopt;
    }
    std::string sha{*recorded};
    while (!sha.empty() && (sha.back() == '\n' || sha.back() == '\r')) {
        sha.pop_back();
    }
    return read_entry(sha);
}

core::Result<std::filesystem::path, FetchError> ModelCache::make_staging() const {
    for (int attempt = 0; attempt < 8; ++attempt) {
        const std::filesystem::path dir = root_ / kStagingDir / random_name();
        std::error_code ec;
        if (std::filesystem::create_directory(dir, ec)) {
            return dir;
        }
    }
    return failure(cache_error("cannot create a staging directory in the cache"));
}

core::Result<CacheEntry, FetchError> ModelCache::commit(const std::filesystem::path& staging,
                                                        const std::string& sha256,
                                                        const std::filesystem::path& source_name,
                                                        ModelFormat format,
                                                        const std::filesystem::path& model_name,
                                                        const std::string& archive_member) const {
    if (!is_sha256_hex(sha256) || !is_model_format(format) ||
        !is_plain_file_name(source_name.string()) || !is_plain_file_name(model_name.string())) {
        return failure(cache_error("invalid cache entry"));
    }
    const std::string model_sha = sha256_of_file(staging / model_name);
    if (model_sha.empty()) {
        return failure(cache_error("cannot hash the model file"));
    }
    std::string text = "source=" + source_name.string() + "\n";
    text += "format=" + std::string{model_format_name(format)} + "\n";
    text += "model=" + model_name.string() + "\n";
    if (!archive_member.empty()) {
        text += "member=" + archive_member + "\n";
    }
    if (!write_atomically(staging / kEntryFile, text)) {
        return failure(cache_error("cannot write the cache entry description"));
    }

    // An existing entry is reused only if it holds exactly what was just
    // staged: its source is verified by read_entry(), its model is compared
    // with the freshly produced one. Otherwise the fresh download wins.
    const auto identical_entry = [&]() -> std::optional<CacheEntry> {
        auto existing = read_entry(sha256);
        if (!existing || existing->source_file.filename() != source_name ||
            existing->model_file.filename() != model_name || existing->format != format ||
            existing->archive_member != archive_member ||
            sha256_of_file(existing->model_file) != model_sha) {
            return std::nullopt;
        }
        return existing;
    };

    const std::filesystem::path final_dir = root_ / sha256;
    std::error_code ec;
    const auto final_type = std::filesystem::symlink_status(final_dir, ec).type();
    if (final_type != std::filesystem::file_type::not_found) {
        if (auto existing = identical_entry()) {
            std::filesystem::remove_all(staging, ec);
            return *std::move(existing);
        }
        // A damaged, altered or linked entry under this address: replace it.
        // A link is removed itself, never the directory it points to.
        if (final_type == std::filesystem::file_type::directory) {
            std::filesystem::remove_all(final_dir, ec);
        } else {
            std::filesystem::remove(final_dir, ec);
        }
        if (ec) {
            return failure(cache_error("cannot replace a damaged cache entry: " + ec.message()));
        }
    }
    std::filesystem::rename(staging, final_dir, ec);
    if (ec) {
        // Another process may have committed the same content meanwhile.
        if (auto existing = identical_entry()) {
            std::filesystem::remove_all(staging, ec);
            return *std::move(existing);
        }
        return failure(cache_error("cannot move the entry into the cache: " + ec.message()));
    }
    auto entry = read_entry(sha256);
    if (!entry) {
        return failure(cache_error("the committed cache entry cannot be verified"));
    }
    return *std::move(entry);
}

std::optional<FetchError> ModelCache::remember_url(std::string_view url,
                                                   const std::string& sha256) const {
    if (!write_atomically(root_ / kUrlDir / sha256_hex(as_bytes(url)), sha256 + "\n")) {
        return cache_error("cannot record the URL in the cache index");
    }
    return std::nullopt;
}

} // namespace orrery::geom::net::detail
