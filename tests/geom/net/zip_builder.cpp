#include "net_test_support.h"

#include <miniz/miniz.h>

#include <memory>
#include <stdexcept>

namespace orrery::geom::net::test {

namespace {

void put16(std::string& out, std::uint32_t value) {
    out.push_back(static_cast<char>(value & 0xFFU));
    out.push_back(static_cast<char>((value >> 8U) & 0xFFU));
}

void put32(std::string& out, std::uint32_t value) {
    put16(out, value & 0xFFFFU);
    put16(out, value >> 16U);
}

struct FreeDeleter {
    void operator()(void* p) const noexcept { mz_free(p); }
};

std::string raw_deflate(const std::string& data) {
    std::size_t size = 0;
    const std::unique_ptr<void, FreeDeleter> compressed{
        tdefl_compress_mem_to_heap(data.data(), data.size(), &size, TDEFL_DEFAULT_MAX_PROBES)};
    if (!compressed) {
        throw std::runtime_error("deflate failed");
    }
    return std::string{static_cast<const char*>(compressed.get()), size};
}

} // namespace

std::string build_zip(const std::vector<ZipMember>& members) {
    std::string archive;
    std::string central;
    for (const ZipMember& member : members) {
        const std::string payload = member.deflate ? raw_deflate(member.data) : member.data;
        const auto crc = static_cast<std::uint32_t>(
            mz_crc32(MZ_CRC32_INIT, reinterpret_cast<const unsigned char*>(member.data.data()),
                     member.data.size()));
        const auto method = member.deflate ? 8U : 0U;
        const auto compressed = static_cast<std::uint32_t>(payload.size());
        const std::uint32_t uncompressed =
            member.declared_size.value_or(static_cast<std::uint32_t>(member.data.size()));
        const auto name_length = static_cast<std::uint32_t>(member.name.size());
        const auto offset = static_cast<std::uint32_t>(archive.size());

        put32(archive, 0x04034b50U);
        put16(archive, 20);
        put16(archive, member.flags);
        put16(archive, method);
        put16(archive, 0);    // time
        put16(archive, 0x21); // date: 1980-01-01
        put32(archive, crc);
        put32(archive, compressed);
        put32(archive, uncompressed);
        put16(archive, name_length);
        put16(archive, 0);
        archive += member.name;
        archive += payload;

        put32(central, 0x02014b50U);
        put16(central, member.unix_mode ? (3U << 8U) | 20U : 20U);
        put16(central, 20);
        put16(central, member.flags);
        put16(central, method);
        put16(central, 0);
        put16(central, 0x21);
        put32(central, crc);
        put32(central, compressed);
        put32(central, uncompressed);
        put16(central, name_length);
        put16(central, 0); // extra
        put16(central, 0); // comment
        put16(central, 0); // disk
        put16(central, 0); // internal attributes
        put32(central, member.unix_mode ? (*member.unix_mode << 16U) : 0U);
        put32(central, offset);
        central += member.name;
    }
    const auto central_offset = static_cast<std::uint32_t>(archive.size());
    const auto central_size = static_cast<std::uint32_t>(central.size());
    const auto count = static_cast<std::uint32_t>(members.size());
    archive += central;
    put32(archive, 0x06054b50U);
    put16(archive, 0);
    put16(archive, 0);
    put16(archive, count);
    put16(archive, count);
    put32(archive, central_size);
    put32(archive, central_offset);
    put16(archive, 0);
    return archive;
}

} // namespace orrery::geom::net::test
