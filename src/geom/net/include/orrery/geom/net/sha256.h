#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

// SHA-256 (FIPS 180-4), used as the content address of downloaded models.
namespace orrery::geom::net {

class Sha256 {
public:
    using Digest = std::array<std::uint8_t, 32>;

    Sha256() noexcept;

    void update(std::span<const std::byte> data) noexcept;
    // Finishes the hash; the object must not be updated afterwards.
    [[nodiscard]] Digest finish() noexcept;

private:
    void compress(const std::uint8_t* block) noexcept;

    std::array<std::uint32_t, 8> state_{};
    std::array<std::uint8_t, 64> buffer_{};
    std::size_t buffered_ = 0;
    std::uint64_t total_bytes_ = 0;
};

// Lower-case hexadecimal form of a digest (64 characters).
[[nodiscard]] std::string to_hex(const Sha256::Digest& digest);

[[nodiscard]] std::string sha256_hex(std::span<const std::byte> data);

} // namespace orrery::geom::net
