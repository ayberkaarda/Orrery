#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <vector>

#include "orrery/aero/compute/compute_backend.h"
#include "orrery/aero/compute/float32_pack.h"
#include "orrery/aero/compute/null_compute_backend.h"

using namespace orrery::aero::compute;

namespace {

std::vector<std::byte> bytes_of(std::span<const std::uint32_t> words) {
    std::vector<std::byte> out(words.size() * sizeof(std::uint32_t));
    std::memcpy(out.data(), words.data(), out.size());
    return out;
}

} // namespace

TEST_CASE("Null backend buffers start zeroed and round-trip uploads", "[aero][compute]") {
    NullComputeBackend backend;
    auto id = backend.create_buffer(16);
    REQUIRE(id);
    CHECK(id.value().valid());
    CHECK(backend.buffer_size(id.value()) == 16);

    std::vector<std::byte> read(16, std::byte{0xff});
    REQUIRE(backend.readback(id.value(), 0, read));
    for (const std::byte b : read) {
        CHECK(b == std::byte{0});
    }
    const std::array<std::uint32_t, 2> words{0x01020304U, 0xa0b0c0d0U};
    REQUIRE(backend.upload(id.value(), 8, bytes_of(words)));
    std::vector<std::byte> tail(8);
    REQUIRE(backend.readback(id.value(), 8, tail));
    CHECK(tail == bytes_of(words));
}

TEST_CASE("Null backend rejects out-of-range and unknown buffers", "[aero][compute]") {
    NullComputeBackend backend;
    CHECK_FALSE(backend.create_buffer(0));
    const BufferId id = backend.create_buffer(8).value();
    std::vector<std::byte> four(4);
    CHECK_FALSE(backend.upload(id, 5, four));
    CHECK_FALSE(backend.readback(id, 9, std::span<std::byte>(four.data(), 0)));
    CHECK(backend.upload(id, 4, four));
    CHECK_FALSE(backend.upload(BufferId{999}, 0, four));
    CHECK_FALSE(backend.readback(BufferId{}, 0, four));
    CHECK(backend.buffer_size(BufferId{999}) == 0);
    backend.release_buffer(id);
    CHECK(backend.live_buffers() == 0);
    CHECK_FALSE(backend.readback(id, 0, four));
    backend.release_buffer(id); // releasing twice is harmless
}

TEST_CASE("ScopedBuffer releases its buffer", "[aero][compute]") {
    NullComputeBackend backend;
    {
        auto buffer = make_buffer(backend, 32);
        REQUIRE(buffer);
        CHECK(backend.live_buffers() == 1);
        ScopedBuffer moved = std::move(buffer).value();
        CHECK(backend.live_buffers() == 1);
        CHECK(backend.buffer_size(moved.id()) == 32);
    }
    CHECK(backend.live_buffers() == 0);
    CHECK_FALSE(make_buffer(backend, 0));
}

TEST_CASE("Null backend validates, records and runs dispatches", "[aero][compute]") {
    NullComputeBackend backend(false);
    const BufferId a = backend.create_buffer(16).value();
    const BufferId b = backend.create_buffer(16).value();
    const std::array<Uniform, 2> uniforms{Uniform{"u_count", std::uint32_t{4}},
                                          Uniform{"u_scale", Float32Uniform{0.5}}};
    const std::array<BufferBinding, 2> bindings{BufferBinding{0, a}, BufferBinding{1, b}};

    // Unknown kernels are refused by a strict backend.
    CHECK_FALSE(backend.dispatch({"double_it", {4, 1, 1}, uniforms, bindings}));
    CHECK_FALSE(backend.has_kernel("double_it"));

    backend.register_kernel(
        "double_it", [](const DispatchCommand& command, const NullComputeBackend::Bindings& bound) {
            const std::span<std::byte> in = bound.at(0);
            const std::span<std::byte> out = bound.at(1);
            for (std::uint32_t k = 0; k < command.global.x; ++k) {
                std::uint32_t v = 0;
                std::memcpy(&v, in.data() + std::size_t{4} * k, 4);
                v *= 2U;
                std::memcpy(out.data() + std::size_t{4} * k, &v, 4);
            }
            return Status(std::monostate{});
        });
    CHECK(backend.has_kernel("double_it"));
    CHECK(backend.info().executes_kernels);

    const std::array<std::uint32_t, 4> input{1, 2, 3, 40};
    REQUIRE(backend.upload(a, 0, bytes_of(input)));
    backend.barrier();
    REQUIRE(backend.dispatch({"double_it", {4, 1, 1}, uniforms, bindings}));
    std::vector<std::byte> out(16);
    REQUIRE(backend.readback(b, 0, out));
    const std::array<std::uint32_t, 4> expected{2, 4, 6, 80};
    CHECK(out == bytes_of(expected));

    REQUIRE(backend.dispatches().size() == 1);
    const auto& record = backend.dispatches().front();
    CHECK(record.kernel == "double_it");
    CHECK(record.global.x == 4);
    CHECK(record.barriers_before == 1);
    REQUIRE(record.uniforms.size() == 2);
    CHECK(record.uniforms[0].name == "u_count");
    CHECK(std::get<std::uint32_t>(record.uniforms[0].value) == 4);
    CHECK(std::get<Float32Uniform>(record.uniforms[1].value).value == 0.5);
    REQUIRE(record.buffers.size() == 2);
    CHECK(record.buffers[1].buffer == b);

    // Invalid dispatches leave no record.
    CHECK_FALSE(backend.dispatch({"double_it", {0, 1, 1}, uniforms, bindings}));
    const std::array<BufferBinding, 1> unknown{BufferBinding{0, BufferId{77}}};
    CHECK_FALSE(backend.dispatch({"double_it", {1, 1, 1}, {}, unknown}));
    const std::array<BufferBinding, 2> twice{BufferBinding{0, a}, BufferBinding{0, b}};
    CHECK_FALSE(backend.dispatch({"double_it", {1, 1, 1}, {}, twice}));
    CHECK(backend.dispatches().size() == 1);
    CHECK(backend.barrier_count() == 1);
}

TEST_CASE("A lenient Null backend accepts unknown kernels as no-ops", "[aero][compute]") {
    NullComputeBackend backend;
    CHECK(backend.info().name == "null");
    CHECK_FALSE(backend.info().executes_kernels);
    CHECK(backend.dispatch({"anything", {1, 1, 1}, {}, {}}));
    CHECK(backend.dispatches().size() == 1);
    backend.clear_log();
    CHECK(backend.dispatches().empty());
}

TEST_CASE("Binary32 packing rounds to nearest and round-trips", "[aero][compute]") {
    const std::vector<double> values{0.0, 1.0, -2.5, 0.1, 1.0 / 3.0, 1e-30, 3.0e38, 1e39};
    const std::vector<std::byte> packed = pack_float32(values);
    REQUIRE(packed.size() == values.size() * float32_bytes);
    std::vector<double> back(values.size());
    REQUIRE(unpack_float32(packed, back));
    for (std::size_t k = 0; k + 1 < values.size(); ++k) {
        INFO(values[k]);
        CHECK(back[k] == round_to_float32(values[k]));
        CHECK(std::abs(back[k] - values[k]) <= std::abs(values[k]) * 0x1p-24);
    }
    CHECK(std::isinf(back.back())); // beyond the binary32 range
    CHECK(round_to_float32(0.1) != 0.1);
    CHECK(round_to_float32(0.5) == 0.5);

    std::vector<double> wrong(values.size() + 1, 7.0);
    CHECK_FALSE(unpack_float32(packed, wrong));
    CHECK(wrong.front() == 7.0);
}
