#include "net_test_support.h"

#include "orrery/geom/net/sha256.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

using orrery::geom::net::Sha256;
using orrery::geom::net::sha256_hex;
using orrery::geom::net::to_hex;
using orrery::geom::net::test::bytes_of;

TEST_CASE("SHA-256 matches the FIPS 180-4 test vectors", "[net][sha256]") {
    CHECK(sha256_hex(bytes_of("")) ==
          "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    CHECK(sha256_hex(bytes_of("abc")) ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(sha256_hex(bytes_of("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq")) ==
          "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");

    const std::string million(1'000'000, 'a');
    CHECK(sha256_hex(bytes_of(million)) ==
          "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST_CASE("SHA-256 gives the same digest for any split of the input", "[net][sha256]") {
    std::string data;
    for (int i = 0; i < 1000; ++i) {
        data.push_back(static_cast<char>('a' + i % 26));
    }
    const std::string whole = sha256_hex(bytes_of(data));
    // Splits around the 55/56/64-byte padding boundaries and odd sizes.
    for (const std::size_t split :
         {std::size_t{0}, std::size_t{1}, std::size_t{55}, std::size_t{56}, std::size_t{63},
          std::size_t{64}, std::size_t{65}, std::size_t{999}}) {
        Sha256 hash;
        hash.update(bytes_of(std::string_view{data}.substr(0, split)));
        hash.update(bytes_of(std::string_view{data}.substr(split)));
        CHECK(to_hex(hash.finish()) == whole);
    }
}
